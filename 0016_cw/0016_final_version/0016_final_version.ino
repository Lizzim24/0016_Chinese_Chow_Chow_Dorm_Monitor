#include "DHT.h"
#include <GxEPD2_BW.h>
#include <Adafruit_GFX.h>
#include <math.h>

#define PIN_MQ135 A1
#define PIN_SOUND A0
#define PIN_DHT 2
#define PIN_LED_R 3
#define PIN_LED_G 4
#define PIN_LED_B 5

#define EPAPER_CS 10
#define EPAPER_DC 11
#define EPAPER_RST 12
#define EPAPER_BUSY 13

#define DHTTYPE DHT22
DHT dht(PIN_DHT, DHTTYPE);

#define LED_SLEEP_AFTER_MS 30000UL
#define LED_MAX_VAL 200

const int RAW_QUIET = 1380;
const int RAW_LOUD = 1800;
const float DB_QUIET = 35.0;
const float DB_LOUD = 80.0;

#define RL_VALUE 10.0           
#define RO_CLEAN_AIR_FACTOR 3.6  
float MQ135_RO = 10.0;          

GxEPD2_BW<GxEPD2_290_T94, GxEPD2_290_T94::HEIGHT> display(
  GxEPD2_290_T94(EPAPER_CS, EPAPER_DC, EPAPER_RST, EPAPER_BUSY));

struct AirQualityData {
  int rawValue;
  int aqi;           
  float co2_ppm;     
  float nh3_ppm;     
  float score;       
  const char *level; 
  const char *color; 
};


struct SensorData {
  float temp;
  float humidity;
  AirQualityData airQuality;  
  float soundDB;
  float last_dB;

  float tempScore;
  float humScore;
  float airScore;
  float noiseScore;
  float comfortIndex;

  float lastTemp;
  float lastHum;
  int lastAQI;
  float lastComfort;
};

SensorData sensor = { 0 };

unsigned long lastSensorRead = 0;
unsigned long lastLedActiveTime = 0;
unsigned long lastEpaperUpdate = 0;

const unsigned long SENSOR_INTERVAL = 2000;
const unsigned long EPAPER_INTERVAL = 300000;
const unsigned long EPAPER_MIN_INTERVAL = 30000;

float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

float smooth(float prev, float cur, float alpha = 0.2) {
  return prev * (1.0 - alpha) + cur * alpha;
}

void calibrateMQ135() {
  Serial.println(F("\n╔════════════════════════════════════╗"));
  Serial.println(F("║   MQ135 Calibration Required      ║"));
  Serial.println(F("╠════════════════════════════════════╣"));
  Serial.println(F("║ Please ensure:                    ║"));
  Serial.println(F("║ 1. Windows are open               ║"));
  Serial.println(F("║ 2. Good ventilation               ║"));
  Serial.println(F("║ 3. No smoking/cooking nearby      ║"));
  Serial.println(F("╚════════════════════════════════════╝"));
  Serial.println(F("Calibrating in 10 seconds..."));

  for (int i = 10; i > 0; i--) {
    Serial.print(i);
    Serial.print("... ");
    delay(1000);
  }
  Serial.println(F("\nCalibrating (sampling 50 times)..."));

  float rsSum = 0;
  for (int i = 0; i < 50; i++) {
    int raw = analogRead(PIN_MQ135);
    float voltage = (raw / 4095.0) * 3.3 * 2.0;
    if (voltage < 0.01) voltage = 0.01;
    float rs = ((3.3 - voltage) / voltage) * RL_VALUE;
    rsSum += rs;
    delay(100);

    if (i % 10 == 0) {
      Serial.print(".");
    }
  }

  float avgRS = rsSum / 50.0;
  MQ135_RO = avgRS / RO_CLEAN_AIR_FACTOR;

  Serial.println();
  Serial.print(F("✓ Calibration complete! RO = "));
  Serial.print(MQ135_RO, 2);
  Serial.println(F(" kΩ"));
  Serial.println(F("═══════════════════════════════════\n"));
}

AirQualityData readMQ135_Enhanced() {
  AirQualityData result;
  result.rawValue = analogRead(PIN_MQ135);

  float voltage = (result.rawValue / 4095.0) * 3.3 * 2.0;
  if (voltage < 0.01) voltage = 0.01;

  float rs = ((3.3 - voltage) / voltage) * RL_VALUE;
  float ratio = rs / MQ135_RO;  // Rs/Ro

  result.co2_ppm = 116.6 * pow(ratio, -2.6);
  result.co2_ppm = constrain(result.co2_ppm, 300, 5000);

  result.nh3_ppm = 102.2 * pow(ratio, -2.47);

  if (result.nh3_ppm < 0.1) result.nh3_ppm = 0.1;
  result.nh3_ppm = constrain(result.nh3_ppm, 0.1, 100.0);

  float targetGas = result.nh3_ppm;

  if (targetGas < 1.0) {
    result.aqi = mapFloat(targetGas, 0.1, 1.0, 0, 50);
    result.level = "Clean";
    result.color = "Green"; 
  } else if (targetGas < 3.0) {
    result.aqi = mapFloat(targetGas, 1.0, 3.0, 51, 100);
    result.level = "Scented"; 
    result.color = "Yellow"; 
  } else if (targetGas < 5.0) {
    result.aqi = mapFloat(targetGas, 3.0, 5.0, 101, 200);
    result.level = "Dirty";
    result.color = "Orange";  
  } else {
    result.aqi = mapFloat(targetGas, 5.0, 20.0, 201, 500);
    result.aqi = constrain(result.aqi, 201, 500);
    result.level = "Toxic";
    result.color = "Red";
  }

  if (result.aqi <= 50) {
    result.score = 100;
  } else {
    result.score = max(0.0, 100.0 - (result.aqi - 50.0) * 0.8);
  }

  return result;
}

float readSound_dB() {
  int raw = analogRead(PIN_SOUND);
  raw = constrain(raw, RAW_QUIET, RAW_LOUD);

  float db = mapFloat((float)raw, (float)RAW_QUIET, (float)RAW_LOUD,
                      DB_QUIET, DB_LOUD);

  sensor.last_dB = smooth(sensor.last_dB, db, 0.2);
  return sensor.last_dB;
}

bool readDHT22(float &temp, float &hum) {
  temp = dht.readTemperature();
  hum = dht.readHumidity();

  if (isnan(temp) || isnan(hum)) {
    Serial.println(F("DHT22 read failed!"));
    return false;
  }
  return true;
}

float scoreTemperature(float t) {
  const float HARD_MIN = 5, GOOD_MIN = 15, GOOD_MAX = 22, HARD_MAX = 28;

  if (t <= HARD_MIN || t >= HARD_MAX) return 0;
  if (t >= GOOD_MIN && t <= GOOD_MAX) return 100;

  if (t < GOOD_MIN) {
    return 100 * (t - HARD_MIN) / (GOOD_MIN - HARD_MIN);
  } else {
    return 100 * (HARD_MAX - t) / (HARD_MAX - GOOD_MAX);
  }
}

float scoreHumidity(float h) {
  const float HARD_MIN = 20, GOOD_MIN = 40, GOOD_MAX = 55, HARD_MAX = 75;

  if (h <= HARD_MIN || h >= HARD_MAX) return 0;
  if (h >= GOOD_MIN && h <= GOOD_MAX) return 100;

  if (h < GOOD_MIN) {
    return 100 * (h - HARD_MIN) / (GOOD_MIN - HARD_MIN);
  } else {
    return 100 * (HARD_MAX - h) / (HARD_MAX - GOOD_MAX);
  }
}

float noiseComfortScore(float dB) {
  if (dB <= 45) return 100;
  if (dB >= 80) return 0;

  if (dB <= 60) {
    return 100 - (dB - 45) / 15.0 * 30;
  } else if (dB <= 75) {
    return 70 - (dB - 60) / 15.0 * 50;
  } else {
    return 20 - (dB - 75) / 5.0 * 20;
  }
}

float computeComfortIndex(float tempScore, float humScore,
                          float airScore, float noiseScore) {
  return 0.30 * tempScore + 0.30 * humScore + 0.20 * airScore + 0.20 * noiseScore;
}

void setRGB(uint8_t r, uint8_t g, uint8_t b) {
  r = constrain(r, 0, LED_MAX_VAL);
  g = constrain(g, 0, LED_MAX_VAL);
  b = constrain(b, 0, LED_MAX_VAL);

  float scale = 255.0 / LED_MAX_VAL;

  analogWrite(PIN_LED_R, 255 - (uint8_t)(r * scale));
  analogWrite(PIN_LED_G, 255 - (uint8_t)(g * scale));
  analogWrite(PIN_LED_B, 255 - (uint8_t)(b * scale));
}

void showComfortRGB(float comfort) {
  uint8_t r = 0, g = 0, b = 0;

  if (comfort >= 70) {
    g = LED_MAX_VAL;
  } else if (comfort >= 40) {
    r = LED_MAX_VAL;
    g = (uint8_t)(LED_MAX_VAL * 0.7);
  } else {
    r = LED_MAX_VAL;
  }

  setRGB(r, g, b);
  lastLedActiveTime = millis();
}

void turnOffRGB() {
  setRGB(0, 0, 0);
}

void drawDogIcon(int16_t x, int16_t y) {
  int16_t cx = x + 8, cy = y + 8;
  display.drawCircle(cx, cy, 7, GxEPD_WHITE);
  display.fillCircle(cx - 5, cy - 5, 2, GxEPD_WHITE);
  display.fillCircle(cx + 5, cy - 5, 2, GxEPD_WHITE);
  display.fillCircle(cx - 3, cy - 1, 1, GxEPD_WHITE);
  display.fillCircle(cx + 3, cy - 1, 1, GxEPD_WHITE);
  display.fillCircle(cx, cy + 2, 1, GxEPD_WHITE);
  display.drawLine(cx - 2, cy + 4, cx, cy + 5, GxEPD_WHITE);
  display.drawLine(cx, cy + 5, cx + 2, cy + 4, GxEPD_WHITE);
}

void drawTempIcon(int16_t x, int16_t y) {
  display.fillCircle(x + 10, y + 15, 3, GxEPD_BLACK);
  display.fillRect(x + 9, y + 5, 2, 10, GxEPD_BLACK);
  display.drawCircle(x + 10, y + 5, 3, GxEPD_BLACK);
}

void drawHumidityIcon(int16_t x, int16_t y) {
  display.fillTriangle(x + 10, y + 2, x + 5, y + 12, x + 15, y + 12, GxEPD_BLACK);
  display.fillCircle(x + 10, y + 13, 5, GxEPD_BLACK);
  display.fillCircle(x + 10, y + 11, 3, GxEPD_WHITE);
}

void drawAirIcon(int16_t x, int16_t y) {
  display.fillCircle(x + 6, y + 10, 3, GxEPD_BLACK);
  display.fillCircle(x + 10, y + 8, 4, GxEPD_BLACK);
  display.fillCircle(x + 14, y + 10, 3, GxEPD_BLACK);
  display.fillRect(x + 6, y + 10, 8, 4, GxEPD_BLACK);
}

void drawSoundIcon(int16_t x, int16_t y) {
  display.fillRect(x + 4, y + 8, 3, 5, GxEPD_BLACK);
  display.fillTriangle(x + 7, y + 8, x + 12, y + 4, x + 12, y + 16, GxEPD_BLACK);
  display.drawLine(x + 14, y + 6, x + 16, y + 4, GxEPD_BLACK);
  display.drawLine(x + 14, y + 10, x + 16, y + 10, GxEPD_BLACK);
  display.drawLine(x + 14, y + 14, x + 16, y + 16, GxEPD_BLACK);
}

void drawComfortFace(int16_t x, int16_t y, float comfort) {
  int16_t cx = x + 9, cy = y + 9;

  display.drawCircle(cx, cy, 8, GxEPD_WHITE);
  display.fillCircle(cx - 3, cy - 2, 1, GxEPD_WHITE);
  display.fillCircle(cx + 3, cy - 2, 1, GxEPD_WHITE);

  if (comfort >= 70) {
    display.drawLine(cx - 4, cy + 2, cx - 2, cy + 4, GxEPD_WHITE);
    display.drawLine(cx - 2, cy + 4, cx + 2, cy + 4, GxEPD_WHITE);
    display.drawLine(cx + 2, cy + 4, cx + 4, cy + 2, GxEPD_WHITE);
  } else if (comfort >= 40) {
    display.drawLine(cx - 4, cy + 3, cx + 4, cy + 3, GxEPD_WHITE);
  } else {
    display.drawLine(cx - 4, cy + 5, cx - 2, cy + 3, GxEPD_WHITE);
    display.drawLine(cx - 2, cy + 3, cx + 2, cy + 3, GxEPD_WHITE);
    display.drawLine(cx + 2, cy + 3, cx + 4, cy + 5, GxEPD_WHITE);
  }
}

void epaperShowStatus(const SensorData &data) {
  display.setFullWindow();
  display.clearScreen();
  delay(100);

  display.firstPage();

  do {
    display.fillScreen(GxEPD_WHITE);

    display.fillRect(0, 0, 296, 2, GxEPD_WHITE);
    display.fillRect(0, 2, 296, 26, GxEPD_BLACK);
    drawDogIcon(3, 7);
    display.setTextColor(GxEPD_WHITE);
    display.setTextSize(1);
    display.setCursor(22, 12);
    display.println("Chinese Chow Chow Dorm Monitor");
    display.setTextColor(GxEPD_BLACK);

    int cardY = 32;
    int cardH = 60;
    int cardW = 70;
    int spacing = 4;
    int startX = 4;

    int card1X = startX;
    display.drawRect(card1X, cardY, cardW, cardH, GxEPD_BLACK);
    drawTempIcon(card1X + 25, cardY + 5);
    display.setCursor(card1X + 20, cardY + 27);
    display.println("TEMP");
    display.setCursor(card1X + 6, cardY + 43);
    display.setTextSize(2);
    display.print(data.temp, 1);
    display.setTextSize(1);
    display.setCursor(card1X + cardW - 14, cardY + cardH - 9);
    display.println("C");

    int card2X = card1X + cardW + spacing;
    display.drawRect(card2X, cardY, cardW, cardH, GxEPD_BLACK);
    drawHumidityIcon(card2X + 25, cardY + 3);
    display.setCursor(card2X + 24, cardY + 27);
    display.println("HUM");
    display.setCursor(card2X + 6, cardY + 43);
    display.setTextSize(2);
    display.print(data.humidity, 1);
    display.setTextSize(1);
    display.setCursor(card2X + cardW - 14, cardY + cardH - 9);
    display.println("%");

    int card3X = card2X + cardW + spacing;
    display.drawRect(card3X, cardY, cardW, cardH, GxEPD_BLACK);
    drawAirIcon(card3X + 25, cardY + 5);
    display.setCursor(card3X + 24, cardY + 27);
    display.println("AIR");
    display.setCursor(card3X + 10, cardY + 43);
    display.setTextSize(2);
    display.println(data.airQuality.aqi);
    display.setTextSize(1);
    display.setCursor(card3X + cardW - 22, cardY + cardH - 9);
    display.println("AQI");

    int card4X = card3X + cardW + spacing;
    display.drawRect(card4X, cardY, cardW, cardH, GxEPD_BLACK);
    drawSoundIcon(card4X + 25, cardY + 5);
    display.setCursor(card4X + 18, cardY + 27);
    display.println("SOUND");
    display.setCursor(card4X + 6, cardY + 43);
    display.setTextSize(2);
    display.print(data.soundDB, 1);
    display.setTextSize(1);
    display.setCursor(card4X + cardW - 16, cardY + cardH - 9);
    display.println("dB");

    int bottomY = 96;
    int bottomH = 32;
    display.fillRect(0, bottomY, 296, bottomH, GxEPD_BLACK);
    display.setTextColor(GxEPD_WHITE);

    drawComfortFace(4, bottomY + 5, data.comfortIndex);

    display.setCursor(24, bottomY + 12);
    display.println("Comfort: ");
    display.setCursor(78, bottomY + 12);
    display.print(data.comfortIndex, 1);
    display.print("/100");

    const char *statusText;
    if (data.comfortIndex >= 70) {
      statusText = "HAPPY & Comfortable";
    } else if (data.comfortIndex >= 40) {
      statusText = "MEDIUM";
    } else {
      statusText = "BAD";
    }

    display.setCursor(150, bottomY + 12);
    display.println(statusText);

    display.setTextColor(GxEPD_BLACK);

  } while (display.nextPage());

  Serial.println(F("✓ E-Paper updated"));
}

void updateLastValues(SensorData &data) {
  data.lastTemp = data.temp;
  data.lastHum = data.humidity;
  data.lastAQI = data.airQuality.aqi;
  data.lastComfort = data.comfortIndex;
}

bool hasSignificantChange(const SensorData &data) {
  bool tempChanged = fabs(data.temp - data.lastTemp) > 0.5;
  bool humChanged = fabs(data.humidity - data.lastHum) > 3.0;
  bool airChanged = abs(data.airQuality.aqi - data.lastAQI) > 20;
  bool comfortChanged = fabs(data.comfortIndex - data.lastComfort) > 5.0;

  return tempChanged || humChanged || airChanged || comfortChanged;
}

void printSmartSuggestions(const SensorData &data) {
  bool hasSuggestion = false;

  Serial.println(F("─────────────────────────────────"));
  Serial.println(F("💡 Smart Suggestions (Chow-Specific):"));

  if (data.airQuality.nh3_ppm > 5.0) {
    Serial.println(F("  ⚠️ TOXIC AIR! Ammonia/Chemicals high. Evacuate pet!"));
    hasSuggestion = true;
  } else if (data.airQuality.nh3_ppm > 3.0) {
    Serial.println(F("  🧹 Strong Odor! Check litter box or reduce air freshener."));
    hasSuggestion = true;
  } else if (data.airQuality.nh3_ppm > 1.0) {
    Serial.println(F("  💨 Ventilation needed. Air is stale/scented."));
    hasSuggestion = true;
  }

  if (data.temp < 15) {
    Serial.println(F("  🌡️  Too cold! Turn on heating."));
    hasSuggestion = true;
  } else if (data.temp > 24) {
    Serial.println(F("  ❄️  Too hot! Turn on cooling or fan."));
    hasSuggestion = true;
  }

  if (data.humidity < 35) {
    Serial.println(F("  💧 Too dry! Use humidifier."));
    hasSuggestion = true;
  } else if (data.humidity > 60) {
    Serial.println(F("  💨 Too humid! Use dehumidifier or ventilate."));
    hasSuggestion = true;
  }

  if (data.soundDB > 65) {
    Serial.println(F("  🔇 Noise level high! Check for disturbances."));
    hasSuggestion = true;
  }

  if (!hasSuggestion) {
    Serial.println(F("  ✓ Environment is perfect for your Chow Chow."));
  }
  Serial.println(F("─────────────────────────────────"));
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) {}

  Serial.println(F("\n╔════════════════════════════════════╗"));
  Serial.println(F("║ Chinese Chow Chow Comfort Monitor ║"));
  Serial.println(F("║           Version 4.0              ║"));
  Serial.println(F("║   Enhanced Air Quality Edition     ║"));
  Serial.println(F("╚════════════════════════════════════╝\n"));

  analogReadResolution(12);
  dht.begin();

  pinMode(PIN_LED_R, OUTPUT);
  pinMode(PIN_LED_G, OUTPUT);
  pinMode(PIN_LED_B, OUTPUT);
  turnOffRGB();

  display.init(115200);
  display.setRotation(1);
  display.setTextColor(GxEPD_BLACK);

  Serial.println(F("Initializing MQ135 sensor..."));
  Serial.println(F("⚠️  IMPORTANT: Sensor needs 24-48h preheat for best accuracy"));
  Serial.println(F("For quick start, we'll calibrate now (less accurate)"));
  Serial.print(F("Skip calibration? (Sensor must be in clean air) [Y/N]: "));

  unsigned long waitStart = millis();
  bool skipCalib = false;
  while (millis() - waitStart < 5000) {
    if (Serial.available()) {
      char c = Serial.read();
      if (c == 'Y' || c == 'y') {
        skipCalib = true;
        Serial.println(F("Y - Skipped"));
        Serial.println(F("Using default RO = 10.0 kΩ"));
        break;
      }
    }
  }

  if (!skipCalib) {
    Serial.println(F("N - Starting calibration..."));
    calibrateMQ135();
  }

  Serial.println(F("\n✓ System initialized!\n"));

  if (readDHT22(sensor.temp, sensor.humidity)) {
    sensor.airQuality = readMQ135_Enhanced();
    sensor.soundDB = readSound_dB();
    sensor.tempScore = scoreTemperature(sensor.temp);
    sensor.humScore = scoreHumidity(sensor.humidity);
    sensor.airScore = sensor.airQuality.score;
    sensor.noiseScore = noiseComfortScore(sensor.soundDB);
    sensor.comfortIndex = computeComfortIndex(sensor.tempScore,
                                              sensor.humScore,
                                              sensor.airScore,
                                              sensor.noiseScore);
    updateLastValues(sensor);
    epaperShowStatus(sensor);
    showComfortRGB(sensor.comfortIndex);
  }
}

void loop() {
  unsigned long now = millis();

  if (now - lastLedActiveTime > LED_SLEEP_AFTER_MS) {
    turnOffRGB();
  }

  if (now - lastSensorRead >= SENSOR_INTERVAL) {
    lastSensorRead = now;

    if (!readDHT22(sensor.temp, sensor.humidity)) {
      sensor.temp = sensor.lastTemp;
      sensor.humidity = sensor.lastHum;
    }

    sensor.airQuality = readMQ135_Enhanced();
    sensor.soundDB = readSound_dB();

    sensor.tempScore = scoreTemperature(sensor.temp);
    sensor.humScore = scoreHumidity(sensor.humidity);
    sensor.airScore = sensor.airQuality.score;
    sensor.noiseScore = noiseComfortScore(sensor.soundDB);
    sensor.comfortIndex = computeComfortIndex(sensor.tempScore,
                                              sensor.humScore,
                                              sensor.airScore,
                                              sensor.noiseScore);


    bool isEmergency = false;

    if (sensor.airQuality.nh3_ppm > 5.0) {
      isEmergency = true;
      Serial.println(F("🚨 ALERT: High Ammonia detected! Overriding status to RED."));
    }

    if (sensor.soundDB > 75) {
      isEmergency = true;
      Serial.println(F("🚨 ALERT: Extreme Noise! Overriding status to RED."));
    }

    float voltage = (analogRead(PIN_MQ135) / 4095.0) * 3.3 * 2.0;

    Serial.print("DEBUG VOLTAGE: ");
    Serial.println(voltage);

    Serial.println(F("\n╔═══════════════════════════════════╗"));
    Serial.println(F("║     ENVIRONMENT MONITORING        ║"));
    Serial.println(F("╠═══════════════════════════════════╣"));

    Serial.print(F("║ Temperature:  "));
    Serial.print(sensor.temp, 1);
    Serial.print(F("°C"));
    Serial.print(F("  (Score: "));
    Serial.print(sensor.tempScore, 0);
    Serial.println(F(")    ║"));

    Serial.print(F("║ Humidity:     "));
    Serial.print(sensor.humidity, 1);
    Serial.print(F("%"));
    Serial.print(F("   (Score: "));
    Serial.print(sensor.humScore, 0);
    Serial.println(F(")    ║"));

    Serial.println(F("╠═══════════════════════════════════╣"));
    Serial.print(F("║ Air Quality:  AQI "));
    Serial.print(sensor.airQuality.aqi);
    Serial.print(F(" ("));
    Serial.print(sensor.airQuality.level);
    Serial.println(F(")  ║"));

    Serial.print(F("║   └─ CO₂:     "));
    Serial.print(sensor.airQuality.co2_ppm, 0);
    Serial.println(F(" ppm           ║"));

    Serial.print(F("║   └─ NH₃:     "));
    Serial.print(sensor.airQuality.nh3_ppm, 1);
    Serial.println(F(" ppm (Pet)      ║"));

    Serial.print(F("║   └─ Score:   "));
    Serial.print(sensor.airScore, 0);
    Serial.println(F(" / 100          ║"));

    Serial.println(F("╠═══════════════════════════════════╣"));
    Serial.print(F("║ Sound Level:  "));
    Serial.print(sensor.soundDB, 1);
    Serial.print(F(" dB"));
    Serial.print(F("  (Score: "));
    Serial.print(sensor.noiseScore, 0);
    Serial.println(F(") ║"));

    Serial.println(F("╠═══════════════════════════════════╣"));
    Serial.print(F("║ COMFORT INDEX: "));
    Serial.print(sensor.comfortIndex, 1);
    Serial.print(F(" / 100"));

    if (sensor.comfortIndex >= 70) {
      Serial.println(F("  ✓     ║"));
    } else if (sensor.comfortIndex >= 40) {
      Serial.println(F("  ~     ║"));
    } else {
      Serial.println(F("  ✗     ║"));
    }

    Serial.println(F("╚═══════════════════════════════════╝"));

    printSmartSuggestions(sensor);

    unsigned long timeSinceLastUpdate = now - lastEpaperUpdate;
    bool hasChanged = hasSignificantChange(sensor);
    bool timeForScheduledUpdate = (timeSinceLastUpdate >= EPAPER_INTERVAL);
    bool cooledDown = (timeSinceLastUpdate >= EPAPER_MIN_INTERVAL);

    if (hasChanged) {
      showComfortRGB(sensor.comfortIndex); 
      Serial.println(F("🔔 Change detected! LED Activated."));
    }

    if (isEmergency) {
      setRGB(200, 0, 0);
      lastLedActiveTime = millis(); 
    } else if (hasChanged) {
      showComfortRGB(sensor.comfortIndex);
    }

    if (timeForScheduledUpdate || (hasChanged && cooledDown)) {
      if (hasChanged && !timeForScheduledUpdate) {
        Serial.println(F("📊 Significant change detected, updating display..."));
      } else {
        Serial.println(F("⏰ Scheduled display update..."));
      }

      epaperShowStatus(sensor);
      updateLastValues(sensor);
      lastEpaperUpdate = now;
    } else if (hasChanged && !cooledDown) {
      Serial.print(F("⏳ Change detected but cooling down... "));
      Serial.print((EPAPER_MIN_INTERVAL - timeSinceLastUpdate) / 1000);
      Serial.println(F("s remaining"));
    }
  }

  delay(100);
}