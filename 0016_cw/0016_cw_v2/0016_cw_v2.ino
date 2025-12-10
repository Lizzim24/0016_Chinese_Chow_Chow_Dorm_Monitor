#include <Adafruit_NeoPixel.h>
#include <DHT.h>
#include <SPI.h>
#include <GxEPD2_BW.h>
#include <Fonts/FreeMonoBold12pt7b.h>

//Sensor Pins
#define DHTPIN 2
#define DHTTYPE DHT22
#define LED_PIN 6
#define NUM_LEDS 12

int vocPin = A0;
int soundPin = A1;

//E-Paper Pins
#define EPAPER_CS   3
#define EPAPER_DC   4
#define EPAPER_RST  5
#define EPAPER_BUSY 7

//Objects
DHT dht(DHTPIN, DHTTYPE);
Adafruit_NeoPixel strip(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);

// ★ Waveshare 2.9-inch e-paper Module V2 -> GxEPD2_290_T94
GxEPD2_BW<GxEPD2_290_T94, GxEPD2_290_T94::HEIGHT> display(
  GxEPD2_290_T94(EPAPER_CS, EPAPER_DC, EPAPER_RST, EPAPER_BUSY)
);

//Tu-song weights
float wVOC = 0.55;
float wHUM = 0.30;
float wSND = 0.15;

//Break time
unsigned long lastEPDUpdate = 0;
const unsigned long EPD_INTERVAL = 5000;

//Load data
int   g_voc   = 0;
int   g_snd   = 0;
float g_hum   = 0;
float g_temp  = 0;
float g_score = 0;
String g_status = "GREEN";

void setup() {
  Serial.begin(9600);
  dht.begin();

  strip.begin();
  strip.show();

  // initialize e-paper
  display.init();
  display.setRotation(1);
  display.setFont(&FreeMonoBold12pt7b);
  display.setTextColor(GxEPD_BLACK);

  for (int i = 0; i < 2; i++) {
    display.firstPage();
    do {
      display.fillScreen(GxEPD_WHITE);
    } while (display.nextPage());
  }

  updateEPaper(); 
}

void loop() {

  //MQ135 VOC
  int vocRaw = analogRead(vocPin);
  g_voc = map(vocRaw, 0, 1023, 0, 100);

  //MAX9814 Sound
  int sndRaw = analogRead(soundPin);
  g_snd = map(sndRaw, 0, 1023, 0, 100);

  //DHT22 Humidity / Temp
  g_hum = dht.readHumidity();
  g_temp = dht.readTemperature();

  if (isnan(g_hum) || isnan(g_temp)) {
    Serial.println("DHT22 read error");
    delay(500);
    return;
  }

  //Dog Comfort Index
  g_score =
      g_voc * wVOC +
      g_hum * wHUM +
      g_snd * wSND;

  //LED Color Logic
  if (g_hum > 85) {
    g_status = "WET";
  } else if (g_score < 30) {
    g_status = "GREEN";
  } else if (g_score < 55) {
    g_status = "YELLOW";
  } else if (g_score < 75) {
    g_status = "ORANGE";
  } else {
    g_status = "RED";
  }

  //
  if (g_hum > 85) {
    setColor(0, 0, 255);      //very humid → blue
  }
  else if (g_score < 30) {
    setColor(0, 255, 0);      //green = very comfortable
  }
  else if (g_score < 55) {
    setColor(255, 200, 0);    //yellow = normal
  }
  else if (g_score < 75) {
    setColor(255, 120, 0);    //orange = stressful
  }
  else {
    setColor(255, 0, 0);      //red = uncomfortable
  }

  //Serial debug
  Serial.print("VOC ");
  Serial.print(g_voc);
  Serial.print(" | HUM ");
  Serial.print(g_hum);
  Serial.print(" | SOUND ");
  Serial.print(g_snd);
  Serial.print(" | TEMP ");
  Serial.print(g_temp);
  Serial.print(" | SCORE ");
  Serial.print(g_score);
  Serial.print(" | STATUS ");
  Serial.println(g_status);

  //Update time
  if (millis() - lastEPDUpdate > EPD_INTERVAL) {
    updateEPaper();
    lastEPDUpdate = millis();
  }

  delay(300);
}

//NeoPixel color
void setColor(int r, int g, int b) {
  for (int i = 0; i < NUM_LEDS; i++) {
    strip.setPixelColor(i, strip.Color(r, g, b));
  }
  strip.show();
}

//Upadate e-paper Screean
void updateEPaper() {
  display.firstPage();
  do {
    display.fillScreen(GxEPD_WHITE);

    // title
    display.setCursor(10, 30);
    display.setFont(&FreeMonoBold12pt7b);
    display.println("DOG COMFORT INDEX");

    // index
    display.setCursor(10, 70);
    display.setFont(&FreeMonoBold12pt7b);
    display.print((int)g_score);
    display.print(" (");
    display.print(g_status);
    display.println(")");

    // VOC
    display.setFont(&FreeMonoBold12pt7b);
    display.setCursor(10, 120);
    display.print("VOC: ");
    display.println(g_voc);

    // Humidity
    display.setCursor(10, 150);
    display.print("Hum: ");
    display.print(g_hum, 1);
    display.println("%");

    // Noise
    display.setCursor(10, 180);
    display.print("Noise: ");
    display.println(g_snd);

    // Temperature
    display.setCursor(10, 210);
    display.print("Temp: ");
    display.print(g_temp, 1);
    display.println(" C");

    // Breed
    display.setCursor(10, 240);
    display.print("Breed: Tu-song");

  } while (display.nextPage());
}
