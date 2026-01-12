#include <Adafruit_NeoPixel.h>
#include <DHT.h>

//pins
#define DHTPIN 2
#define DHTTYPE DHT22
#define LED_PIN 6
#define NUM_LEDS 12

int vocPin = A0;
int soundPin = A1;

//objects
DHT dht(DHTPIN, DHTTYPE);
Adafruit_NeoPixel strip(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);

//Tu-song weights
float wVOC = 0.55;  
float wHUM = 0.30;   
float wSND = 0.15;   


void setup() {
  Serial.begin(9600);
  dht.begin();
  strip.begin();
  strip.show();
}

void loop() {

  //MQ135 VOC
  int vocRaw = analogRead(vocPin);
  int voc = map(vocRaw, 0, 1023, 0, 100);

  //MAX9814 Sound
  int sndRaw = analogRead(soundPin);
  int snd = map(sndRaw, 0, 1023, 0, 100);

  //DHT22 Humidity / Temp
  float hum = dht.readHumidity();
  float temp = dht.readTemperature();

  if (isnan(hum) || isnan(temp)) {
    Serial.println("DHT Error");
    return;
  }

  //Dog Comfort Index
  float score =
      voc * wVOC +
      hum * wHUM +
      snd * wSND;

  //LED Color Logic
  if (hum > 85) {
    setColor(0, 0, 255);     //very humid → blue
  }
  else if (score < 30) {
    setColor(0, 255, 0);     //green = very comfortable
  }
  else if (score < 55) {
    setColor(255, 200, 0);   //yellow = normal
  }
  else if (score < 75) {
    setColor(255, 120, 0);   //orange = stressful
  }
  else {
    setColor(255, 0, 0);     //red = uncomfortable
  }

  //Serial debug
  Serial.print("VOC: "); Serial.print(voc);
  Serial.print(" | Humid: "); Serial.print(hum);
  Serial.print(" | Sound: "); Serial.print(snd);
  Serial.print(" | Score: "); Serial.println(score);

  delay(300);
}

void setColor(int r, int g, int b) {
  for (int i = 0; i < NUM_LEDS; i++) {
    strip.setPixelColor(i, strip.Color(r, g, b));
  }
  strip.show();
}
