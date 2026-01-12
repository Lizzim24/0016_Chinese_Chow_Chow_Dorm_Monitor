/****************************************************
 * Chow Chow Comfort Monitor - MKR WiFi 1010 Version V3
 * 板子：Arduino MKR WiFi 1010（3.3V 逻辑）
 * 传感器：MQ135 + DHT22 + MAX9814 + Neopixel(8) + 2.9" e-paper (296x128)
 *
 * 保护点：
 *  - Neopixel 全局限亮度 MAX_BRIGHTNESS
 *  - 显示逻辑避免全白全亮，只用偏暗的红/黄/绿
 *  - 自动熄灯（LED_SLEEP_AFTER_MS）降低平均电流
 *  - MAX9814 用 3.3V 供电
 *  - 所有模拟输入使用 analogReadResolution(12)（0~4095）
 ****************************************************/

#include <Adafruit_NeoPixel.h>
#include "DHT.h"

#include <GxEPD2_BW.h>
#include <Adafruit_GFX.h>

// ================= 引脚定义 =================
// 模拟输入
#define PIN_MQ135 A1  // MQ135 模拟输出
#define PIN_SOUND A0  // MAX9814 模拟输出

// 数字 IO
#define PIN_DHT 2       // DHT22 数据脚
#define PIN_NEOPIXEL 6  // Neopixel DIN

// e-paper SPI 控制脚（MKR 的 SPI SCK/MOSI 走板子上的专用 SPI 引脚）
#define EPAPER_CS 10
#define EPAPER_DC 11
#define EPAPER_RST 12
#define EPAPER_BUSY 13

// ================= DHT 配置 =================
#define DHTTYPE DHT22
DHT dht(PIN_DHT, DHTTYPE);

// ================= Neopixel 配置 =================
#define NUM_PIXELS 8
#define MAX_BRIGHTNESS 40           // 电流保护：0~255，建议 20~60
#define LED_SLEEP_AFTER_MS 30000UL  // 30s 无更新则自动熄灯

Adafruit_NeoPixel strip(NUM_PIXELS, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);

// ================= MAX9814 声音 → dB 映射配置 =================
// 这些值来自你之前在 MKR1010 上测试的 raw 范围（12bit ADC 0~4095）
// 可以根据实际宿舍环境再微调
int RAW_QUIET = 1380;  // 宿舍安静时的 analogRead(A0)
int RAW_LOUD = 1800;   // 有人说话/稍吵时 analogRead(A0)

// 映射到的 dB 范围（参考宠物噪音阈值）
float DB_QUIET = 35.0;  // 安静房间约 35 dB
float DB_LOUD = 80.0;   // 比较吵的环境约 80 dB

float last_dB = 40.0;  // 用于平滑滤波的上一帧 dB

// ================= e-paper 显示配置 =================
// 这里假设你的 2.9" 296x128 屏是 GxEPD2_290_T94 型号
// 如果编译报错，请去 GxEPD2 示例中确认正确的类名替换这一行
GxEPD2_BW<GxEPD2_290_T94, GxEPD2_290_T94::HEIGHT> display(
  GxEPD2_290_T94(EPAPER_CS, EPAPER_DC, EPAPER_RST, EPAPER_BUSY));

// ================= 全局变量 =================
unsigned long lastSensorRead = 0;
unsigned long sensorInterval = 2000;  // 每 2 秒更新一次传感器
unsigned long lastLedActiveTime = 0;

float lastTemp = 0;
float lastHum = 0;
int lastMQ135 = 0;

// ================= 工具函数 =================

// 浮点映射
float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// 简单平滑滤波
float smooth(float prev, float cur, float alpha = 0.2) {
  return prev * (1 - alpha) + cur * alpha;
}

// ========== 1. 声音：MAX9814 → dB → 噪音舒适度评分 ==========

// 读取声传感器原始值，并映射到大约 35~80 dB
float readSound_dB() {
  int raw = analogRead(PIN_SOUND);

  // 限制范围，去掉极端异常值
  raw = constrain(raw, RAW_QUIET, RAW_LOUD);

  // 映射到 dB
  float db = mapFloat((float)raw, (float)RAW_QUIET, (float)RAW_LOUD,
                      DB_QUIET, DB_LOUD);

  // 平滑一下，让变化不要太跳
  last_dB = smooth(last_dB, db, 0.2);

  return last_dB;
}

// 噪音舒适度：0~100（越高越舒适）
float noiseComfortScore(float dB) {
  if (dB <= 45) return 100;  // 非常安静
  if (dB >= 80) return 0;    // 非常吵

  if (dB <= 60) {
    // 45 → 60 dB：舒适度从 100 降到 70
    float ratio = (dB - 45) / (60 - 45);
    return 100 - ratio * 30;  // 100 → 70
  } else if (dB <= 75) {
    // 60 → 75 dB：舒适度从 70 降到 20
    float ratio = (dB - 60) / (75 - 60);
    return 70 - ratio * 50;  // 70 → 20
  } else {
    // 75 → 80 dB：20 → 0
    float ratio = (dB - 75) / (80 - 75);
    return 20 - ratio * 20;  // 20 → 0
  }
}

// ========== 2. 温度 / 湿度 / 空气质量评分（0~100） ==========

// 温度舒适度：松狮犬 15–22°C 最佳，22–25 还行，>26 不适
float scoreTemperature(float t) {
  const float HARD_MIN = 5;
  const float GOOD_MIN = 15;
  const float GOOD_MAX = 22;
  const float HARD_MAX = 28;

  if (t <= HARD_MIN || t >= HARD_MAX) return 0;
  if (t >= GOOD_MIN && t <= GOOD_MAX) return 100;

  if (t < GOOD_MIN) {
    float ratio = (t - HARD_MIN) / (GOOD_MIN - HARD_MIN);
    return 100 * ratio;
  } else {  // t > GOOD_MAX && t < HARD_MAX
    float ratio = (HARD_MAX - t) / (HARD_MAX - GOOD_MAX);
    return 100 * ratio;
  }
}

// 湿度舒适度：40–55% 最佳，55–65 一般，>65 不适
float scoreHumidity(float h) {
  const float HARD_MIN = 20;
  const float GOOD_MIN = 40;
  const float GOOD_MAX = 55;
  const float HARD_MAX = 75;

  if (h <= HARD_MIN || h >= HARD_MAX) return 0;
  if (h >= GOOD_MIN && h <= GOOD_MAX) return 100;

  if (h < GOOD_MIN) {
    float ratio = (h - HARD_MIN) / (GOOD_MIN - HARD_MIN);
    return 100 * ratio;
  } else {  // h > GOOD_MAX && h < HARD_MAX
    float ratio = (HARD_MAX - h) / (HARD_MAX - GOOD_MAX);
    return 100 * ratio;
  }
}

// 空气质量评分（MKR 是 12bit ADC，0~4095，阈值按 UNO 版放大约 4 倍）
// 这里先给一个大概范围，之后你可以根据实际 MQ135 的 baseline 再调
float scoreAirQuality(int mqRaw) {
  const float HARD_MIN = 400;   // 理论较干净时下限
  const float GOOD_MIN = 600;   // 很干净
  const float GOOD_MAX = 1040;  // 普通宿舍
  const float HARD_MAX = 2000;  // 很差 / 有烟雾/喷雾

  if (mqRaw <= GOOD_MIN) return 100;
  if (mqRaw >= HARD_MAX) return 0;

  if (mqRaw <= GOOD_MAX) {
    // GOOD_MIN~GOOD_MAX: 100 → 80
    float ratio = (mqRaw - GOOD_MIN) / (GOOD_MAX - GOOD_MIN);
    return 100 - 20 * ratio;
  } else {
    // GOOD_MAX~HARD_MAX: 80 → 0
    float ratio = (mqRaw - GOOD_MAX) / (HARD_MAX - GOOD_MAX);
    return 80 * (1.0 - ratio);
  }
}

// 综合舒适度（Chow Chow Comfort Index）：0~100
float computeComfortIndex(float tempScore, float humScore,
                          float airScore, float noiseScore) {
  // 温湿度权重大一点，空气和噪音次之
  return 0.30 * tempScore
         + 0.30 * humScore
         + 0.20 * airScore
         + 0.20 * noiseScore;
}

// ========== 3. Neopixel 显示（带“电流保护”设计） ==========

// 舒适度 0~100 → 亮灯个数 0~NUM_PIXELS
int comfortToLedCount(float comfort) {
  int n = (int)round(comfort / 100.0 * NUM_PIXELS);
  if (n < 0) n = 0;
  if (n > NUM_PIXELS) n = NUM_PIXELS;
  return n;
}

// 舒适度 → 颜色（使用低饱和度防止电流过大）
uint32_t comfortToColor(float comfort) {
  // 70~100: 绿色   (0, 80, 0)
  // 40~70 : 黄色   (80, 50, 0)
  // 0~40  : 红色   (80, 0, 0)
  if (comfort >= 70) {
    return strip.Color(0, 80, 0);
  } else if (comfort >= 40) {
    return strip.Color(80, 50, 0);
  } else {
    return strip.Color(80, 0, 0);
  }
}

// 把舒适度显示在灯条上（左边亮，右边灭）
void showComfortOnStrip(float comfort) {
  int count = comfortToLedCount(comfort);
  uint32_t col = comfortToColor(comfort);

  for (int i = 0; i < NUM_PIXELS; i++) {
    if (i < count) {
      strip.setPixelColor(i, col);
    } else {
      strip.setPixelColor(i, 0);
    }
  }
  strip.show();
  lastLedActiveTime = millis();
}

// 熄灭所有灯（自动休眠用）
void turnOffStrip() {
  for (int i = 0; i < NUM_PIXELS; i++) {
    strip.setPixelColor(i, 0);
  }
  strip.show();
}

// ========== 4. e-paper 显示 ==========

void epaperInit() {
  display.init(115200);    // Waveshare 推荐波特率
  display.setRotation(1);  // 竖屏方向最正确
  display.setFullWindow();
  display.setTextColor(GxEPD_BLACK);
}
// 在 (x, y) 附近画一个小一点的狗狗头像（约 20x20）
void drawDogIcon(int16_t x, int16_t y) {
  int16_t headR = 8;    // 脸半径变小
  int16_t cx = x + 10;  // 中心位置也相应缩小
  int16_t cy = y + 10;

  // 头（圆脸）
  display.drawCircle(cx, cy, headR, GxEPD_BLACK);

  // 耳朵（半径和偏移都缩小）
  display.fillCircle(cx - 7, cy - 7, 3, GxEPD_BLACK);  // 左耳
  display.fillCircle(cx + 7, cy - 7, 3, GxEPD_BLACK);  // 右耳

  // 眼睛
  display.fillCircle(cx - 3, cy - 1, 1, GxEPD_BLACK);
  display.fillCircle(cx + 3, cy - 1, 1, GxEPD_BLACK);

  // 鼻子
  display.fillCircle(cx, cy + 2, 1, GxEPD_BLACK);

  // 嘴巴（短一点）
  display.drawLine(cx - 4, cy + 3, cx - 1, cy + 5, GxEPD_BLACK);
  display.drawLine(cx + 1, cy + 5, cx + 4, cy + 3, GxEPD_BLACK);
}


// 在屏幕上显示所有数据：温度、湿度、空气、噪音、四个 score + 总舒适度
void epaperShowStatus(float t, float h, int mq, float dB,
                      float sTemp, float sHum, float sAir, float sNoise,
                      float comfort) {

  display.setFullWindow();
  display.firstPage();
  do {
    display.fillScreen(GxEPD_WHITE);

    // ===== 行高控制 =====
    int y = 12;             // 第一行位置
    const int lineH = 11;   // 每行实际占用的高度（字体 1 大概 8px，这里加 3px）

    // ===== 顶部图标 + 单行标题 =====
    drawDogIcon(4, 4);
    display.setCursor(26, y);
    display.println("Chinese Chow Chow Comfort Monitor");
    y += lineH + 3;

    // ===== 传感器数据 =====
    display.setCursor(4, y);
    display.print("Temp: "); display.print(t,1); display.println(" C");
    y += lineH;

    display.setCursor(4, y);
    display.print("Hum : "); display.print(h,1); display.println(" %");
    y += lineH;

    display.setCursor(4, y);
    display.print("Air(MQ135): "); display.println(mq);
    y += lineH;

    display.setCursor(4, y);
    display.print("Sound: "); display.print(dB,1); display.println(" dB");
    y += lineH + 2;

    // ===== Score =====
    display.setCursor(4, y);
    display.print("T/H/A/N: ");
    display.print((int)sTemp); display.print("/");
    display.print((int)sHum);  display.print("/");
    display.print((int)sAir);  display.print("/");
    display.println((int)sNoise);
    y += lineH;

    // ===== Comfort Index =====
    display.setCursor(4, y);
    display.print("Index: ");
    display.print(comfort,1);
    display.println(" /100");
    y += lineH;

    // ===== Status =====
    const char* statusText =
      comfort >= 70 ? "Happy & Comfortable" :
      comfort >= 40 ? "Medium" :
                      "Uncomfortable";

    // 绝对不允许越界
    if (y > 112) y = 112;
    display.setCursor(4, y);
    display.print("Status: ");
    display.println(statusText);

    // ===== 保险：强制底部 12px 清白 =====
    display.fillRect(0, 116, 296, 12, GxEPD_WHITE);

  } while (display.nextPage());
}

  // ========== setup & loop ==========

  void setup() {
    Serial.begin(115200);
    while (!Serial)
      ;  // MKR 系列常用：等串口就绪

    analogReadResolution(12);  // MKR1010: 模拟输入 0~4095

    dht.begin();

    strip.begin();
    strip.setBrightness(MAX_BRIGHTNESS);  // 电流保护：全局限亮度
    strip.show();
    turnOffStrip();

    epaperInit();

    Serial.println(F("Chow Chow Comfort Monitor V3 (MKR1010) started."));
  }

  void loop() {
    unsigned long now = millis();

    // === 自动熄灯（减少平均功耗，保护 USB & Neopixel） ===
    if (now - lastLedActiveTime > LED_SLEEP_AFTER_MS) {
      turnOffStrip();
    }

    // === 每 sensorInterval 读取一次传感器 ===
    if (now - lastSensorRead >= sensorInterval) {
      lastSensorRead = now;

      // 1. DHT22 温湿度
      float t = dht.readTemperature();
      float h = dht.readHumidity();

      if (isnan(t) || isnan(h)) {
        Serial.print(F("DHT22 read failed, keep last values | "));
        t = lastTemp;
        h = lastHum;
      } else {
        lastTemp = t;
        lastHum = h;
      }

      // 2. MQ135 空气质量
      int mqRaw = analogRead(PIN_MQ135);
      lastMQ135 = mqRaw;

      // 3. 声音 MAX9814 → dB
      float dB = readSound_dB();
      float sNoise = noiseComfortScore(dB);

      // 4. 各项舒适度评分
      float sTemp = scoreTemperature(lastTemp);
      float sHum = scoreHumidity(lastHum);
      float sAir = scoreAirQuality(lastMQ135);

      float comfort = computeComfortIndex(sTemp, sHum, sAir, sNoise);

      // 5. 用 Neopixel 显示综合舒适度
      showComfortOnStrip(comfort);

      // 6. 串口一行输出（方便你看 & 可用于 Serial Plotter）
      Serial.print("T:");
      Serial.print(lastTemp, 1);
      Serial.print("  H:");
      Serial.print(lastHum, 1);
      Serial.print("  MQ:");
      Serial.print(lastMQ135);
      Serial.print("  dB:");
      Serial.print(dB, 1);
      Serial.print("  ScT:");
      Serial.print(sTemp, 0);
      Serial.print("  ScH:");
      Serial.print(sHum, 0);
      Serial.print("  ScA:");
      Serial.print(sAir, 0);
      Serial.print("  ScN:");
      Serial.print(sNoise, 0);
      Serial.print("  CI:");
      Serial.print(comfort, 1);
      Serial.println();

      // 7. e-paper 更新（例如每 5 次更新一次，大概 10s 刷新一帧）
      static int epaperCounter = 0;
      epaperCounter++;
      if (epaperCounter >= 5) {
        epaperCounter = 0;
        epaperShowStatus(lastTemp, lastHum, lastMQ135, dB,
                         sTemp, sHum, sAir, sNoise, comfort);
      }
    }

    delay(100);

    // 其余时间什么也不做，让 MCU 和电源“喘口气”
  }
