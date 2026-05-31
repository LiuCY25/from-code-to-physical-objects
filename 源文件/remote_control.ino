/*
 * ============================================================
 * 遥控器端 (ESP32-C3) — 正式版
 * 通信: ESP-NOW 单播 (信道1), 控制50Hz / 传感器接收
 * 显示: 1.8寸 ST7735 TFT (128x160), 四页面
 * 输入: 单摇杆 (Y=速度 X=转向), SW1=下一页 SW2=上一页
 * ============================================================
 *
 * 接线方案 (TFT引脚名按板子丝印):
 *   ESP32-C3 遥控器端
 *   ┌─────────────────────────────────────────┐
 *   │                                         │
 *   │  ★ 摇杆模块                             │
 *   │  摇杆 VCC ─────── 3.3V                  │
 *   │  摇杆 GND ─────── GND                   │
 *   │  摇杆 VRx ─────── GPIO0  (X轴=左右转向)  │
 *   │  摇杆 VRy ─────── GPIO1  (Y轴=前进后退)  │
 *   │  摇杆 SW  ─────── GPIO18 (SW1=下一页)    │
 *   │                                         │
 *   │  ★ SW2 按键 (轻触开关)                   │
 *   │  GPIO10 ──┬── 按键 ── GND               │
 *   │  (内部 INPUT_PULLUP)                     │
 *   │                                         │
 *   │  ★ 1.8寸 ST7735 TFT (板子丝印名)         │
 *   │  TFT VCC ─────── 3.3V                   │
 *   │  TFT GND ─────── GND                    │
 *   │  TFT CS  ─────── GPIO4                  │
 *   │  TFT DC  ─────── GPIO5                  │
 *   │  TFT RES ─────── GPIO6                  │
 *   │  TFT SDA ─────── GPIO7                  │
 *   │  TFT SCL ─────── GPIO8                  │
 *   │  TFT BL  ─────── 3.3V (背光常亮)        │
 *   │                                         │
 *   │  ★ GPIO19 被USB占用, SW2 不要接GPIO19    │
 *   │  ★ GPIO9 连接内部Flash, 不可用作GPIO     │
 *   └─────────────────────────────────────────┘
 */

// ==================== 引脚定义 ====================
#define PIN_JOY1_X   0
#define PIN_JOY1_Y   1
#define PIN_JOY1_SW  18
#define PIN_JOY2_X   2
#define PIN_JOY2_Y   3
#define PIN_JOY2_SW  10   // SW2 (GPIO19被USB D+占用, 改用GPIO10)

// TFT 引脚 (板子丝印: BL / CS / DC / RES / SDA / SCL / VCC / GND)
#define PIN_TFT_CS   4    // TFT CS
#define PIN_TFT_DC   5    // TFT DC
#define PIN_TFT_RST  6    // TFT RES
#define PIN_TFT_MOSI 7    // TFT SDA
#define PIN_TFT_SCK  8    // TFT SCL
// TFT BL  → 3.3V (背光)
// TFT VCC → 3.3V
// TFT GND → GND

// ==================== 参数常量 ====================
#define ADC_DEADZONE  200
#define ADC_CENTER    2048
#define SW  128
#define SH  160
#define SENSOR_TIMEOUT_MS 2000
#define SEND_INTERVAL_MS  20
#define ROW_H  10

// ==================== 数据包结构 ====================
typedef struct { int8_t speed, steering; uint8_t seq; } ctrl_packet_t;
typedef struct {
  uint16_t air_quality;
  float    latitude, longitude, speed_kmh;
  uint16_t distance_mm;
  uint8_t  satellites, alert, seq;
} sensor_packet_t;

// ==================== 库引用 ====================
#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include <SPI.h>

// ==================== 全局对象 ====================
Adafruit_ST7735 tft(PIN_TFT_CS, PIN_TFT_DC, PIN_TFT_MOSI, PIN_TFT_SCK, PIN_TFT_RST);

ctrl_packet_t   outgoing_ctrl   = {0, 0, 0};
sensor_packet_t incoming_sensor = {0};
bool new_sensor_data = false;

unsigned long last_sensor_recv = 0, last_send_time = 0;
uint8_t ctrl_seq = 0, sensor_seq_last = 255;

// ★ 替换为你的小车 MAC 地址
uint8_t car_mac[] = {0x9C, 0xCC, 0x01, 0x61, 0x77, 0x7C};

// 四页面系统
enum Page { PAGE_DRIVE, PAGE_DIST, PAGE_GPS, PAGE_AIR, PAGE_COUNT };
Page current_page = PAGE_DRIVE;
bool page_dirty[4] = {true, true, true, true};

// ==================== 辅助函数 ====================
uint8_t RY(int r) { return r * ROW_H; }

int8_t readJoy(int pin) {
  int d = analogRead(pin) - ADC_CENTER;
  if (abs(d) < ADC_DEADZONE) return 0;
  return constrain(d > 0 ? map(d, ADC_DEADZONE, 2047, 0, 100)
                         : map(d, -2047, -ADC_DEADZONE, -100, 0), -100, 100);
}

bool btn(int p) {
  if (digitalRead(p) == LOW) { delay(20); return digitalRead(p) == LOW; }
  return false;
}

void btnWait(int p) { while (digitalRead(p) == LOW) delay(10); }

// ==================== ESP-NOW ====================
void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (len != sizeof(sensor_packet_t)) return;
  sensor_packet_t pkt; memcpy(&pkt, data, len);
  uint8_t d = pkt.seq - sensor_seq_last;
  if (d == 0 || d > 127) return;
  incoming_sensor = pkt; sensor_seq_last = pkt.seq;
  new_sensor_data = true; last_sensor_recv = millis();
}

void onDataSent(const wifi_tx_info_t *info, esp_now_send_status_t s) {
  // 发送失败由 TFT Link 指示, 不打印串口
}

void sendControl() {
  unsigned long n = millis();
  if (n - last_send_time < SEND_INTERVAL_MS) return;
  last_send_time = n;
  outgoing_ctrl.seq = ctrl_seq++;
  esp_now_send(car_mac, (uint8_t*)&outgoing_ctrl, sizeof(ctrl_packet_t));
}

// ==================== UI: Link 状态栏 ====================
void drawLinkBar() {
  static bool last_online = false;
  bool online = (millis() - last_sensor_recv < SENSOR_TIMEOUT_MS);
  if (online != last_online || page_dirty[current_page]) {
    last_online = online;
    tft.fillRect(0, RY(0), SW, ROW_H, ST7735_BLACK);
    tft.setTextSize(1);
    tft.setTextColor(online ? ST7735_GREEN : ST7735_RED);
    tft.setCursor(0, RY(0));
    tft.print(online ? "Link:OK" : "Link:LOST");
  }
}

// ==================== UI: 底栏 ====================
void drawPageBar() {
  if (!page_dirty[current_page]) return;
  tft.fillRect(0, RY(15), SW, ROW_H, ST7735_BLACK);
  tft.setTextSize(1);
  tft.setTextColor(ST7735_WHITE);
  tft.setCursor(0, RY(15));
  tft.print("<SW1  P"); tft.print(current_page + 1); tft.print("/4  SW2>");
}

// ==================== P0: 驾驶状态 ====================
void drawDrivePage() {
  if (page_dirty[PAGE_DRIVE]) {
    tft.fillRect(0, RY(1), SW, SH - RY(1), ST7735_BLACK);
    tft.setTextSize(1);
    tft.setTextColor(ST7735_WHITE); tft.setCursor(0, RY(1)); tft.print("-- DRIVE --");
    tft.setCursor(0, RY(3)); tft.print("Speed:");
    tft.setCursor(0, RY(4)); tft.print("Steer:");
  }

  static int8_t last_spd = -128, last_str = -128;
  if (outgoing_ctrl.speed != last_spd || outgoing_ctrl.steering != last_str || page_dirty[PAGE_DRIVE]) {
    last_spd = outgoing_ctrl.speed; last_str = outgoing_ctrl.steering;
    tft.setTextSize(1);
    // 速度值
    tft.fillRect(36, RY(3), SW - 36, ROW_H, ST7735_BLACK);
    tft.setTextColor(ST7735_GREEN); tft.setCursor(36, RY(3));
    tft.print(last_spd);
    if (last_spd > 0) tft.print(" FWD");
    else if (last_spd < 0) tft.print(" REV");
    else tft.print(" STOP");
    // 转向值
    tft.fillRect(36, RY(4), SW - 36, ROW_H, ST7735_BLACK);
    tft.setTextColor(ST7735_CYAN); tft.setCursor(36, RY(4));
    tft.print(last_str);
    if (last_str > 20) tft.print(" R");
    else if (last_str < -20) tft.print(" L");
    else tft.print(" CTR");
    // 速度条 (中位零: 右绿前进, 左红后退)
    tft.fillRect(0, RY(6), SW, 9, ST7735_BLACK);
    tft.drawRect(0, RY(6), SW, 8, ST7735_BLUE);
    int mid = SW / 2;
    tft.drawLine(mid, RY(6), mid, RY(6) + 8, ST7735_WHITE);
    if (last_spd > 0) {
      int len = map(last_spd, 0, 100, 0, mid);
      tft.fillRect(mid, RY(6) + 1, len, 6, ST7735_GREEN);
    } else if (last_spd < 0) {
      int len = map(-last_spd, 0, 100, 0, mid);
      tft.fillRect(mid - len, RY(6) + 1, len, 6, ST7735_RED);
    }
  }

  // 障碍警告
  static uint8_t last_alert_d = 0xFF;
  if (incoming_sensor.alert != last_alert_d || page_dirty[PAGE_DRIVE]) {
    last_alert_d = incoming_sensor.alert;
    tft.fillRect(0, RY(10), SW, ROW_H, ST7735_BLACK);
    if (last_alert_d & 0x01) {
      tft.setTextColor(ST7735_RED); tft.setCursor(0, RY(10)); tft.print("!! OBSTACLE !!");
    }
  }

  page_dirty[PAGE_DRIVE] = false;
  drawLinkBar();
  drawPageBar();
}

// ==================== P1: 测距 ====================
void drawDistPage() {
  if (page_dirty[PAGE_DIST]) {
    tft.fillRect(0, RY(1), SW, SH - RY(1), ST7735_BLACK);
    tft.setTextSize(1);
    tft.setTextColor(ST7735_WHITE); tft.setCursor(0, RY(1)); tft.print("-- DISTANCE --");
    tft.setCursor(0, RY(3)); tft.print("Front:");
    tft.setCursor(0, RY(7)); tft.print("Thr:30/15cm");
  }

  static uint16_t last_dist = 0xFFFF; static uint8_t last_alert = 0xFF;
  if (incoming_sensor.distance_mm != last_dist || incoming_sensor.alert != last_alert || page_dirty[PAGE_DIST]) {
    last_dist = incoming_sensor.distance_mm; last_alert = incoming_sensor.alert;
    tft.setTextSize(1);
    // 距离值
    tft.fillRect(30, RY(3), SW - 30, ROW_H, ST7735_BLACK);
    tft.setTextColor(ST7735_CYAN); tft.setCursor(30, RY(3));
    if (last_dist == 65535) tft.print("N/A");
    else { tft.print(last_dist / 10.0, 1); tft.print("cm"); }
    // 距离条
    tft.fillRect(0, RY(5), SW, 15, ST7735_BLACK);
    int db = map(constrain(last_dist, 0, 1000), 0, 1000, SW, 0);
    tft.drawRect(0, RY(5), SW, 14, ST7735_BLUE);
    uint16_t bc = (last_dist < 300 && last_dist != 65535) ? ST7735_RED : ST7735_GREEN;
    tft.fillRect(0, RY(5), db, 14, bc);
    // 状态
    tft.fillRect(0, RY(9), SW, ROW_H, ST7735_BLACK);
    tft.setCursor(0, RY(9));
    if (last_alert & 0x01) { tft.setTextColor(ST7735_RED); tft.print("!! WARNING !!"); }
    else { tft.setTextColor(ST7735_GREEN); tft.print("Safe"); }
  }

  page_dirty[PAGE_DIST] = false;
  drawLinkBar();
  drawPageBar();
}

// ==================== P2: GPS ====================
void drawGPSPage() {
  if (page_dirty[PAGE_GPS]) {
    tft.fillRect(0, RY(1), SW, SH - RY(1), ST7735_BLACK);
    tft.setTextSize(1);
    tft.setTextColor(ST7735_WHITE); tft.setCursor(0, RY(1)); tft.print("-- GPS --");
    tft.setTextColor(ST7735_YELLOW);
    tft.setCursor(0, RY(3)); tft.print("Sat:");
  }

  static uint8_t last_sat = 0xFF; static int32_t last_li = 0x7FFFFFFF, last_ni = 0x7FFFFFFF;
  int32_t li = (int32_t)(incoming_sensor.latitude * 1000000), ni = (int32_t)(incoming_sensor.longitude * 1000000);
  if (li != last_li || ni != last_ni || incoming_sensor.satellites != last_sat || page_dirty[PAGE_GPS]) {
    last_li = li; last_ni = ni; last_sat = incoming_sensor.satellites;
    tft.setTextSize(1);
    // 卫星数
    tft.fillRect(24, RY(3), 20, ROW_H, ST7735_BLACK);
    tft.setTextColor(ST7735_YELLOW); tft.setCursor(24, RY(3)); tft.print(last_sat);
    // 经纬度
    tft.fillRect(0, RY(4), SW, ROW_H, ST7735_BLACK);
    tft.setCursor(0, RY(4)); tft.print(incoming_sensor.latitude, 6);
    tft.fillRect(0, RY(5), SW, ROW_H, ST7735_BLACK);
    tft.setCursor(0, RY(5)); tft.print(incoming_sensor.longitude, 6);
    // 锁定状态
    tft.fillRect(0, RY(9), SW, ROW_H, ST7735_BLACK);
    if (last_sat >= 4) { tft.setTextColor(ST7735_GREEN); tft.setCursor(0, RY(9)); tft.print("Fix: Locked"); }
    else { tft.setTextColor(ST7735_RED); tft.setCursor(0, RY(9)); tft.print("Fix: None"); }
  }

  page_dirty[PAGE_GPS] = false;
  drawLinkBar();
  drawPageBar();
}

// ==================== P3: 空气质量 ====================
void drawAirPage() {
  if (page_dirty[PAGE_AIR]) {
    tft.fillRect(0, RY(1), SW, SH - RY(1), ST7735_BLACK);
    tft.setTextSize(1);
    tft.setTextColor(ST7735_WHITE); tft.setCursor(0, RY(1)); tft.print("-- AIR QUALITY --");
    tft.setCursor(0, RY(3)); tft.print("MQ135:");
    tft.setCursor(0, RY(5)); tft.print("V:");
    tft.setCursor(24, RY(5)); tft.print("V");
    tft.setCursor(0, RY(9)); tft.print("0");
    tft.setCursor(SW - 25, RY(9)); tft.print("400");
  }

  static uint16_t last_aq = 0xFFFF;
  if (incoming_sensor.air_quality != last_aq || page_dirty[PAGE_AIR]) {
    last_aq = incoming_sensor.air_quality;
    tft.setTextSize(1);
    // MQ135 值 + 等级
    tft.fillRect(36, RY(3), SW - 36, ROW_H, ST7735_BLACK);
    tft.setTextColor(ST7735_GREEN); tft.setCursor(36, RY(3));
    tft.print(last_aq);
    tft.print(" ");
    tft.print(last_aq < 100 ? "Good" : last_aq < 200 ? "Fair" : "Poor");
    // 电压
    tft.fillRect(12, RY(5), 30, ROW_H, ST7735_BLACK);
    tft.setTextColor(ST7735_CYAN); tft.setCursor(12, RY(5));
    tft.print(last_aq * 3.3 / 4095, 2);
    tft.print("V");
    // 进度条
    tft.fillRect(0, RY(7), SW, 15, ST7735_BLACK);
    int bw = map(last_aq, 0, 400, 0, SW);
    tft.drawRect(0, RY(7), SW, 14, ST7735_BLUE);
    tft.fillRect(0, RY(7), bw, 14, ST7735_GREEN);
  }

  page_dirty[PAGE_AIR] = false;
  drawLinkBar();
  drawPageBar();
}

void drawCurrentPage() {
  switch (current_page) {
    case PAGE_DRIVE: drawDrivePage(); break;
    case PAGE_DIST:  drawDistPage();  break;
    case PAGE_GPS:   drawGPSPage();   break;
    case PAGE_AIR:   drawAirPage();   break;
  }
}

// ==================== 初始化 ====================
void setup() {
  Serial.begin(115200); delay(500);
  Serial.println("[遥控器] 启动");

  pinMode(PIN_JOY1_SW, INPUT_PULLUP);
  pinMode(PIN_JOY2_SW, INPUT_PULLUP);
  analogReadResolution(12);

  // TFT 初始化 (已验证: INITR_GREENTAB + rotation 0)
  tft.initR(INITR_GREENTAB);
  tft.setRotation(0);
  tft.fillScreen(ST7735_BLACK);
  Serial.println("[OK] TFT");

  // ESP-NOW (信道1, 防止漂移)
  WiFi.mode(WIFI_STA); WiFi.disconnect();
  esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);
  esp_now_init();
  esp_now_register_recv_cb(onDataRecv);
  esp_now_register_send_cb(onDataSent);

  esp_now_peer_info_t p = {};
  memcpy(p.peer_addr, car_mac, 6);
  p.channel = 1; p.encrypt = false;
  esp_now_add_peer(&p);

  Serial.print("MAC: "); Serial.println(WiFi.macAddress());
  Serial.println("[遥控器] 就绪");

  drawCurrentPage();
}

// ==================== 主循环 ====================
void loop() {
  // 摇杆始终控制小车 (单摇杆: Y=速度, X=转向)
  int8_t s = readJoy(PIN_JOY1_Y), t = -readJoy(PIN_JOY1_X);
  static int8_t ls = 0, lt = 0;
  if (s != ls || t != lt) { ls = s; lt = t; outgoing_ctrl.speed = s; outgoing_ctrl.steering = t; }
  sendControl();

  // SW1: 下一页
  if (btn(PIN_JOY1_SW)) {
    btnWait(PIN_JOY1_SW);
    page_dirty[current_page] = true;
    current_page = (Page)((current_page + 1) % PAGE_COUNT);
    page_dirty[current_page] = true;
    drawCurrentPage();
  }

  // SW2: 上一页
  if (btn(PIN_JOY2_SW)) {
    btnWait(PIN_JOY2_SW);
    page_dirty[current_page] = true;
    current_page = (Page)((current_page + PAGE_COUNT - 1) % PAGE_COUNT);
    page_dirty[current_page] = true;
    drawCurrentPage();
  }

  // 传感器数据刷新当前页 (限速500ms)
  static unsigned long ltd = 0;
  if (new_sensor_data) {
    new_sensor_data = false;
    if (millis() - ltd >= 500) { ltd = millis(); drawCurrentPage(); }
  }

  delay(1);
}
