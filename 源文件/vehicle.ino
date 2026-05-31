/*
 * ============================================================
 * 智能小车端 (ESP32-C3) — 正式版 (优化版)
 * 通信: ESP-NOW 单播 (信道1)
 * 舵机: MG90S 360° x2 (驱动+转向), ESP32Servo 库
 * 传感器: MQ-135空气质量 / HC-SR04超声波测距 / VK2828U7G5LF GPS
 *
 * ★ 修复要点:
 *   1. FreeRTOS 独立任务以固定 50Hz 更新舵机，消除 loop 阻塞延迟
 *   2. HC-SR04 中断方式替代 pulseIn()，不再阻塞 30ms
 *   3. 距离读数最小值校验 (>=20mm)，防止噪声触发避障
 *   4. 收到 ESP-NOW 控制包时立即更新舵机（零延迟路径）
 *   5. DEBUG_TIMING=1 可开启各阶段耗时监控
 * ============================================================
 *
 * 接线方案 (4×AA 同源供电, 电解电容并电源入口):
 *   ESP32-C3 小车端
 *   ┌──────────────────────────────────────────────┐
 *   │                                              │
 *   │  ★ 4×AA 电池盒输出                            │
 *   │  ┌─────────────────────────────────────┐     │
 *   │  │ 电池(+) ──┬───┬───┬───┬─── VCC总线   │     │
 *   │  │           │   │   │   │              │     │
 *   │  │          ─┴─  │   │   │              │     │
 *   │  │       电解电容 │   │   │              │     │
 *   │  │     100~1000μF│   │   │              │     │
 *   │  │          ─┬─  │   │   │              │     │
 *   │  │           │   │   │   │              │     │
 *   │  │ 电池(-) ──┴───┴───┴───┴─── GND总线   │     │
 *   │  └─────────────────────────────────────┘     │
 *   │                                              │
 *   │  ESP32-C3 VIN  ──── VCC总线 (5~6V)           │
 *   │  ESP32-C3 GND  ──── GND总线                  │
 *   │                                              │
 *   │  驱动舵机 橙线 ──── GPIO1                     │
 *   │  驱动舵机 红线 ──── VCC总线                   │
 *   │  驱动舵机 棕线 ──── GND总线                   │
 *   │                                              │
 *   │  转向舵机 橙线 ──── GPIO2                     │
 *   │  转向舵机 红线 ──── VCC总线                   │
 *   │  转向舵机 棕线 ──── GND总线                   │
 *   │                                              │
 *   │  状态LED  ──── GPIO10 ──[220Ω]── GND总线     │
 *   │  警告LED  ──── GPIO4  ──[220Ω]── GND总线     │
 *   │                                              │
 *   │  MQ-135 AO ──── GPIO0  (ADC)                 │
 *   │  MQ-135 VCC ──── VCC总线                     │
 *   │  MQ-135 GND ──── GND总线                     │
 *   │                                              │
 *   │  HC-SR04 VCC ──── VCC总线                    │
 *   │  HC-SR04 GND ──── GND总线                    │
 *   │  HC-SR04 TRIG ─── GPIO6                      │
 *   │  HC-SR04 ECHO ───[1kΩ]──┬── GPIO5            │
 *   │                         │                    │
 *   │                        [2kΩ]                 │
 *   │                         │                    │
 *   │                       GND总线                │
 *   │               ★ 5V→3.3V分压!                 │
 *   │                                              │
 *   │  GPS VCC ─────── VCC总线                     │
 *   │  GPS GND ─────── GND总线                     │
 *   │  GPS TX  ─────── GPIO7 (ESP32 RX)            │
 *   │  GPS RX  ─────── GPIO8 (ESP32 TX)            │
 *   │                                              │
 *   │  ★ 4×AA 内阻自然降压，带载约 4.5~5.5V         │
 *   │  ★ 舵机不经过 ESP32 板载LDO，直接挂 VCC总线    │
 *   │  ★ 电解电容并在电池入口，缓冲舵机启动电流       │
 *   │  ★ 所有 GND 必须共地                          │
 *   │  ★ HC-SR04 ECHO 为 5V 电平，需电阻分压至 3.3V  │
 *   │  ★ 不要串二极管! 压降太大致 ESP32 欠压复位     │
 *   └──────────────────────────────────────────────┘
 */

// ==================== 传感器开关 ====================
#define ENABLE_GPS      1   // VK2828U7G5LF GPS
#define ENABLE_MQ135    1   // MQ-135 空气质量 (ADC)
#define ENABLE_HCSR04   1   // HC-SR04 超声波测距

// ==================== 调试开关 ====================
#define DEBUG_TIMING    1   // 设为 0 关闭各阶段耗时输出

// ==================== 引脚定义 ====================
#define PIN_DRIVE_SERVO     1
#define PIN_STEER_SERVO     2
#define PIN_MQ135           0
#define PIN_LED_WARN        4
#define PIN_LED_STATUS      10
#define PIN_HCSR04_TRIG    6
#define PIN_HCSR04_ECHO    5
#define PIN_GPS_TX          7
#define PIN_GPS_RX          8

// ==================== 舵机参数 ====================
#define DRIVE_STOP_US       1500
#define DRIVE_FWD_MAX_US    2000
#define DRIVE_BWD_MAX_US    1000
#define STEER_CENTER_US     1530
#define STEER_LEFT_MAX_US   1000
#define STEER_RIGHT_MAX_US  2000

// ==================== 避障阈值 ====================
#define DISTANCE_ALERT_MM    500
#define DISTANCE_STOP_MM     300
#define DISTANCE_MIN_VALID   20    // HC-SR04 最小有效距离 (防噪声)

// ==================== 超时保护 ====================
#define CTRL_TIMEOUT_MS      2000

// ==================== 舵机更新频率 ====================
#define SERVO_HZ             50
#define SERVO_INTERVAL_MS    (1000 / SERVO_HZ)

// ==================== 库引用 ====================
#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <ESP32Servo.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#if ENABLE_GPS
#include <TinyGPS++.h>
#endif

// ==================== 数据包结构 ====================
typedef struct {
  int8_t  speed;
  int8_t  steering;
  uint8_t seq;
} ctrl_packet_t;

typedef struct {
  uint16_t air_quality;
  float    latitude;
  float    longitude;
  float    speed_kmh;
  uint16_t distance_mm;
  uint8_t  satellites;
  uint8_t  alert;
  uint8_t  seq;
} sensor_packet_t;

// ==================== 全局对象 ====================
#if ENABLE_GPS
TinyGPSPlus gps;
HardwareSerial gpsSerial(1);
#endif

Servo driveServo;
Servo steerServo;

ctrl_packet_t   incoming_ctrl   = {0, 0, 0};
sensor_packet_t outgoing_sensor = {0};

uint8_t       ctrl_seq_last    = 255;
unsigned long last_ctrl_recv   = 0;
bool          connection_alive = false;

unsigned long last_sensor_send = 0;
unsigned long last_buzzer_beep = 0;
bool          buzzer_state     = false;
uint8_t       sensor_seq       = 0;

// 硬件运行时检测状态
bool hw_check_done = false;
bool gps_hw_ok     = false;
bool hcsr04_hw_ok  = false;

// ★ 替换为你的遥控器 MAC 地址
uint8_t remote_mac[] = {0x9C, 0xCC, 0x01, 0x61, 0x1F, 0x6C};

// ==================== HC-SR04 中断状态 ====================
#if ENABLE_HCSR04
volatile enum {
  HCSR04_IDLE,       // 空闲，可以触发下一次测量
  HCSR04_TRIGGERED,  // 已发送触发脉冲，等待回声
} hcsr04_state = HCSR04_IDLE;

volatile unsigned long hcsr04_echo_start = 0;
volatile unsigned long hcsr04_echo_dur   = 0;
volatile bool          hcsr04_echo_ready = false;
uint32_t               hcsr04_trigger_time = 0;
bool                   hcsr04_ok = false;
#endif

// ==================== FreeRTOS 舵机任务 ====================
TaskHandle_t servoTaskHandle = NULL;

// 避障限速: 根据超声波距离对前进速度施加限制
// 返回有效速度值 (不修改原始 incoming_ctrl.speed)
int8_t applyObstacleLimit(int8_t raw_speed) {
#if ENABLE_HCSR04
  uint16_t d = outgoing_sensor.distance_mm;
  if (d == 65535) return raw_speed;
  if (d < DISTANCE_STOP_MM)  return (raw_speed > 0) ? 0 : raw_speed;
  if (d < DISTANCE_ALERT_MM) return (raw_speed > 20) ? 20 : raw_speed;
#endif
  return raw_speed;
}

// 舵机任务: 以固定 50Hz 更新舵机，与主 loop 解耦
// ★ 在此处统一施加避障限速，确保单一写入源，消除竞态
void servoTask(void *pvParameters) {
  TickType_t lastWakeTime = xTaskGetTickCount();
  while (1) {
    int8_t raw_spd = incoming_ctrl.speed;
    int8_t raw_str = incoming_ctrl.steering;

    int8_t eff_spd = applyObstacleLimit(raw_spd);

    setDriveSpeed(eff_spd);
    setSteering(raw_str);

    vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(SERVO_INTERVAL_MS));
  }
}

// ==================== 舵机控制 ====================
void setDriveSpeed(int8_t pct) {
  pct = constrain(pct, -100, 100);
  uint16_t us;
  if (pct == 0) us = DRIVE_STOP_US;
  else if (pct > 0) us = map(pct, 1, 100, DRIVE_STOP_US + 10, DRIVE_FWD_MAX_US);
  else us = map(pct, -100, -1, DRIVE_BWD_MAX_US, DRIVE_STOP_US - 10);
  driveServo.writeMicroseconds(us);
}

void setSteering(int8_t pct) {
  pct = constrain(pct, -100, 100);
  uint16_t us;
  if (pct == 0) us = STEER_CENTER_US;
  else if (pct < 0) us = map(pct, -100, -1, STEER_LEFT_MAX_US, STEER_CENTER_US - 10);
  else us = map(pct, 1, 100, STEER_CENTER_US + 10, STEER_RIGHT_MAX_US);
  steerServo.writeMicroseconds(us);
}

// ==================== HC-SR04 中断服务 ====================
#if ENABLE_HCSR04
void IRAM_ATTR hcsr04EchoISR() {
  uint8_t level = digitalRead(PIN_HCSR04_ECHO);
  if (level == HIGH) {
    // 上升沿：仅在 TRIGGERED 状态记录，防止噪声误触发
    if (hcsr04_state == HCSR04_TRIGGERED) {
      hcsr04_echo_start = micros();
    }
  } else {
    // 下降沿：计算脉宽
    if (hcsr04_state == HCSR04_TRIGGERED && hcsr04_echo_start > 0) {
      unsigned long now = micros();
      if (now > hcsr04_echo_start) {
        unsigned long dur = now - hcsr04_echo_start;
        // 脉宽校验: HC-SR04 有效范围 ~120us(2cm) ~ 25000us(4.3m)
        // 超出范围的视为噪声，直接丢弃
        if (dur >= 120 && dur <= 25000) {
          hcsr04_echo_dur = dur;
          hcsr04_echo_ready = true;
        }
      }
    }
    hcsr04_state = HCSR04_IDLE;
  }
}
#endif

// ==================== ESP-NOW 回调 ====================
void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (len != sizeof(ctrl_packet_t)) return;

  ctrl_packet_t pkt;
  memcpy(&pkt, data, len);

  uint8_t diff = pkt.seq - ctrl_seq_last;
  if (diff == 0 || diff > 127) return;  // 重复包或旧包

  incoming_ctrl  = pkt;
  ctrl_seq_last  = pkt.seq;
  last_ctrl_recv = millis();

  // ★ 立即更新舵机 (零延迟)，同时施加避障限速防止与 servoTask 冲突
  setDriveSpeed(applyObstacleLimit(pkt.speed));
  setSteering(pkt.steering);

  if (!connection_alive) {
    connection_alive = true;
    Serial.println("[通信] 连接恢复");
  }
}

void onDataSent(const wifi_tx_info_t *tx_info, esp_now_send_status_t status) {
  if (status != ESP_NOW_SEND_SUCCESS) {
    Serial.println("[警告] 传感器数据发送失败");
  }
}

// ==================== HC-SR04 非阻塞触发 ====================
#if ENABLE_HCSR04
void hcsr04Trigger() {
  // ★ 先检查超时：若卡在 TRIGGERED 超过 35ms 仍无回声，强制回收
  if (hcsr04_state != HCSR04_IDLE) {
    if (hcsr04_trigger_time > 0 && millis() - hcsr04_trigger_time > 35) {
      hcsr04_state = HCSR04_IDLE;
      hcsr04_echo_ready = false;
    } else {
      return;  // 正常等待回声，不触发新测量
    }
  }

  // 状态为空闲 (或刚被超时回收)，启动新测量
  hcsr04_echo_ready = false;
  hcsr04_echo_start = 0;
  hcsr04_echo_dur   = 0;
  hcsr04_state      = HCSR04_TRIGGERED;
  hcsr04_trigger_time = millis();

  // 发送 10us 触发脉冲
  digitalWrite(PIN_HCSR04_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_HCSR04_TRIG, LOW);
}
#endif

// ==================== 传感器读取 ====================
void readSensors() {
#if ENABLE_MQ135
  static uint16_t buf[8] = {0};
  static uint8_t  idx    = 0;
  uint16_t raw = analogRead(PIN_MQ135);
  buf[idx] = raw;
  idx = (idx + 1) % 8;
  uint32_t sum = 0;
  for (int i = 0; i < 8; i++) sum += buf[i];
  outgoing_sensor.air_quality = sum / 8;
#else
  outgoing_sensor.air_quality = 0;
#endif

#if ENABLE_HCSR04
  // ★ 非阻塞方式读取 HC-SR04 + 滑动窗口中值滤波 (5样本)
  if (hcsr04_hw_ok) {
    if (hcsr04_echo_ready) {
      hcsr04_echo_ready = false;
      uint16_t raw_dist = (uint16_t)(hcsr04_echo_dur * 10 / 58);
      // 距离校验: 有效范围 20mm ~ 4000mm
      if (raw_dist >= DISTANCE_MIN_VALID && raw_dist <= 4000) {
        // 滑动窗口滤波 (5个历史值取中位数, 抗脉冲噪声)
        static uint16_t win[5] = {0};
        static uint8_t  wi     = 0;
        static bool     win_full = false;
        win[wi] = raw_dist;
        wi = (wi + 1) % 5;
        if (wi == 0) win_full = true;
        if (win_full) {
          // 排序取中位数
          uint16_t s[5];
          memcpy(s, win, sizeof(s));
          for (int a = 0; a < 4; a++)
            for (int b = a + 1; b < 5; b++)
              if (s[a] > s[b]) { uint16_t t = s[a]; s[a] = s[b]; s[b] = t; }
          outgoing_sensor.distance_mm = s[2];  // 中位数
        } else {
          outgoing_sensor.distance_mm = raw_dist;
        }
      }
    }
    // 触发下一次测量 (非阻塞，仅空闲时触发)
    hcsr04Trigger();
  } else {
    outgoing_sensor.distance_mm = 65535;
  }
#else
  outgoing_sensor.distance_mm = 65535;
#endif

#if ENABLE_GPS
  outgoing_sensor.latitude   = gps.location.lat();
  outgoing_sensor.longitude  = gps.location.lng();
  outgoing_sensor.speed_kmh  = gps.speed.kmph();
  outgoing_sensor.satellites = gps.satellites.value();
#else
  outgoing_sensor.latitude   = 0.0f;
  outgoing_sensor.longitude  = 0.0f;
  outgoing_sensor.speed_kmh  = 0.0f;
  outgoing_sensor.satellites = 0;
#endif

  // 警告标志
  outgoing_sensor.alert = 0;
#if ENABLE_HCSR04
  if (outgoing_sensor.distance_mm < DISTANCE_ALERT_MM && outgoing_sensor.distance_mm != 65535)
    outgoing_sensor.alert |= 0x01;
#endif

  outgoing_sensor.seq = sensor_seq++;
}

// ==================== 避障 (LED警告，限速由 servoTask 统一处理) ====================
void handleObstacleAvoidance() {
#if ENABLE_HCSR04
  uint16_t d = outgoing_sensor.distance_mm;

  if (d == 65535) {
    digitalWrite(PIN_LED_WARN, LOW);
    buzzer_state = false;
    return;
  }

  if (d < DISTANCE_STOP_MM) {
    digitalWrite(PIN_LED_WARN, HIGH);
    return;
  }

  if (d < DISTANCE_ALERT_MM) {
    if (millis() - last_buzzer_beep > 150) {
      last_buzzer_beep = millis();
      buzzer_state = !buzzer_state;
      digitalWrite(PIN_LED_WARN, buzzer_state);
    }
    return;
  }
#endif

  digitalWrite(PIN_LED_WARN, LOW);
  buzzer_state = false;
}

// ==================== 发送传感器数据 ====================
void sendSensorData() {
  esp_err_t err = esp_now_send(remote_mac, (uint8_t*)&outgoing_sensor, sizeof(sensor_packet_t));
  if (err != ESP_OK) {
    Serial.print("[错误] 发送失败, 错误码: ");
    Serial.println(err);
  }
}

// ==================== 硬件真实检测 ====================
// 启动后延迟执行一次，通过实际通信判断传感器是否在线
void checkHardware() {
  if (hw_check_done) return;

  // GPS 冷启动需 1-3 分钟，这里用 30s 窗口判断 UART 是否有数据流入
  if (millis() < 30000) return;
  hw_check_done = true;

  // --- HC-SR04 (数字触发/回波) ---
#if ENABLE_HCSR04
  hcsr04_hw_ok = hcsr04_ok;
#endif

  // --- GPS (UART) ---
#if ENABLE_GPS
  gps_hw_ok = (gps.charsProcessed() > 0);  // 30s 内收到任何 NMEA 字符即判在线
#endif

  Serial.println("\n=== 硬件自检 (上电30s) ===");

  Serial.print("  HC-SR04:  ");
#if ENABLE_HCSR04
  Serial.println(hcsr04_hw_ok ? "在线" : "未连接");
#else
  Serial.println("未安装");
#endif

  Serial.print("  GPS:      ");
#if ENABLE_GPS
  if (gps_hw_ok) {
    Serial.print("在线 (收到"); Serial.print((int)gps.charsProcessed()); Serial.print("字符");
    if (gps.satellites.value() > 0) {
      Serial.print(", 卫星"); Serial.print((int)gps.satellites.value());
    }
    Serial.println(")");
  } else {
    Serial.println("未连接 (30s内无NMEA数据)");
  }
#else
  Serial.println("未安装");
#endif

  Serial.print("  MQ-135:   ");
#if ENABLE_MQ135
  {
    uint16_t aq = outgoing_sensor.air_quality;
    Serial.print("ADC="); Serial.print(aq);
    if (aq <= 5 || aq >= 4090)
      Serial.println(" [异常: 读数接近轨电压, 疑似未连接]");
    else
      Serial.println(" [正常]");
  }
#else
  Serial.println("未安装");
#endif

  Serial.println("============================\n");
}

// ==================== 初始化 ====================
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println();
  Serial.println("============================");
  Serial.println("  智能小车端 — 正式版 (优化)");
  Serial.println("============================");

  pinMode(PIN_LED_WARN, OUTPUT);
  pinMode(PIN_LED_STATUS, OUTPUT);
  digitalWrite(PIN_LED_WARN, LOW);
  digitalWrite(PIN_LED_STATUS, HIGH);

  // 舵机初始化 (ESP32Servo 库)
  driveServo.attach(PIN_DRIVE_SERVO);
  steerServo.attach(PIN_STEER_SERVO);
  setDriveSpeed(0);
  setSteering(0);
  Serial.println("[OK] 舵机就绪");

  // ★ 创建独立舵机更新任务 (优先级 2，高于默认 loop_task 优先级 1)
  xTaskCreate(
    servoTask,           // 任务函数
    "servo",             // 名称
    2048,                // 栈大小 (字)
    NULL,                // 参数
    2,                   // 优先级 (高于 loop_task)
    &servoTaskHandle     // 句柄
  );
  Serial.println("[OK] 舵机任务 50Hz");

  analogReadResolution(12);

  // HC-SR04 超声波测距 (中断方式)
#if ENABLE_HCSR04
  pinMode(PIN_HCSR04_TRIG, OUTPUT);
  pinMode(PIN_HCSR04_ECHO, INPUT);
  digitalWrite(PIN_HCSR04_TRIG, LOW);
  hcsr04_ok = true;
  // ★ 挂载 CHANGE 中断替代 pulseIn
  attachInterrupt(digitalPinToInterrupt(PIN_HCSR04_ECHO), hcsr04EchoISR, CHANGE);
  Serial.println("[OK] HC-SR04 就绪 (中断模式)");
#endif

  // GPS
#if ENABLE_GPS
  gpsSerial.begin(9600, SERIAL_8N1, PIN_GPS_TX, PIN_GPS_RX);
  Serial.println("[OK] GPS 串口就绪 (9600bps)");
#endif

  // MQ-135 预热
#if ENABLE_MQ135
  for (int i = 0; i < 8; i++) {
    analogRead(PIN_MQ135);
    delay(5);
  }
  Serial.println("[OK] MQ-135 ADC 就绪 (需预热10分钟)");
#endif

  // 硬件自检将在 30 秒后执行 (需要给传感器预热和收数据的时间)

  // ESP-NOW 初始化
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);

  if (esp_now_init() != ESP_OK) {
    Serial.println("[错误] ESP-NOW 初始化失败, 系统停止!");
    while (1) {
      digitalWrite(PIN_LED_STATUS, !digitalRead(PIN_LED_STATUS));
      delay(200);
    }
  }
  esp_now_register_recv_cb(onDataRecv);
  esp_now_register_send_cb(onDataSent);

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, remote_mac, 6);
  peer.channel = 1;
  peer.encrypt = false;
  if (esp_now_add_peer(&peer) != ESP_OK) {
    Serial.println("[错误] 添加 Peer 失败!");
  } else {
    Serial.println("[OK] Peer 添加成功");
  }

  Serial.print("本机 MAC: ");
  Serial.println(WiFi.macAddress());
  Serial.print("目标 MAC: ");
  for (int i = 0; i < 6; i++) {
    if (remote_mac[i] < 0x10) Serial.print("0");
    Serial.print(remote_mac[i], HEX);
    if (i < 5) Serial.print(":");
  }
  Serial.println();
  Serial.println("等待遥控指令...");
  Serial.println("============================\n");

  delay(300);
  digitalWrite(PIN_LED_STATUS, LOW);
}

// ==================== 主循环 ====================
void loop() {
#if DEBUG_TIMING
  unsigned long t0 = micros();
#endif

  // ---- 1. GPS NMEA 数据喂入 (限制每次最多处理 16 字节，防止饥饿) ----
#if ENABLE_GPS
  {
    int gps_count = 0;
    while (gpsSerial.available() && gps_count < 16) {
      gps.encode(gpsSerial.read());
      gps_count++;
    }
  }
#endif

#if DEBUG_TIMING
  unsigned long t1 = micros();
#endif

  // ---- 2. 启动后 30 秒执行一次硬件自检 ----
  checkHardware();

  // ---- 3. 传感器读取 (每200ms) ----
  static unsigned long last_read = 0;
  if (millis() - last_read >= 200) {
    last_read = millis();
    readSensors();
  }

#if DEBUG_TIMING
  unsigned long t2 = micros();
#endif

  // ---- 4. 控制信号超时检测 (超时只停车, 不影响数据采集) ----
  if (millis() - last_ctrl_recv > CTRL_TIMEOUT_MS) {
    if (connection_alive) {
      connection_alive = false;
      incoming_ctrl.speed    = 0;
      incoming_ctrl.steering = 0;
      Serial.println("[警告] 遥控信号超时! 舵机已停止");
    }
  }

  // ---- 5. 避障声光警告 (限速由 servoTask/onDataRecv 中的 applyObstacleLimit 统一处理) ----
  handleObstacleAvoidance();

  // ---- 6. 传感器数据发送 (每500ms, 独立于连接状态) ----
  if (millis() - last_sensor_send >= 500) {
    last_sensor_send = millis();
    sendSensorData();
  }

#if DEBUG_TIMING
  unsigned long t3 = micros();
#endif

  // ---- 7. 状态指示灯 (断连时闪烁) ----
  if (!connection_alive) {
    if (millis() - last_buzzer_beep > 150) {
      last_buzzer_beep = millis();
      digitalWrite(PIN_LED_STATUS, !digitalRead(PIN_LED_STATUS));
    }
  } else {
    digitalWrite(PIN_LED_STATUS, LOW);
  }

#if DEBUG_TIMING
  unsigned long t4 = micros();

  // 每秒输出一次各阶段耗时
  static unsigned long last_dbg = 0;
  static uint16_t dbg_count = 0;
  dbg_count++;
  if (millis() - last_dbg >= 1000) {
    last_dbg = millis();
    int8_t eff = applyObstacleLimit(incoming_ctrl.speed);
    Serial.print("[时序] GPS=");
    Serial.print(t1 - t0);
    Serial.print("us  Sensor=");
    Serial.print(t2 - t1);
    Serial.print("us  Tx+Avoid=");
    Serial.print(t3 - t2);
    Serial.print("us  LED=");
    Serial.print(t4 - t3);
    Serial.print("us  Total=");
    Serial.print(t4 - t0);
    Serial.print("us  Loops=");
    Serial.print(dbg_count);
    Serial.print("/s  Dist=");
    Serial.print(outgoing_sensor.distance_mm);
    Serial.print("mm  RawSpd=");
    Serial.print(incoming_ctrl.speed);
    Serial.print("  EffSpd=");
    Serial.print(eff);
    if (eff != incoming_ctrl.speed) Serial.print(" [避障]");
    Serial.println();
    dbg_count = 0;
  }
#endif

  delay(1);
}
