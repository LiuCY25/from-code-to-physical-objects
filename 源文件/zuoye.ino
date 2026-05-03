#include <WiFi.h>

// ===================== 【可修改配置】 =====================
const char *ssid = "chen iQOO Z10x";
const char *password = "Lcyyc9426";

#define PHOTO_PIN     2
#define SERVO_PIN     5
#define LED_PIN       7
#define KEY_MODE_PIN  8     // 模式切换按键（自动/手动）
#define KEY_MANUAL_PIN 12    // 手动模式开关灯按键 → 已改为 GPIO12
#define LOW_THRESHOLD  2000
#define HIGH_THRESHOLD 3500
// ====================================================================

#define LEDC_FREQ     50
#define LEDC_RES      12
#define SERVO_RIGHT   340
#define SERVO_LEFT    100
#define SERVO_STOP    205

WiFiServer server(80);
int photoValue = 0;
bool manualMode = false;
bool lightState = false;  // 手动模式灯光状态

// 按键消抖通用函数
bool readKey(int pin, bool &lastState, unsigned long &debounceTime) {
  int reading = digitalRead(pin);
  bool keyPress = false;
  if (reading != lastState) {
    debounceTime = millis();
  }
  if (millis() - debounceTime > 80) {
    if (reading == LOW) {
      keyPress = true;
    }
  }
  lastState = reading;
  return keyPress;
}

// 光敏滤波
int readPhoto() {
  int sum = 0;
  for (int i = 0; i < 5; i++) {
    sum += analogRead(PHOTO_PIN);
    delay(5);
  }
  return sum / 5;
}

// 灯光控制（舵机+LED）
void setLight(bool on) {
  if (on) {
    ledcWrite(SERVO_PIN, SERVO_RIGHT);
    digitalWrite(LED_PIN, HIGH);
    lightState = true;
  } else {
    ledcWrite(SERVO_PIN, SERVO_LEFT);
    digitalWrite(LED_PIN, LOW);
    lightState = false;
  }
}

// 按键变量
bool lastModeKeyState = HIGH;
unsigned long modeDebounceTime = 0;

bool lastManualKeyState = HIGH;
unsigned long manualDebounceTime = 0;

void setup() {
  Serial.begin(115200);
  ledcAttach(SERVO_PIN, LEDC_FREQ, LEDC_RES);
  ledcWrite(SERVO_PIN, SERVO_STOP);
  
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // 两个轻触按键上拉输入
  pinMode(KEY_MODE_PIN, INPUT_PULLUP);
  pinMode(KEY_MANUAL_PIN, INPUT_PULLUP);

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(50);
    Serial.print(".");
  }
  Serial.println("\nWiFi连接成功");
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());
  server.begin();
}

void loop() {
  photoValue = readPhoto();

  // ===================== 按键1：切换自动/手动 =====================
  if (readKey(KEY_MODE_PIN, lastModeKeyState, modeDebounceTime)) {
    manualMode = !manualMode;
    Serial.println("=== 按键切换模式 ===");
  }

  // ===================== 按键2：手动模式下开关灯 =====================
  if (manualMode && readKey(KEY_MANUAL_PIN, lastManualKeyState, manualDebounceTime)) {
    lightState = !lightState;
    setLight(lightState);
  }

  // ===================== 自动模式逻辑 =====================
  if (!manualMode) {
    if (photoValue < LOW_THRESHOLD) {
      setLight(true);
    } else if (photoValue > HIGH_THRESHOLD) {
      setLight(false);
    }
  }

  // ===================== 实时串口打印 =====================
  Serial.print("模式:");
  Serial.print(manualMode ? "手动" : "自动");
  Serial.print(" | 光敏:");
  Serial.print(photoValue);
  Serial.print(" | LED:");
  Serial.println(digitalRead(LED_PIN) ? "亮" : "灭");

  // ===================== WiFi网页控制 =====================
  WiFiClient client = server.accept();
  if (client) {
    String req = client.readStringUntil('\r');
    client.flush();

    if (req.indexOf("/R") != -1) {
      manualMode = true; setLight(true);
    }
    if (req.indexOf("/L") != -1) {
      manualMode = true; setLight(false);
    }
    if (req.indexOf("/A") != -1) {
      manualMode = false;
    }
    if (req.indexOf("/M") != -1) {
      manualMode = true;
    }

    client.println("HTTP/1.1 200 OK");
    client.println("Content-type:text/html; charset=utf-8");
    client.println();
    client.println("<h1>双按键智能光控系统</h1>");
    client.println("<p>模式：" + String(manualMode ? "手动" : "自动") + "</p>");
    client.println("<p>光敏值：" + String(photoValue) + "</p>");
    client.println("<p>LED：" + String(digitalRead(LED_PIN) ? "亮" : "灭") + "</p>");
    client.println("<p><a href='/R'>💡开灯</a> <a href='/L'>⚫关灯</a> <a href='/M'>手动模式</a> <a href='/A'>自动模式</a></p>");
    
    client.stop();
  }

  delay(50);
}