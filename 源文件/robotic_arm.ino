#include <ESP32Servo.h>
#include <WiFi.h>
#include <WebServer.h>

// 你的WiFi配置
const char* ssid     = "不重要的Pura 70 Pro";
const char* password = "Lcyyc9426";

WebServer server(80);

// ====================== LOLIN S2 Mini 引脚 ======================
#define PIN_BASE     4     // 底座
#define PIN_SHOULDER 5     // 大臂
#define PIN_ELBOW    6     // 小臂
#define PIN_WRIST    7     // 手腕（上下俯仰）
#define PIN_CLAW     8     // 夹爪
#define PIN_WRIST_LR 9     // 手腕（左右旋转）

// 舵机对象
Servo base, shoulder, elbow, wrist, claw;
Servo wristLR;

// 夹爪参数
#define CLAW_OPEN    110
#define CLAW_CLOSE   30

// 当前角度
int baseAng = 90;
int shoulderAng = 90;  
int elbowAng = 30;    
int wristAng = 90;
int wristLRAng = 90;
int clawAng = CLAW_OPEN;

// ====================== 实时驱动 ======================
void servoMove(Servo &s, int &nowAng, int tarAng) {
  tarAng = constrain(tarAng, 0, 180);
  s.write(tarAng);
  nowAng = tarAng;
}

// 机械臂复位
void armReset() {
  servoMove(base, baseAng, 90);
  servoMove(shoulder, shoulderAng, 90);
  servoMove(elbow, elbowAng, 30);   
  servoMove(wrist, wristAng, 90);
  servoMove(wristLR, wristLRAng, 90);
  servoMove(claw, clawAng, CLAW_OPEN);
}

// ====================== 网页控制界面 ======================
String getPage() {
  String html = F("<!DOCTYPE html><html lang='zh-CN'><head><meta charset='UTF-8'><title>六轴机械臂</title>");
  html += F("<style>body{text-align:center;margin:20px;font-size:22px}");
  html += F("input{width:95%;height:22px;margin:12px 0;border-radius:12px}");
  html += F("button{padding:18px 32px;margin:12px;font-size:24px;border-radius:10px}</style></head><body>");
  html += F("<h2>ESP32六轴机械臂控制</h2>");

  html += "<div>底座<input type='range' min=0 max=180 value="+String(baseAng)+" oninput=fetch('/set?base='+this.value)></div>";
  html += "<div>大臂<input type='range' min=0 max=90 value="+String(shoulderAng)+" oninput=fetch('/set?shoulder='+this.value)></div>";
  html += "<div>小臂<input type='range' min=0 max=90 value="+String(elbowAng)+" oninput=fetch('/set?elbow='+this.value)></div>"; 
  html += "<div>手腕上下<input type='range' min=0 max=180 value="+String(wristAng)+" oninput=fetch('/set?wrist='+this.value)></div>";
  html += "<div>手腕左右<input type='range' min=0 max=180 value="+String(wristLRAng)+" oninput=fetch('/set?wrist_lr='+this.value)></div>";
  html += "<div>夹爪<input type='range' min=30 max=110 value="+String(clawAng)+" oninput=fetch('/set?claw='+this.value)></div>";

  html += F("<p><button onclick=fetch('/cmd?act=reset')>复位</button>");
  html += F("<button onclick=fetch('/cmd?act=grab')>抓取</button>");
  html += F("<button onclick=fetch('/cmd?act=release')>松开</button></p></body></html>");
  return html;
}

// 网页请求处理
void handleRoot() { server.send(200, "text/html", getPage()); }
void handleSet() {
  if(server.hasArg("base"))     servoMove(base, baseAng, server.arg("base").toInt());
  if(server.hasArg("shoulder")) servoMove(shoulder, shoulderAng, server.arg("shoulder").toInt());
  if(server.hasArg("elbow"))    servoMove(elbow, elbowAng, server.arg("elbow").toInt());
  if(server.hasArg("wrist"))    servoMove(wrist, wristAng, server.arg("wrist").toInt());
  if(server.hasArg("wrist_lr")) servoMove(wristLR, wristLRAng, server.arg("wrist_lr").toInt());
  if(server.hasArg("claw"))     servoMove(claw, clawAng, server.arg("claw").toInt());
  server.send(200, "text/plain", "OK");
}

void handleCmd() {
  String cmd = server.arg("act");
  if(cmd == "reset") armReset();
  if(cmd == "grab")  servoMove(claw, clawAng, CLAW_CLOSE);
  if(cmd == "release") servoMove(claw, clawAng, CLAW_OPEN);
  server.send(200,"text/plain","OK");
}

// ====================== 初始化 ======================
void setup() {
  Serial.begin(115200);

  base.attach(PIN_BASE, 500, 2400);
  shoulder.attach(PIN_SHOULDER, 500, 2400);
  elbow.attach(PIN_ELBOW, 500, 2400);
  wrist.attach(PIN_WRIST, 500, 2400);
  wristLR.attach(PIN_WRIST_LR, 500, 2400);
  claw.attach(PIN_CLAW, 500, 2400);

  WiFi.begin(ssid, password);
  Serial.print("正在连接WiFi: "); Serial.println(ssid);
  while (WiFi.status() != WL_CONNECTED) {
    delay(300);
    Serial.print(".");
  }

  Serial.println("\nWiFi连接成功！");
  Serial.print("控制地址: "); Serial.println(WiFi.localIP());

  server.on("/", handleRoot);
  server.on("/set", handleSet);
  server.on("/cmd", handleCmd);
  server.begin();

  armReset();
}

void loop() {
  server.handleClient();
}