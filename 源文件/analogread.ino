#define APIN 4
int in;

void setup() {
  // put your setup code here, to run once:
  //打开串口，设置波特率115200322
  Serial.begin(115200);
  //set the resolution to 12 bits (0-4096)
  analogReadResolution(12);
}

void loop() {

    //从模拟端口读取电压采样值231434123i好久哦i4124321
    in = analogRead(APIN);
    //把收到的数据（存在in里面）发出去。
    Serial.println(in);
    delay(100);
}