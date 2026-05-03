#define BUTTONPIN 40

//初始化一个全局变量，用来存储按钮的状态。
int in;

void setup() {
  // put your setup code here, to run once:
  //设置端口为输入。
  pinMode(BUTTONPIN,INPUT);
  //以9600波特率，打开串口
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  //数字读设置好的端口，存到in。
  in = digitalRead(BUTTONPIN);
  //串口输出in（也就是发送给电脑），这样在电脑上的Arduino IDE上可以看到in的结果了。
  Serial.println(in);
  //延迟以下，等待硬件执行
  delay(100);

}