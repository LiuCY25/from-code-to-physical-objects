#define APIN 40

void setup() {
   //设置分辨率
   analogWriteResolution(10);
}

void loop() {
  //输出
   analogWrite(APIN,1023);
   delay(100);
   analogWrite(APIN,768);
   delay(100);
   analogWrite(APIN,512);
   delay(100);
   analogWrite(APIN,256);
   delay(100);
   analogWrite(APIN,128);
   delay(100);
   analogWrite(APIN,64);
   delay(100);
}