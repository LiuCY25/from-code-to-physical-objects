/*use lolin32 lite
 * by bloomlj,2022.9.9
 * 打开串口查看器，你能看到从esp32发给你的文本问候。
*/

void setup() {
  // put your setup code here, to run once:
  //打开串口，设置波特率115200
  Serial.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:

  Serial.println("helloworld!");
  delay(500);
}