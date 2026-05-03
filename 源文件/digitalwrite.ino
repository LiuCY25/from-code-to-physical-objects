
#define LEDPIN 8
void setup() {
   pinMode(LEDPIN,OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(LEDPIN,HIGH);
  delay(100);
 digitalWrite(LEDPIN,LOW);
  delay(100);

}
