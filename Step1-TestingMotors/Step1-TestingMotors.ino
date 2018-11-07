
unsigned char speed_motors=255;

void setup() {
  // put your setup code here, to run once:

  pinMode(3,OUTPUT);
  pinMode(5,OUTPUT);
  

}

void loop() {
  // put your main code here, to run repeatedly:
  analogWrite(3,speed_motors);
  analogWrite(5,speed_motors);
}
