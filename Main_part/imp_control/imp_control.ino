#include <Servo.h>
Servo motor;
String incomingByte;
Servo myservo;
String str;
int val=0;  
void setup() 
{
Serial.begin(115200);
myservo.attach(9);
}
void loop(){
  if (Serial.available() > 0) {  //если есть доступные данные
// считываем байт
        incomingByte = Serial.readStringUntil('\n');
// delay(10);


if(incomingByte.equals("init")){
  init1();
  incomingByte = "";
}

if(incomingByte.startsWith("suck")){
  int data=incomingByte.substring(4).toInt();
  suck(data);
  incomingByte = "";
}

if(incomingByte.equals("hold")){
  hold();
  incomingByte = "";
}

if(incomingByte.equals("unsuck")){
  unsuck();
  incomingByte = "";
}

if(incomingByte.equals("stop")){
  stop();
  incomingByte = "";
}


if(incomingByte.startsWith("go to")){
  int data=incomingByte.substring(5).toInt();
  // Serial.println(incomingByte.substring(5));
  go_to(data);
  incomingByte = "";
  
}

if(incomingByte.equals("go2")){
  go_to_2();
  incomingByte = "";
}

}
}

void go_to(int val){
  myservo.write(val);   
}

void go_to_2(){
  for (int i=0;i<=180;i++){
    myservo.write(i);
    delay(15);
  }
}

void unsuck(){
  // Serial.print("i am in unsuck");  
  motor.writeMicroseconds(1500);//1550
  delay(3000);
  motor.writeMicroseconds(1000);
}

void suck(int pwm){  
  motor.writeMicroseconds(pwm);
  // delay(2000);
  // motor.writeMicroseconds(1000);
}

void init1(){
  // Serial.print("i am in init"); 
  motor.attach(3);
}

void stop(){
  // Serial.print("i am in stop"); 
  motor.detach();
}

void hold(){
  // Serial.print("i am in hold"); 
   motor.writeMicroseconds(1000); 
  
}