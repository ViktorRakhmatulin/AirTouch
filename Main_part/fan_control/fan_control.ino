#define PIN_RELAY 6
int status = 0;


void setup() {
  // put your setup code here, to run once:
    pinMode(PIN_RELAY,OUTPUT);
    Serial.begin(9600);
    Serial.setTimeout(50);
    digitalWrite(PIN_RELAY,LOW);
    Serial.println("pin zanizhen don");

}

void loop() {
  // put your main code here, to run repeatedly:
  if(Serial.available()) {
    
    status = Serial.parseInt();
    if (status == 1) {
      digitalWrite(PIN_RELAY, HIGH);
      Serial.println("YOU SHALL NOT PASS");
    }
    else if (status == 0){
    digitalWrite(PIN_RELAY,LOW);
    Serial.println("PROHODI NE ZADERZHIVAYSYA");
    }
  }
}
