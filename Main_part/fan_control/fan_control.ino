#define relay_pin 6
int status = 0;


void setup() {
  // put your setup code here, to run once:
    pinMode(relay_pin,OUTPUT);
    Serial.begin(9600);
    Serial.setTimeout(50);

}

void loop() {
  // put your main code here, to run repeatedly:
    status = Serial.parseInt();
    if (status) {

    }

}
