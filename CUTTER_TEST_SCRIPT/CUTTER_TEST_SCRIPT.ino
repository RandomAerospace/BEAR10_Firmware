//HEATER AND CUTTER PINS
#define PIN_HEATER 25
#define PIN_CUTTER 26



void setup() {
  // put your setup code here, to run once:
  pinMode(PIN_HEATER, OUTPUT);
  digitalWrite(PIN_HEATER, LOW);
  pinMode(PIN_CUTTER, OUTPUT);
  digitalWrite(PIN_CUTTER, LOW);
  // put your main code here, to run repeatedly:
  Serial.print("CUTTING IN");
  for (int i=0;i>30;i++){ //30 second count down
    Serial.println(i);
    delay(1000);
  }
  digitalWrite(PIN_CUTTER,HIGH);
  delay(10000); //state high for 10s
  digitalWrite(PIN_CUTTER,LOW);
  delay(5000);

}

void loop() {

}
