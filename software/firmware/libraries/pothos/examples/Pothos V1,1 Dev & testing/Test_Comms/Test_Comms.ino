void setup() {
  // put your setup code here, to run once:
  pinMode(33,OUTPUT);
  digitalWrite(33,LOW);
  Serial3.begin(115200);
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  if(Serial.available())
    if(Serial.read() == 'r'){
      Serial.println(Serial3.available());
      while(Serial3.available())
        Serial.println(Serial3.read());
    }
}
