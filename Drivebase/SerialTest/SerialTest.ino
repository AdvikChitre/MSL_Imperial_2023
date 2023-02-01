void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
}

void loop() {
  // put your main code here, to run repeatedly:
  float m1_speed = -1.241;
  Serial.println(m1_speed);
  delay(1000);
}
