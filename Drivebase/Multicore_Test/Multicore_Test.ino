volatile bool led = false;

void setup() {
  // put your setup code here, to run once:
  
}

void setup1() {
  // put your setup code here, to run once:
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:

  led = !led;
  delay(100);

}

void loop1() {
  // put your main code here, to run repeatedly:
  if (led){
    digitalWrite(LED_BUILTIN, HIGH);
  }
  else{
    digitalWrite(LED_BUILTIN, LOW);
  }
  delay(10);

}
