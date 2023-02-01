// Variables

// Serial
char opcode[4];
char readwrite;
char data_length;
char startCharacter = '!';
String data;

bool ledStatus = true;










void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
}

void setup1() {
  // put your setup code here, to run once:
  pinMode(LED_BUILTIN, OUTPUT);
}





// Wait for incoming serial data until the buffer has at least the specified number of bytes
bool waitForSerialData(int bufferSize = 1){

  // Setup function variables
  bool dataAvailable = false;

  // Wait until the specified amount of data is in the buffer
  while (!dataAvailable){

    // Update the availability status
    dataAvailable = (Serial.available() >= bufferSize);
  }

  return dataAvailable;
}

// LED function
void led(char status){

  int code = 130;

  // Off
  if (status == '0'){
    ledStatus = false;
    code = 230;
  }

  // On
  else if (status == '1'){
    ledStatus = true;
    code = 230;
  }

  // Toggle
  else if (status == '2'){
    if (ledStatus){
      ledStatus = false;
    }
    else{
      ledStatus = true;
    }
    //ledStatus = !ledStatus;
    code = 230;
  }

  // Error
  else{
    code = 430;
  }

  Serial.print(code);
}

void loop() {
  // put your main code here, to run repeatedly:

  // If there is serial data to be read, handle it
  waitForSerialData();
  char serialData = Serial.read();
  if (serialData == startCharacter){
    data = Serial.readString();
    Serial.print(220);

    led(data[5]);
    Serial.print(ledStatus);
  }
 }

void loop1() {
  if (ledStatus){
    digitalWrite(LED_BUILTIN, HIGH);
  }
  else{
    digitalWrite(LED_BUILTIN, LOW);
   }
}
