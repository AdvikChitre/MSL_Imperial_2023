
// Define pins
#define PWM1 8 // 1
#define PWM2 3
#define PWM3 5

#define DIR1 11 // 2
#define DIR2 4
#define DIR3 6








// Variables

// Serial
char opcode[4];
char readwrite;
char data_length;
char startCharacter = '!';


bool ledStatus = true;
// Motor

// Targets
float motor1target = 0;
float motor2target = 0;
float motor3target = 0;

// Actual
float motor1actual = 0;
float motor2actual = 0;
float motor3actual = 0;











// Setup core 0
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

//  pinMode(LED_BUILTIN, OUTPUT);
}

// Setup core 1
void setup1() {
  // put your setup code here, to run once:
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(PWM1, OUTPUT);
  pinMode(PWM2, OUTPUT);
  pinMode(PWM3, OUTPUT);
  pinMode(DIR1, OUTPUT);
  pinMode(DIR2, OUTPUT);
  pinMode(DIR3, OUTPUT);
  // Serial1.setRX(0);
  // Serial1.setTX(1);
  // Serial1.begin(115200);
}







// User functions

// Wait for incoming serial data until the buffer has at least the specified number of bytes
bool waitForSerialData(int bufferSize = 1){

  // Setup function variables
  bool dataAvailable = false;

  // Wait until the specified amount of data is in the buffer
  while (!dataAvailable){
    delay(1);
    // Update the availability status
    dataAvailable = (Serial.available() >= bufferSize);
  }

  return dataAvailable;
}

// LED function
void led(int status){

  int code = 130;

  // Off
  if (status == 0){
    ledStatus = false;
    code = 230;
  }

  // On
  else if (status == 1){
    ledStatus = true;
    code = 230;
  }

  // Toggle
  else if (status == 2){
    ledStatus = !ledStatus;
    code = 230;
  }

  // Error
  else{
    code = 430;
  }

  // if (ledStatus){
  //   digitalWrite(LED_BUILTIN, LOW);
  //   code = 230;
  // }
  // else{
  //   digitalWrite(LED_BUILTIN, HIGH);
  //   code = 230;
  // }
  
  
  Serial.print(code);
}

// Single motor write function
void motor_write(int motor, float motor_speed, char actualtarget='T'){
  // TODO make function work with more motors

  // Write actual speed (speed jumps to target speed)
  if (actualtarget == 'A'){

    // Set motorxtarget
    motor1target = motor_speed;
    
    // Set the direction
    if (motor_speed < 0){
      digitalWrite(DIR1, LOW);
      motor_speed = -motor_speed;
    }
    else{
      digitalWrite(DIR1, HIGH);
    }

    // Convert speed into analogue writable int
    int analog_speed = int((motor_speed/3) * 255);
    
    // Write speed
    analogWrite(PWM1, analog_speed);
  }
}

// Single motor read function
void motor_read(int motor, char actualtarget){

  // TODO make function work with more motors

  // Determines which variable to return and prints it to serial
  if ((motor == 1) && (actualtarget == 'A')){
    Serial.write(motor1actual);
  }
  if ((motor == 1) && (actualtarget == 'T')){
    Serial.write(motor1target);
  }
  if ((motor == 2) && (actualtarget == 'A')){
    Serial.write(motor2actual);
  }
  if ((motor == 2) && (actualtarget == 'T')){
    Serial.write(motor2target);
  }
  if ((motor == 3) && (actualtarget == 'A')){
    Serial.write(motor3actual);
  }
  if ((motor == 3) && (actualtarget == 'T')){
    Serial.write(motor3target);
  }
}

// Handle incoming serial data
void handle_serial(){
  opcode[3] = '\0';
  // Reads the opcode, whether it is read or write, and the length of the data
  waitForSerialData(5);
  opcode[0]  = Serial.read();
  opcode[1]  = Serial.read();
  opcode[2]  = Serial.read();
  readwrite  = Serial.read();
  data_length = Serial.read();
  
  char data[data_length];
  
  // Reads the data if the incoming command is a write command
  if (readwrite == 'W'){

    // Wait until all the data is in the buffer before reading it
    waitForSerialData(data_length);

    // Reads the data from the buffer
    for (int i=0; i<data_length; i++){
      
      data[i] = Serial.read();
    }
  } 
  
  // Print successfully read code
  Serial.print(220);

  // Decoding the instruction, and calling the appropriate function

  // LED command
  if ((opcode[0] == 'L') && (opcode[1] == 'E') && (opcode[2] == 'D')){
    led(atoi(data));
    }

  // Motor command
  if (opcode[0] == 'M'){

    // Motor write
    if (readwrite == 'W'){
      motor1target = atof(data);
      Serial.print(240);
      }
      
    // Motor read
    else if (readwrite == 'R'){
      motor_read(opcode[1], opcode[2]);
      }
    }
    Serial.flush();
  }









void loop() {
  // put your main code here, to run repeatedly:

  // If there is serial data to be read, handle it
//  waitForSerialData();
  char serialData = Serial.read();
  if (serialData == startCharacter){
    handle_serial();
  }

 }

void loop1() {
  // Serial1.print(ledStatus);
  motor_write(1, motor1target, 'A');
  if (ledStatus){
    digitalWrite(LED_BUILTIN, LOW);
  }
  else{
    digitalWrite(LED_BUILTIN, HIGH);

   }
}