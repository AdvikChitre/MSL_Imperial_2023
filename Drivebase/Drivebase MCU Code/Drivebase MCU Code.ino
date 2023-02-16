/* 
TODO:
- Implement FIR filter on sampled signal
- Tidy up code
- Continue usbHandle()
- Add IMU over I2C and process data
- Add hysterisis to PID controller
- Remove offset from PID controlelr
*/

//-------------------------------- Includes ---------------------------------------------

#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <semphr.h>

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

#include <map>

#include <math.h>

#include "AS5600.h"

AS5600 as5600;

//-------------------------------- Defines ----------------------------------------------

#define PWM1 9
#define PWM2 8
#define PWM3 13
#define PWM4 12

#define DIR1 10
#define DIR2 11
#define DIR3 3
#define DIR4 7

#define HALL1 A0
#define HALL2 A1
#define HALL3 A2

#define BTN 14

#define LED1 0
#define LED2 1

#define IMU_INT 6
#define IMU_SCL 5
#define IMU_SDA 4

#define DEBUG_OUTPUT true
#define DEBUG_INPUT true

//-------------------------------- Global Variables -------------------------------------

// Pin arrays
static const uint8_t hallSensor[] = { HALL1, HALL2, HALL3 };   // Array of hall effect sensor pins
static const uint8_t motorPWM[] = { PWM1, PWM2, PWM3, PWM4 };  // Array of motor PWM pins
static const uint8_t motorDIR[] = { DIR1, DIR2, DIR3, DIR4 };  // Array of motor DIR pins
static const uint8_t led[] = { LED_BUILTIN, LED1, LED2 };      // Array of LED pins

static const uint8_t numberOfHallSensors = sizeof(hallSensor);  // Number of hall effect sensors
static const uint8_t numberOfMotors = sizeof(motorPWM);         // Number of motors
static const uint8_t numberOfLEDs = sizeof(led);                // Number of motors

// Analogue constants
static const uint32_t pwmFrequency = 20000;                  // Sets the PWM frequency (in Hz)
static const uint8_t pwmResolution = 8;                      // Sets the PWM resolution (in bits), default is 8
static const uint16_t pwmRange = pow(2, pwmResolution) - 1;  // Calculates the PWM range based on the resolution
static const uint8_t adcResolution = 10;                     // Sets the ADC resolution
static const uint16_t adcRange = pow(2, adcResolution) - 1;  // Calculates the ADC range

// Sampling constants
static const uint16_t samplingFrequency = 50;   // Sampling frequency in Hz, default: 1000
static const unsigned long timeout = 200000;    // Amount of time with no new data before frequency is considered to be 0
static const uint8_t frequencyBufferSize = 20;  // Number of data points that the rolling frequency average is calculated from

// Control constants & variables
static volatile uint16_t controlFrequency = 50;                                                      // Control loop frequenct in Hz, default: 10
static const float kp = 0.4;                                                                         // Proportional gain of the pid controller (0.3 for 10Hz)
static const float ki = 0.02;                                                                        // Integral gain of the pid controller (0.1 for 10Hz)
static const float kd = 0.0;                                                                         // Derivative gain of the pid controller (0.001 for 10Hz)
static volatile float targetMotorFrequency[numberOfMotors];                                          // Array to store the target motor frequencies (in Hz)
static volatile bool motorDirection[numberOfMotors];                                                 // Array to store the direction of the motor (true = +ve, false = -ve)
static volatile float actualMotorFrequency[numberOfHallSensors];                                     // Array to store the average motor frequencies (in Hz) across a number of samples
static const uint8_t wheelDiameter = 100;                                                            // Wheel diameter in mm, used to convert speeds to frequencies
static const float maxMotorFrequency = 2.93;                                                         // Max motor frequency in Hz, default : 13.02
static const uint8_t maxAllowedSpeedPercent = 30;                                                    // Max allowed speed given as a percentage of actual max speed
static const float maxAllowedMotorFrequency = maxMotorFrequency * (maxAllowedSpeedPercent / 100.0);  // Max allowed motor frequency in Hz
static const uint16_t maxAllowedPWM = pwmRange * (maxAllowedSpeedPercent / 100.0);                   // Max allowed value in analogWrite()

// LED constants & variables
static volatile bool ledStatus[numberOfLEDs];  // Stores the state of each LED

// USB constants & variables
static const char startCharacter = '!';  // Sets the USB start character

// Create task handles
static TaskHandle_t getMotorFrequencyHandle = nullptr;
static TaskHandle_t pidControllerHandle = nullptr;
static TaskHandle_t usbHandle = nullptr;
static TaskHandle_t updateLedsHandle = nullptr;
static TaskHandle_t debugHandle = nullptr;



//-------------------------------- Functions --------------------------------------------

// Convert frequency in Hz to speed in m/s
float frequencyToSpeed(float frequency) {
  return (float)(frequency * (wheelDiameter / 1000) * PI);
}

// Convert speed in m/s to frequency in Hz
float speedToFrequency(float speed) {
  return (float)(speed / ((wheelDiameter / 1000) * PI));
}

int16_t frequencyToPWM(float frequency) {
  return (int16_t)((frequency * pwmRange) / (maxMotorFrequency));
}

// Function to print the current status of each task, used for debugging
std::map<eTaskState, const char*> eTaskStateName{ { eReady, "Ready" }, { eRunning, "Running" }, { eBlocked, "Blocked" }, { eSuspended, "Suspended" }, { eDeleted, "Deleted" } };
void taskStatusUpdate() {
  int tasks = uxTaskGetNumberOfTasks();
  TaskStatus_t* pxTaskStatusArray = new TaskStatus_t[tasks];
  unsigned long runtime;
  tasks = uxTaskGetSystemState(pxTaskStatusArray, tasks, &runtime);
  Serial.printf("\n# Tasks: %d\n", tasks);
  Serial.println("ID NAME,            STATE        PRIO     CYCLES           MEMORY REMAINING");
  for (int i = 0; i < tasks; i++) {
    Serial.printf("%d: %-16s %-12s %-8d %-16lu %d\n", i, pxTaskStatusArray[i].pcTaskName, eTaskStateName[pxTaskStatusArray[i].eCurrentState], (int)pxTaskStatusArray[i].uxCurrentPriority, pxTaskStatusArray[i].ulRunTimeCounter, pxTaskStatusArray[i].usStackHighWaterMark);
  }
  delete[] pxTaskStatusArray;
}

//-------------------------------- USB Functions --------------------------------------------

// Wait for incoming serial data until the buffer has at least the specified number of bytes
bool waitForSerialData(uint8_t bufferSize = 1) {

  // Setup function variables
  bool dataAvailable = false;

  // Wait until the specified amount of data is in the buffer
  while (!dataAvailable) {
    vTaskDelay(pdMS_TO_TICKS(5));

    // Update the availability status
    dataAvailable = (Serial.available() >= bufferSize);
  }

  return dataAvailable;
}

// LED function
uint16_t ledWrite(uint8_t led = 0, uint8_t status = 0) {

  uint16_t code = 130;

  // Off
  if (status == 0) {
    ledStatus[led] = false;
    code = 230;
  }

  // On
  else if (status == 1) {
    ledStatus[led] = true;
    code = 230;
  }

  // Toggle
  else if (status == 2) {
    ledStatus[led] = !ledStatus[led];
    code = 230;
  }

  // Error
  else {
    code = 430;
  }

  return code;
}

// Handle incoming serial data
void handleSerial() {

  // Create variables to store incoming mesage
  char opcode[] = "XXX";
  char readwrite;
  uint8_t data_length;

  // Reads the opcode, whether it is read or write, and the length of the data
  waitForSerialData((uint8_t)5);
  opcode[0] = Serial.read();
  opcode[1] = Serial.read();
  opcode[2] = Serial.read();
  readwrite = Serial.read();
  data_length = ((uint8_t)Serial.read()) - 48;

  char data[data_length];

  // Reads the data if the incoming command is a write command
  if (readwrite == 'W') {

    // Wait until all the data is in the buffer before reading it
    waitForSerialData(data_length);

    // Reads the data from the buffer
    for (uint8_t i = 0; i < data_length; i++) {
      data[i] = Serial.read();
    }
  }

  // Print successfully read code
  if (!DEBUG_INPUT) {
    Serial.print(220);
  }

  // Decoding the instruction, and calling the appropriate function

  // LED command
  if (strcmp(opcode, "LED") == 0) {
    Serial.print(ledWrite((data[0] - 48), (data[1] - 48)));
  }

  // Motor command
  if (opcode[0] == 'M') {
    if (opcode[2] == 'T') {
      targetMotorFrequency[opcode[1] - 49] = atof(data);
    }  //!M1TW4+0.3
  }
}

//-------------------------------- Task Functions ----------------------------------------

// Task function to calculate the motor frequencies
void getMotorFrequencyTask(void* pvParameters) {

  (void)pvParameters;

  // Create local variables

  static uint16_t previousSample[numberOfHallSensors];     // Stores the previous samples
  static uint16_t currentSample;                           // Stores the current sample
  static unsigned long previousTime[numberOfHallSensors];  // Stores the time when previous samples were taken
  static unsigned long currentTime;                        // Stores the time when the current sample was taken
  static unsigned long interval[numberOfHallSensors];      // Time between samples

  static float instantaniousMotorFrequency[numberOfHallSensors];           // Array to store the instantanious motor frequencies (in Hz)
  static bool frequencyUpdated[numberOfHallSensors];                       //
  static float frequencyBuffer[numberOfHallSensors][frequencyBufferSize];  // Array to store the previous samples used to calculate the rolling average
  static uint8_t frequencyBufferIndex[numberOfHallSensors];                // Index in the frequencyBuffer that will be read and then written next
  static float totalFrequency[numberOfHallSensors];                        // Stores the sum of frequencies used to calculate the rolling average


  // Setup timer so this task executes at the frequency specified in samplingFrequency
  const TickType_t xFrequency = configTICK_RATE_HZ / samplingFrequency;
  TickType_t xLastWakeTime = xTaskGetTickCount();

  // Inititalise arrays and pins
  for (uint8_t i = 0; i < numberOfHallSensors; i++) {

    // Initialise 1D sampling arrays
    previousSample[i] = 0;
    previousTime[i] = 0;
    interval[i] = 0;

    // Initialise 1D frequency array
    instantaniousMotorFrequency[i] = 0;
    frequencyUpdated[i] = false;

    // Initialise 1D rolling average arrays
    frequencyBufferIndex[i] = 0;
    totalFrequency[i] = 0;
    actualMotorFrequency[i] = 0;

    // Initialise pins
    pinMode(hallSensor[i], INPUT);

    // Initialise 2D arrays
    for (uint8_t j = 0; j < frequencyBufferSize; j++) {
      frequencyBuffer[i][j] = 0;
    }
  }

  // Start the loop
  while (true) {

    // Pause the task until enough time has passed
    vTaskDelayUntil(&xLastWakeTime, xFrequency);

    // Loop through each hall sensor
    for (uint8_t i = 0; i < numberOfHallSensors; i++) {

      // Sample the ADC (this is a critical section since we don't want an interrupt between the sample and recording the time it was taken)
      taskENTER_CRITICAL();
      currentSample = analogRead(hallSensor[i]);
      currentTime = micros();
      taskEXIT_CRITICAL();

      // Calculate the time since the last valid sample
      interval[i] = currentTime - previousTime[i];

      //
      if ((currentSample < 1000) && (currentSample > 30)) {

        instantaniousMotorFrequency[i] = ((currentSample - previousSample[i]) * 1000000.0) / ((float)(interval[i] * adcRange));

        // // Round to 0 if below
        // if (instantaniousMotorFrequency[i] < 0.5){
        //   instantaniousMotorFrequency[i] = 0.0;
        // }

        previousSample[i] = currentSample;
        previousTime[i] = currentTime;
        frequencyUpdated[i] = true;

      } else if (interval[i] > timeout) {
        instantaniousMotorFrequency[i] = 0.0;
        // previousTime[i] = currentTime;
        frequencyUpdated[i] = true;
      }

      else {
        // previousTime[i] = currentTime;
        frequencyUpdated[i] = false;
      }


      //
      if ((instantaniousMotorFrequency[i] > maxMotorFrequency) || (instantaniousMotorFrequency[i] < -maxMotorFrequency)) {
        instantaniousMotorFrequency[i] = 0.0;
        frequencyUpdated[i] = false;
      }

      // If the calculated frequency is valid, work out the rolling average
      if (frequencyUpdated[i]) {



        // Update the total frequency array
        totalFrequency[i] += instantaniousMotorFrequency[i];
        totalFrequency[i] -= frequencyBuffer[i][frequencyBufferIndex[i]];

        // Add the current frequency to the buffer array
        frequencyBuffer[i][frequencyBufferIndex[i]] = instantaniousMotorFrequency[i];

        // Calculate the new average
        actualMotorFrequency[i] = totalFrequency[i] / frequencyBufferSize;

        // Increment the frequencyBufferIndex and reset if needed
        if (++frequencyBufferIndex[i] >= frequencyBufferSize) {
          frequencyBufferIndex[i] = 0;
        }
      }
      if (i == 0) {
        Serial.print(0);
        Serial.print(", ");
        Serial.print(1023);
        Serial.print(", ");
        Serial.print(currentSample);
        Serial.print(", ");
        Serial.println(instantaniousMotorFrequency[i] * 1000);
        // Serial.print(", ");
        // Serial.print(frequencyBuffer[i][frequencyBufferIndex[i]] * 1000);
        // Serial.print(", ");
        // Serial.println(actualMotorFrequency[i] * 1000);
      }
    }
  }
}

// Task function to run the pid control loop
void pidControllerTask(void* pvParameters) {

  (void)pvParameters;

  // Create local variables
  static float previousError[numberOfHallSensors];
  static float currentError[numberOfHallSensors];
  static float totalError[numberOfHallSensors];
  static float changeInError[numberOfHallSensors];
  static float u[numberOfHallSensors];
  static uint16_t p[numberOfHallSensors];
  static bool saturated[numberOfHallSensors];
  static uint16_t offset = 256;

  // Make the task execute at a specified frequency
  const TickType_t xFrequency = configTICK_RATE_HZ / controlFrequency;
  TickType_t xLastWakeTime = xTaskGetTickCount();

  // Inititalise arrays and pins
  for (uint8_t i = 0; i < numberOfHallSensors; i++) {
    previousError[i] = 0;
    currentError[i] = 0;
    totalError[i] = 0;
    changeInError[i] = 0;
    u[i] = 0;
    p[i] = 0;
    saturated[i] = false;
    pinMode(motorPWM[i], OUTPUT);
    pinMode(motorDIR[i], OUTPUT);
  }

  // Start the loop
  while (true) {

    // Pause the task until enough time has passed
    vTaskDelayUntil(&xLastWakeTime, xFrequency);

    // Loops through each motor
    for (uint8_t i = 0; i < numberOfHallSensors; i++) {

      // Calculate error terms
      if (motorDirection[i]) {
        currentError[i] = targetMotorFrequency[i] - actualMotorFrequency[i];
      } else {
        currentError[i] = targetMotorFrequency[i] + actualMotorFrequency[i];
      }
      if (!saturated[i]) {
        totalError[i] += currentError[i];
      }
      changeInError[i] = (currentError[i] - previousError[i]) * controlFrequency;

      // Calculate the PID control variable "u"
      u[i] = (kp * currentError[i]) + (ki * totalError[i]) + (kd * changeInError[i]);

      // Convert u into a PWM value with an offset
      p[i] = offset + frequencyToPWM(u[i]);

      // Limit the PWM value
      if (p[i] > (offset + maxAllowedPWM)) {
        p[i] = offset + maxAllowedPWM;
        saturated[i] = true;
      } else if (p[i] < (offset - maxAllowedPWM)) {
        p[i] = offset - maxAllowedPWM;
        saturated[i] = true;
      } else {
        saturated[i] = false;
      }

      // Write the direction
      if (p[i] >= offset) {
        digitalWrite(motorDIR[i], HIGH);
        motorDirection[i] = true;
        p[i] = p[i] - offset;
      } else {
        digitalWrite(motorDIR[i], LOW);
        motorDirection[i] = false;
        p[i] = offset - p[i];
      }

      // Write the PWM value
      // analogWrite(motorPWM[i], p[i]);
      analogWrite(motorPWM[i], frequencyToPWM(targetMotorFrequency[i]));

      // Print debug signals
      // if (i == 0) {
      //   // Serial.print(changeInError[i]);
      //   // Serial.print(", ");
      //   // Serial.print(currentError[i]);
      //   // Serial.print(", ");
      //   // Serial.print(totalError[i]/10.0);
      //   // Serial.print(", ");
      //   // Serial.print(u[i]);
      //   // Serial.print(", ");
      // Serial.print(instantaniousMotorFrequency[i]);
      // Serial.print(", ");
      // Serial.println(targetMotorFrequency[i]);
      // }
    }
  }
}

// Task to handle serial communication with host
void usbTask(void* pvParameters) {

  (void)pvParameters;

  // Create local variables
  static char serialData;

  while (true) {

    // Wait for data
    waitForSerialData();

    // Read character
    serialData = Serial.read();

    // If read character it is the start of a message then handle the message
    if (serialData == startCharacter) {
      handleSerial();
    }
  }
}

// Task to update the LEDs
void updateLedsTask(void* pvParameters) {

  (void)pvParameters;

  // Initialise all LEDs as off and set the pins as outputs
  for (uint8_t i = 0; i < numberOfLEDs; i++) {
    ledStatus[i] = false;
    pinMode(led[i], OUTPUT);
  }

  // Main loop
  while (true) {

    // Check status of each LED and update accordingly
    for (uint8_t i = 0; i < numberOfLEDs; i++) {
      if (ledStatus[i]) {
        digitalWrite(led[i], HIGH);
      } else {
        digitalWrite(led[i], LOW);
      }
    }

    // Delay
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

// Task to print debug messages
void debugTask(void* pvParameters) {

  (void)pvParameters;

  pinMode(BTN, INPUT);
  pinMode(motorPWM[0], OUTPUT);

  while (true) {
    // analogWrite(motorPWM[0], 0);
    // ledStatus[2] = false;
    // vTaskDelay(pdMS_TO_TICKS(5000));
    // analogWrite(motorPWM[0], 35);
    // ledStatus[2] = true;
    // vTaskDelay(pdMS_TO_TICKS(5000));

    if (!digitalRead(BTN)) {
      targetMotorFrequency[0] = 0.3;

      ledStatus[2] = true;
      vTaskDelay(pdMS_TO_TICKS(100));

    } else {
      targetMotorFrequency[0] = 0;
      ledStatus[2] = false;
      vTaskDelay(pdMS_TO_TICKS(100));
    }

    // Wire.beginTransmission(0x3C);
    // // sprintf(b, "pass %d", p++);
    // Wire.write(0x05);
    // Wire.write(0x0F);
    // Wire.write(0xFF);
    // Wire.endTransmission();

    vTaskDelay(pdMS_TO_TICKS(50));
    //   taskStatusUpdate();
  }
}

//-------------------------------- Setups -----------------------------------------------

void setup() {

  // Setup USB
  Serial.begin(115200);
  delay(1000);

  // Define analogue parameters
  analogWriteFreq(pwmFrequency);
  analogWriteResolution(pwmResolution);
  analogReadResolution(adcResolution);

  // Initialise arrays
  for (uint8_t i = 0; i < numberOfMotors; i++) {
    targetMotorFrequency[i] = 0;
    motorDirection[i] = false;
  }

  // Create tasks
  xTaskCreate(
    getMotorFrequencyTask,     /* Function that implements the task */
    "GET_FREQ",                /* Text name for the task */
    1000,                      /* Stack size in words, not bytes */
    nullptr,                   /* Parameter passed into the task */
    5,                         /* Task priority */
    &getMotorFrequencyHandle); /* Pointer to store the task handle */

  xTaskCreate(
    pidControllerTask,     /* Function that implements the task */
    "PID_CTRL",            /* Text name for the task */
    1000,                  /* Stack size in words, not bytes */
    nullptr,               /* Parameter passed into the task */
    5,                     /* Task priority */
    &pidControllerHandle); /* Pointer to store the task handle */

  xTaskCreate(
    usbTask,     /* Function that implements the task */
    "SERIAL",    /* Text name for the task */
    1000,        /* Stack size in words, not bytes */
    nullptr,     /* Parameter passed into the task */
    1,           /* Task priority */
    &usbHandle); /* Pointer to store the task handle */

  xTaskCreate(
    updateLedsTask,     /* Function that implements the task */
    "LED_UPDATE",       /* Text name for the task */
    1000,               /* Stack size in words, not bytes */
    nullptr,            /* Parameter passed into the task */
    1,                  /* Task priority */
    &updateLedsHandle); /* Pointer to store the task handle */

  // Set task affinities (0x00 -> no cores, 0x01 -> C0, 0x02 -> C1, 0x03 -> C0 and C1)
  // vTaskCoreAffinitySet(getMotorFrequencyHandle, (UBaseType_t)0x03);
  // vTaskCoreAffinitySet(pidControllerHandle, (UBaseType_t)0x03);
  // vTaskCoreAffinitySet(usbHandle, (UBaseType_t)0x03);
  // vTaskCoreAffinitySet(updateLedsHandle, (UBaseType_t)0x03);

  // Enable the debug task if configured to do so
  if (DEBUG_OUTPUT) {
    xTaskCreate(
      debugTask,     /* Function that implements the task */
      "DEBUG",       /* Text name for the task */
      1000,          /* Stack size in words, not bytes */
      nullptr,       /* Parameter passed into the task */
      1,             /* Task priority */
      &debugHandle); /* Pointer to store the task handle */
    // vTaskCoreAffinitySet(debugHandle, (UBaseType_t)0x01);
  }

  // Starts the scheduler (may be unecessary? delete if buggy)
  //vTaskStartScheduler();

  // Delete "setup" and "loop" task
  vTaskDelete(NULL);
}

void setup1() {
  // Delete "setup1" and "loop1" task
  vTaskDelete(NULL);
}

//-------------------------------- Loops -----------------------------------------------

void loop() {
  // Should never get to this point
}

void loop1() {
  // Should never get to this point
}
