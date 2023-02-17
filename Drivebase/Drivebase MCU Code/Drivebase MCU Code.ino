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

// Personal Files
#include "FIRFilter.h"

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

#define DEBUG_OUTPUT false
#define DEBUG_INPUT true

#define CLOSED_LOOP_CONTROL false

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
static const uint16_t samplingFrequency = 50;  // Sampling frequency in Hz, default: 1000
static const unsigned long timeout = 200000;   // Amount of time with no new data before frequency is considered to be 0
static const uint8_t frequencyBufferSize = 5;  // Number of data points that the rolling frequency average is calculated from

//!M1TW4+0.3

// Control constants & variables
static volatile uint16_t controlFrequency = 50;                                                      // Control loop frequenct in Hz, default: 10
static const float kp = 0.08;                                                                         // Proportional gain of the pid controller (0.08 for 50Hz)
static const float ki = 0.06;                                                                        // Integral gain of the pid controller (0.06 for 50Hz)
static const float kd = 0.00;                                                                         // Derivative gain of the pid controller (0.004 for 50Hz)
static volatile float targetMotorFrequency[numberOfMotors];                                          // Array to store the target motor frequencies (in Hz)
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

// Create Queue
static const uint8_t sampleQueueLength = 20;
static QueueHandle_t sampleQueue;

// Create task handles
static TaskHandle_t sampleADCsHandle = nullptr;
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

// Task function to sample the ADCs at regular intervals, and stream the data to a queue
void sampleADCsTask(void* pvParameters) {

  (void)pvParameters;

  // Create local variables
  static uint16_t samples[numberOfHallSensors];  // Stores the samples

  // Setup timer so this task executes at the frequency specified in samplingFrequency
  const TickType_t xFrequency = configTICK_RATE_HZ / samplingFrequency;
  TickType_t xLastWakeTime = xTaskGetTickCount();

  // Initialise variables and pins
  for (uint8_t i = 0; i < numberOfHallSensors; i++) {
    samples[i] = 0;
    pinMode(hallSensor[i], INPUT);
  }

  // Start the loop
  while (true) {

    // Pause the task until enough time has passed
    vTaskDelayUntil(&xLastWakeTime, xFrequency);

    // Loop through each ADC
    taskENTER_CRITICAL();
    for (uint8_t i = 0; i < numberOfHallSensors; i++) {

      // Sample the ADC
      samples[i] = analogRead(hallSensor[i]);
    }
    taskEXIT_CRITICAL();

    // Send the samples to the queue
    xQueueSend(sampleQueue, &samples, 0);
  }
}

// Task function to calculate the motor frequencies
void getMotorFrequencyTask(void* pvParameters) {

  (void)pvParameters;

  // Create local variables
  static uint16_t previousSample[numberOfHallSensors];            // Stores the previous samples
  static uint16_t currentSample[numberOfHallSensors];             // Stores the current sample
  static unsigned long timeFirstOutOfRange[numberOfHallSensors];  // Store the time since the sample left the valid range (used in timeout)
  static float instantaniousMotorFrequency[numberOfHallSensors];  // Array to store the instantanious motor frequencies (in Hz)
  static bool frequencyUpdated[numberOfHallSensors];              //

  // Create low pass filters
  FIRFilter lpf[numberOfHallSensors];

  // Inititalise arrays and pins
  for (uint8_t i = 0; i < numberOfHallSensors; i++) {

    // Initialise 1D sampling arrays
    previousSample[i] = 0;

    timeFirstOutOfRange[i] = 0;

    // Initialise 1D frequency array
    instantaniousMotorFrequency[i] = 0.0;
    frequencyUpdated[i] = false;


    // Initialise low pass filters
    FIRFilterInit(&lpf[i]);
  }

  // Start the loop
  while (true) {

    // Recieve new samples from the queue
    xQueueReceive(sampleQueue, &currentSample, portMAX_DELAY);

    // Loop through each data stream
    for (uint8_t i = 0; i < numberOfHallSensors; i++) {

      // Calculate the instantanious frequency if the data is in range
      if ((currentSample[i] < 1000) && (currentSample[i] > 30)) {

        instantaniousMotorFrequency[i] = ((currentSample[i] - previousSample[i]) * ((float)samplingFrequency)) / ((float)adcRange);
        frequencyUpdated[i] = true;

        timeFirstOutOfRange[i] = micros();
      }

      // If out of range and timed out: set frequency to 0
      else if (micros() - timeFirstOutOfRange[i] > timeout) {
        instantaniousMotorFrequency[i] = 0.0;
        frequencyUpdated[i] = true;

      }

      // Else out of range but not timed out: if just left range, set ti
      else {
        frequencyUpdated[i] = false;
      }

      // Reject impossible values
      if ((instantaniousMotorFrequency[i] > maxMotorFrequency) || (instantaniousMotorFrequency[i] < -maxMotorFrequency)) {
        instantaniousMotorFrequency[i] = 0.0;
        frequencyUpdated[i] = false;
      }

      // If frequency updated, apply FIR filter and output to actualMotorFrequency
      if (frequencyUpdated[i]) {
        actualMotorFrequency[i] = FIRFilterUpdate(&lpf[i], instantaniousMotorFrequency[i]);
      }

      // Set the previous sample to the current one before the next loop
      previousSample[i] = currentSample[i];

      // if (i == 0) {
      //   Serial.print(0);
      //   Serial.print(", ");
      //   Serial.print(1023);
      //   Serial.print(", ");
      //   Serial.print(currentSample[i]);
      //   Serial.print(", ");
      //   // Serial.println(lpf[i].output);
      //   // Serial.print(", ");
      //   // Serial.print(instantaniousMotorFrequency[i] * 1000);
      //   // Serial.print(", ");
      //   // Serial.print(frequencyBuffer[i][frequencyBufferIndex[i]] * 1000);
      //   // Serial.print(", ");
      //   Serial.print(targetMotorFrequency[i]*1000);
      //   Serial.print(", ");
      //   Serial.println(actualMotorFrequency[i]*1000);
      // }
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
  static int16_t p[numberOfHallSensors];
  static bool saturated[numberOfHallSensors];

  // Make the task execute at a specified frequency
  const TickType_t xFrequency = configTICK_RATE_HZ / controlFrequency;
  TickType_t xLastWakeTime = xTaskGetTickCount();

  // Inititalise arrays and pins
  for (uint8_t i = 0; i < numberOfHallSensors; i++) {
    previousError[i] = 0.0;
    currentError[i] = 0.0;
    totalError[i] = 0.0;
    changeInError[i] = 0.0;
    u[i] = 0.0;
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

      // If using closed loop control
      if (CLOSED_LOOP_CONTROL) {

        // Calculate error terms
        currentError[i] = targetMotorFrequency[i] - actualMotorFrequency[i];

        // Turn integrator off when saturated

        if (!saturated[i]) {
          totalError[i] += currentError[i];
        }

        changeInError[i] = (currentError[i] - previousError[i]) * controlFrequency;

        // Calculate the PID control variable "u"
        u[i] = (kp * currentError[i]) + (ki * totalError[i]) + (kd * changeInError[i]);
      }

      // Else open loop control
      else {
        u[i] = targetMotorFrequency[i];
      }

      // Hysteresis to prevent oscillating around 0 when stopping
      if (((u[i] < 0.05) && (u[i] > -0.05)) && ((targetMotorFrequency[i] < 0.05) && (targetMotorFrequency[i] > -0.05))) {
        u[i] = 0.0;
      }

      // Convert u into a PWM value with an offset
      p[i] = frequencyToPWM(u[i]);

      // Limit the PWM value
      if (p[i] > (maxAllowedPWM)) {
        p[i] = maxAllowedPWM;
        saturated[i] = true;
      } else if (p[i] < -maxAllowedPWM) {
          p[i] = -maxAllowedPWM;
          saturated[i] = true;
        }
      else {
        saturated[i] = false;
      }

      // Write the direction
      if (p[i] >= 0) {
        digitalWrite(motorDIR[i], HIGH);
      } else {
        digitalWrite(motorDIR[i], LOW);
        p[i] = -p[i];
      }

      // Write the PWM value
      analogWrite(motorPWM[i], p[i]);

      // Print debug signals
      if (i == 0) {
        // Serial.print(changeInError[i]);
        // Serial.print(", ");
        // Serial.print(currentError[i]);
        // Serial.print(", ");
        // Serial.print(totalError[i]/10.0);
        // Serial.print(", ");
        // Serial.print(u[i]);
        // Serial.print(", ");
      Serial.print(actualMotorFrequency[i]);
      Serial.print(", ");
      Serial.println(targetMotorFrequency[i]);
      }
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
  }

  // Create queues
  sampleQueue = xQueueCreate(sampleQueueLength, sizeof(uint16_t[numberOfHallSensors]));  // Queue length is the number of samples, item size uint16_t)

  // Create tasks

  xTaskCreate(
    sampleADCsTask,     /* Function that implements the task */
    "GET_SAMP",         /* Text name for the task */
    1000,               /* Stack size in words, not bytes */
    nullptr,            /* Parameter passed into the task */
    5,                  /* Task priority */
    &sampleADCsHandle); /* Pointer to store the task handle */

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
    "LED",              /* Text name for the task */
    1000,               /* Stack size in words, not bytes */
    nullptr,            /* Parameter passed into the task */
    1,                  /* Task priority */
    &updateLedsHandle); /* Pointer to store the task handle */

  // Set task affinities (0x00 -> no cores, 0x01 -> C0, 0x02 -> C1, 0x03 -> C0 and C1)
  // vTaskCoreAffinitySet(sampleADCsHandle, (UBaseType_t)0x03);
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