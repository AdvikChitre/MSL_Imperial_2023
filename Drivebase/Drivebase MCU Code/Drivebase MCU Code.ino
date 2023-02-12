/* 
TODO:
- Add negative speeds
- Check for integral term saturation and clamp
- Add more magnets to wheel
- Tidy up code
- Continue usbHandle()
- Add IMU over I2C and process data
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

#define DEBUG_ENABLED true

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
static const uint16_t samplingFrequency = 200;        // Sampling frequency in Hz, default: 1000
static const uint16_t upperThreshold = 570;           // Upper threshold value that is compared to the analogRead() value to count the time interval between oscillations
static const uint16_t lowerThreshold = 490;           // Lower threshold value that is compared to the analogRead() value to count the time interval between oscillations
static const unsigned long timeoutConstant = 500000;  // timeout = timeoutConstant / instantaniousFrequency
static const uint8_t frequencyBufferSize = 10;        // Number of data points that the rolling frequency average is calculated from

// Control constants & variables
static volatile uint16_t controlFrequency = 50;                                                      // Control loop frequenct in Hz, default: 10                                                                      // Proportional gain of the pid controller (0.3 for 10Hz)
static const float kp = 0.4;                                                                         //0.3;                                                                        // Proportional gain of the pid controller (0.3 for 10Hz)
static const float ki = 0.04;                                                                        //0.1;                                                                         // Integral gain of the pid controller (0.1 for 10Hz)
static const float kd = 0.001;                                                                         //0.0;                                                                          // Derivative gain of the pid controller (0.001 for 10Hz)
static volatile float targetMotorFrequency[numberOfMotors];                                          // Array to store the target motor frequencies (in Hz)
static volatile float instantaniousMotorFrequency[numberOfHallSensors];                              // Array to store the instantanious motor frequencies (in Hz)
static volatile float actualMotorFrequency[numberOfHallSensors];                                     // Array to store the rolling averages of motor frequencies (in Hz)
static const uint8_t wheelDiameter = 100;                                                            // Wheel diameter in mm, used to convert speeds to frequencies
static const float maxMotorFrequency = 2.93;                                                         // Max motor frequency in Hz, default : 13.02
static const uint8_t maxAllowedSpeedPercent = 80;                                                    // Max allowed speed given as a percentage of actual max speed
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
  Serial.print(220);

  // Decoding the instruction, and calling the appropriate function

  // LED command
  if (strcmp(opcode, "LED") == 0) {
    Serial.print(ledWrite((data[0] - 48), (data[1] - 48)));
  }

  // Motor command
  if (opcode[0] == 'M') {
    if (opcode[2] == 'T') {
    }
  }
}

//-------------------------------- Task Functions ----------------------------------------

// Task function to calculate the motor frequencies
void getMotorFrequencyTask(void* pvParameters) {

  (void)pvParameters;

  // Create local variables
  static bool previousSampleUnderUpperThreshold[numberOfHallSensors];      // Stores whether the previous sample was under the threshold or not
  static bool previousSampleAboveLowerThreshold[numberOfHallSensors];      // Stores whether the previous sample was above the lower threshold or not
  static uint16_t currentSample;                                           // Stores the current sample
  static unsigned long timePrevious[numberOfHallSensors][4];               // Stores the previous time a peak occured for each hall sensor
  static unsigned long timeCurrent;                                        // The time when the current sample was taken
  static unsigned long timeSinceLastPeak[4];                                       // Stores the time since the last peak (float because floating point arithmatic is done with it)
  static float frequencyBuffer[numberOfHallSensors][frequencyBufferSize];  // Array to store the previous samples used to calculate the rolling average
  static uint8_t frequencyBufferIndex = 0;                                 // Index in the frequencyBuffer that will be read and then written next
  static float totalFrequency;                                             // Stores the sum of frequencies used to calculate the rolling average
  static bool frequencyUpdate = false;                                     // true when an update to a frequency has occured
  static unsigned long timeout;
  static bool updated = false;

  // Setup timer so this task executes at the frequency specified in samplingFrequency
  const TickType_t xFrequency = configTICK_RATE_HZ / samplingFrequency;
  TickType_t xLastWakeTime = xTaskGetTickCount();

  // Inititalise arrays and pins
  for (uint8_t i = 0; i < numberOfHallSensors; i++) {

    // Initialise 1D arrays
    previousSampleUnderUpperThreshold[i] = true;
    previousSampleAboveLowerThreshold[i] = true;
    //    timePrevious[i] = 0;
    instantaniousMotorFrequency[i] = 0;
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

      // Sample the ADC and calculate the time since the last peak (this is a critical section since we don't want an interruption between the sample and recording the time it was taken)
      taskENTER_CRITICAL();
      currentSample = analogRead(hallSensor[i]);
      timeCurrent = micros();
      taskEXIT_CRITICAL();
      if (i == 0) {
        // Serial.print(upperThreshold);
        // Serial.print(", ");
        // Serial.print(lowerThreshold);
        // Serial.print(", ");
        // Serial.print(1000);
        // Serial.print(", ");
        // Serial.print(0);
        // Serial.print(", ");
        // Serial.println(currentSample);
      }
      timeSinceLastPeak[0] = timeCurrent - timePrevious[i][0];
      timeSinceLastPeak[1] = timeCurrent - timePrevious[i][1];
      timeSinceLastPeak[2] = timeCurrent - timePrevious[i][2];
      timeSinceLastPeak[3] = timeCurrent - timePrevious[i][3];

      // Check if the current sample is above the upper threshold and the previous sample was below, if true there is a peak
      if ((currentSample > upperThreshold) && (previousSampleUnderUpperThreshold[i])) {

        // Work out the instantanious frequency based on the time since last peak
        instantaniousMotorFrequency[i] = 250000 / ((float) timeSinceLastPeak[0]);

        // Set the frequencyUpdate flag
        frequencyUpdate = true;

        // Update variables for next loop
        timePrevious[i][0] = timeCurrent;
        updated = true;
      }
      // Check if the current sample is above the upper threshold and the previous sample was below, if true there is a peak
      else if ((currentSample < upperThreshold) && (!previousSampleUnderUpperThreshold[i])) {

        // Work out the instantanious frequency based on the time since last peak
        instantaniousMotorFrequency[i] = 250000 / ((float)timeSinceLastPeak[1]);

        // Set the frequencyUpdate flag
        frequencyUpdate = true;

        // Update variables for next loop
        timePrevious[i][1] = timeCurrent;
        updated = true;
      }
      // Check if the current sample is above the upper threshold and the previous sample was below, if true there is a peak
      else if ((currentSample < lowerThreshold) && (previousSampleAboveLowerThreshold[i])) {

        // Work out the instantanious frequency based on the time since last peak
        instantaniousMotorFrequency[i] = 250000 / ((float)timeSinceLastPeak[2]);

        // Set the frequencyUpdate flag
        frequencyUpdate = true;

        // Update variables for next loop
        timePrevious[i][2] = timeCurrent;
        updated = true;
      }
      // Check if the current sample is above the upper threshold and the previous sample was below, if true there is a peak
      else if ((currentSample > lowerThreshold) && (!previousSampleAboveLowerThreshold[i])) {

        // Work out the instantanious frequency based on the time since last peak
        instantaniousMotorFrequency[i] = 250000 / ((float)timeSinceLastPeak[3]);

        // Set the frequencyUpdate flag
        frequencyUpdate = true;

        // Update variables for next loop
        timePrevious[i][3] = timeCurrent;
        updated = true;
      }

      // if (instantaniousMotorFrequency[i] != 0.0) {
        timeout = timeoutConstant / instantaniousMotorFrequency[i];
      // }
      // else {
      //   timeout = timeoutConstant;
      // }

      if (i == 0) {
        // Serial.print(timeout / 1024);
        // Serial.print(", ");
        // Serial.print(instantaniousMotorFrequency[i] * 100.0);
        // Serial.print(", ");
        // Serial.print(upperThreshold);
        // Serial.print(", ");
        // Serial.print(lowerThreshold);
        // Serial.print(", ");
        // Serial.print(1000);
        // Serial.print(", ");
        // Serial.print(0);
        // Serial.print(", ");
        // Serial.println(currentSample);
      }
      // If no peak after timeout time, or previous frequency 0 then frequency is considered to be 0
      if (((timeSinceLastPeak[0] > timeout) || (timeSinceLastPeak[1] > timeout) || (timeSinceLastPeak[2] > timeout) || (timeSinceLastPeak[3] > timeout)) && !updated) {
        // Serial.println(1500);
        // Set the frequency to 0
        instantaniousMotorFrequency[i] = 0;

        // Set the frequencyUpdate flag
        frequencyUpdate = true;
      } else {
        updated = false;
      }

      // If an update to the frequency occured, recalculate the rolling average
      // if (frequencyUpdate) {

      //   // Update the total frequency and frequency buffer
      //   totalFrequency = totalFrequency - frequencyBuffer[i][frequencyBufferIndex];
      //   frequencyBuffer[i][frequencyBufferIndex] = instantaniousMotorFrequency[i];
      //   totalFrequency = totalFrequency + instantaniousMotorFrequency[i];

      //   // Calculate the average and update actualMotorFrequency
      //   actualMotorFrequency[i] = totalFrequency / frequencyBufferSize;

      //   // Increment frequencyBufferIndex and reset if it has reached the end of the array
      //   if (++frequencyBufferIndex >= frequencyBufferSize) {
      //     frequencyBufferIndex = 0;
      //   }

      //   // Reset the frequencyUpdate flag
      //   frequencyUpdate = false;
      // }

      // Update variables for next loop
      if (currentSample > upperThreshold) {
        previousSampleUnderUpperThreshold[i] = false;
        previousSampleAboveLowerThreshold[i] = true;
      } else if (currentSample < lowerThreshold) {
        previousSampleUnderUpperThreshold[i] = true;
        previousSampleAboveLowerThreshold[i] = false;
      } else {
        previousSampleUnderUpperThreshold[i] = true;
        previousSampleAboveLowerThreshold[i] = true;
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
  signed int p[numberOfHallSensors];
  signed int deltaP[numberOfHallSensors];
  signed int pDash[numberOfHallSensors];

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
    deltaP[i] = 0;
    pDash[i] = 0;
    pinMode(motorPWM[i], OUTPUT);
  }

  // Start the loop
  while (true) {

    // Pause the task until enough time has passed
    vTaskDelayUntil(&xLastWakeTime, xFrequency);

    // Loops through each motor
    for (uint8_t i = 0; i < numberOfHallSensors; i++) {

      // Calculate error terms
      currentError[i] = targetMotorFrequency[i] - instantaniousMotorFrequency[i];
      totalError[i] += currentError[i];
      changeInError[i] = (currentError[i] - previousError[i]) * controlFrequency;

      // If the error is large use controller 1 for
      // if ((currentError[i] > 0.3) || (currentError[i] < 0.3)) {
      //   u[i] = kp1 * currentError[i];
      // }
      // // Else use controller 1 for good static precision
      // else {
      // Calculate u, the PID controller output
      u[i] = (kp * currentError[i]) + (ki * totalError[i]) + (kd * changeInError[i]);
      // }

      // Calculate the PWM value
      // p[i] = frequencyToPWM(targetMotorFrequency[i]);
      deltaP[i] = frequencyToPWM(u[i]);
      pDash[i] = p[i] + deltaP[i];

      // Limit the PWM value
      if (pDash[i] > maxAllowedPWM) {
        pDash[i] = maxAllowedPWM;
      } else if (pDash[i] < 0) {
        pDash[i] = 0;
      }

      if (i == 0) {
        // Serial.print(changeInError[i]);
        // Serial.print(", ");
        // Serial.print(currentError[i]);
        // Serial.print(", ");
        // Serial.print(totalError[i]);
        // Serial.print(", ");
        Serial.print(instantaniousMotorFrequency[i]);
        Serial.print(", ");
        Serial.println(targetMotorFrequency[i]);
      }


      //      Serial.println(pDash[i]);
      // Write the PWM value
      analogWrite(motorPWM[i], pDash[i]);
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
    vTaskDelay(pdMS_TO_TICKS(10));
    // Serial.print(instantaniousMotorFrequency[0]);
    // Serial.print(", ");
    // Serial.println(targetMotorFrequency[0]);
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

    vTaskDelay(pdMS_TO_TICKS(50));
    // Serial.println(actualMotorFrequency[0]);
    //   taskStatusUpdate();
  }
}

//-------------------------------- Setups -----------------------------------------------

void setup() {

  // Setup USB
  Serial.begin(115200);
  delay(1000);

  // Define analogue parameters
  // analogWriteFreq(pwmFrequency);
  // analogWriteResolution(pwmResolution);
  // analogReadResolution(adcResolution);

  // Initialise arrays
  for (uint8_t i = 0; i < numberOfMotors; i++) {
    targetMotorFrequency[i] = 0;
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
  if (DEBUG_ENABLED) {
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
