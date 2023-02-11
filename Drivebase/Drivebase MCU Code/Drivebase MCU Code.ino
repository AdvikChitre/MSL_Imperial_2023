/* 
TODO:
- Finish PID control loop
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

#define DEBUG_ENABLED false

//-------------------------------- Global Variables -------------------------------------

static const uint32_t pwmFrequency = 20000;                  // Sets the PWM frequency (in Hz)
static const uint8_t pwmResolution = 8;                      // Sets the PWM resolution (in bits), default is 8
static const uint16_t pwmRange = pow(2, pwmResolution) - 1;  // Calculates the PWM range based on the resolution
static const uint8_t adcResolution = 10;                     // Sets the ADC resolution
static const uint16_t adcRange = pow(2, adcResolution) - 1;  // Calculates the ADC range

static const uint16_t samplingFrequency = 200;                         // Sampling frequency in Hz, default: 1000
static const uint8_t numberOfMotors = 3;                               // Number of motors, default: 3
static const int8_t motors[numberOfMotors] = { HALL1, HALL2, HALL3 };  // Vector to store the pins to sample for each motor
static const uint16_t threshold = 560;                                 // Threshold value that is compared to the analogRead() value to count the time interval between oscillations
static const long timeout = 1000000;                                   // Time in milliseconds after which the motor speed is considerd to be 0
static const uint8_t frequencyBufferSize = 10;                         // Number of data points that the rolling frequency average is calculated from

static volatile uint16_t controlFrequency = 10;                                   // Control loop frequenct in Hz, default: 10
static volatile float targetMotorFrequency[numberOfMotors] = { 0, 0, 0 };         // Array to store the target motor frequencies (in Hz)
static volatile float instantaniousMotorFrequency[numberOfMotors] = { 0, 0, 0 };  // Array to store the instantanious motor frequencies (in Hz)
static volatile float actualMotorFrequency[numberOfMotors] = { 0, 0, 0 };         // Array to store the rolling averages of motor frequencies (in Hz)

static const uint8_t wheelDiameter = 100;                                                          // Wheel diameter in mm, used to convert speeds to frequencies
static const float maxMotorFrequency = 13.02;                                                      // Max motor frequency in Hz, default : 13.02
static const uint8_t maxAllowedSpeedPercent = 30;                                                  // Max allowed speed given as a percentage of actual max speed
static const float maxAllowedMotorFrequency = maxMotorFrequency * (maxAllowedSpeedPercent / 100);  // Max allowed motor frequency in Hz
static const uint16_t maxAllowedPWM = adcRange * (maxAllowedSpeedPercent / 100);                   // Max allowed value in analogWrite()

static const int8_t leds[3] = { LED_BUILTIN, LED1, LED2 };
static volatile bool ledStatus[3] = { false, false, false };

static const char startCharacter = '!';

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

// Function to print the current status of each task, used for debugging
std::map<eTaskState, const char*> eTaskStateName{ { eReady, "Ready" }, { eRunning, "Running" }, { eBlocked, "Blocked" }, { eSuspended, "Suspended" }, { eDeleted, "Deleted" } };
void ps() {
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
uint16_t led(uint8_t led = 0, uint8_t status = 0) {

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
    Serial.print(led((data[0] - 48), (data[1] - 48)));
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

  // Set input pins
  pinMode(HALL1, INPUT);
  pinMode(HALL2, INPUT);
  pinMode(HALL3, INPUT);

  // Create local variables
  static bool previousSampleUnderThreshold[numberOfMotors];           //
  static uint16_t currentSample;                                      // Stores the current sample
  static unsigned long timePrevious[numberOfMotors];                  // Stores the previous time a peak occured for each hall sensor
  static unsigned long timeCurrent;                                   // The time when the current sample was taken
  static float timeSinceLastPeak;                                     // Stores the time since the last peak (float because floating point arithmatic is done with it)
  static float frequencyBuffer[numberOfMotors][frequencyBufferSize];  // Array to store the previous samples used to calculate the rolling average
  static uint8_t frequencyBufferIndex = 0;                            // Index in the frequencyBuffer that will be read and then written next
  static float totalFrequency;                                        // Stores the sum of frequencies used to calculate the rolling average
  static bool frequencyUpdate = false;                                // true when an update to a frequency has occured

  // Setup timer so this task executes at the frequency specified in samplingFrequency
  const TickType_t xFrequency = configTICK_RATE_HZ / samplingFrequency;
  TickType_t xLastWakeTime = xTaskGetTickCount();

  // Inititalise arrays
  for (uint8_t i = 0; i < numberOfMotors; i++) {

    // Initialise 1D arrays
    previousSampleUnderThreshold[i] = true;
    timePrevious[i] = 0;

    // Initialise 2D arrays
    for (uint8_t j = 0; j < frequencyBufferSize; j++) {
      frequencyBuffer[i][j] = 0;
    }
  }

  // Start the loop
  while (true) {

    // Pause the task until enough time has passed
    vTaskDelayUntil(&xLastWakeTime, xFrequency);

    // Loop through each motor
    for (uint8_t i = 0; i < numberOfMotors; i++) {

      // Sample the ADC and calculate the time since the last peak (this is a critical section since we don't want an interruption between the sample and recording the time it was taken)
      taskENTER_CRITICAL();
      currentSample = analogRead(motors[i]);
      timeCurrent = micros();
      taskEXIT_CRITICAL();
      timeSinceLastPeak = timeCurrent - timePrevious[i];

      // Check if the current sample is above the threshold and the previous sample was below, if true there is a peak
      if ((currentSample > threshold) && (previousSampleUnderThreshold[i])) {

        // Work out the instantanious frequency based on the time since last peak
        instantaniousMotorFrequency[i] = 250000 / (timeSinceLastPeak);

        // Set the frequencyUpdate flag
        frequencyUpdate = true;

        // Update variables for next loop
        timePrevious[i] = timeCurrent;
      }

      // If no peak after timeout time, or previous frequency 0 then frequency is considered to be 0
      else if ((timeSinceLastPeak > timeout) || (instantaniousMotorFrequency[i] == 0)) {

        // Set the frequency to 0
        instantaniousMotorFrequency[i] = 0;

        // Set the frequencyUpdate flag
        frequencyUpdate = true;
      }

      // If an update to the frequency occured, recalculate the rolling average
      if (frequencyUpdate) {

        // Update the total frequency and frequency buffer
        totalFrequency = totalFrequency - frequencyBuffer[i][frequencyBufferIndex];
        frequencyBuffer[i][frequencyBufferIndex] = instantaniousMotorFrequency[i];
        totalFrequency = totalFrequency + instantaniousMotorFrequency[i];

        // Calculate the average and update actualMotorFrequency
        actualMotorFrequency[i] = totalFrequency / frequencyBufferSize;

        // Increment frequencyBufferIndex and reset if it has reached the end of the array
        if (frequencyBufferIndex++ >= frequencyBufferSize) {
          frequencyBufferIndex = 0;
        }

        // Reset the frequencyUpdate flag
        frequencyUpdate = false;
      }

      // Update variables for next loop
      if (currentSample > threshold) {
        previousSampleUnderThreshold[i] = false;
      } else {
        previousSampleUnderThreshold[i] = true;
      }
    }
  }
}

// Task function to run the pid control loop
void pidControllerTask(void* pvParameters) {

  (void)pvParameters;

  // Create local variables
  static float previousError[numberOfMotors] = { 0, 0, 0 };
  static float currentError[numberOfMotors] = { 0, 0, 0 };
  static float totalError[numberOfMotors] = { 0, 0, 0 };
  static float changeInError[numberOfMotors] = { 0, 0, 0 };

  // Make the task execute at a specified frequency
  const TickType_t xFrequency = configTICK_RATE_HZ / controlFrequency;
  TickType_t xLastWakeTime = xTaskGetTickCount();

  // Start the loop
  while (true) {

    // Pause the task until enough time has passed
    vTaskDelayUntil(&xLastWakeTime, xFrequency);

    // Loops through each motor
    for (uint8_t motor = 0; motor < numberOfMotors; motor++) {

      currentError[motor] = targetMotorFrequency[motor] - actualMotorFrequency[motor];
      totalError[motor] += currentError[motor];
      changeInError[motor] = (currentError[motor] - previousError[motor]) * controlFrequency;
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

  // Set pins
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);

  // Main loop
  while (true) {

    // Check status of each LED and update accordingly
    for (uint8_t i = 0; i < sizeof(leds); i++) {
      if (ledStatus[i]) {
        digitalWrite(leds[i], HIGH);
      } else {
        digitalWrite(leds[i], LOW);
      }
    }

    // Delay
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

// Task to print debug messages
void debugTask(void* pvParameters) {

  (void)pvParameters;

  while (true) {
    vTaskDelay(pdMS_TO_TICKS(500));
    ps();
  }
}

//-------------------------------- Setups -----------------------------------------------

void setup() {

  // Setup USB
  Serial.begin(115200);
  delay(1000);

  pinMode(LED_BUILTIN, OUTPUT);

  // Define analogue parameters
  // analogWriteFreq(pwmFrequency);
  // analogWriteResolution(pwmResolution);
  // analogReadResolution(adcResolution);

  // Create tasks
  xTaskCreate(
    getMotorFrequencyTask,     /* Function that implements the task */
    "GET_FREQ",                /* Text name for the task */
    1000,                      /* Stack size in words, not bytes */
    nullptr,                   /* Parameter passed into the task */
    1,                         /* Task priority */
    &getMotorFrequencyHandle); /* Pointer to store the task handle */

  xTaskCreate(
    pidControllerTask,     /* Function that implements the task */
    "PID_CTRL",            /* Text name for the task */
    1000,                  /* Stack size in words, not bytes */
    nullptr,               /* Parameter passed into the task */
    1,                     /* Task priority */
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
    10000,              /* Stack size in words, not bytes */
    nullptr,            /* Parameter passed into the task */
    1,                  /* Task priority */
    &updateLedsHandle); /* Pointer to store the task handle */

  // Set task affinities (0x00 -> no cores, 0x01 -> C0, 0x02 -> C1, 0x03 -> C0 and C1)
  vTaskCoreAffinitySet(getMotorFrequencyHandle, (UBaseType_t)0x03);
  vTaskCoreAffinitySet(pidControllerHandle, (UBaseType_t)0x03);
  vTaskCoreAffinitySet(usbHandle, (UBaseType_t)0x03);
  vTaskCoreAffinitySet(updateLedsHandle, (UBaseType_t)0x03);

  // Enable the debug task if configured to do so
  if (DEBUG_ENABLED) {
    xTaskCreate(
      debugTask,     /* Function that implements the task */
      "DEBUG",       /* Text name for the task */
      1000,          /* Stack size in words, not bytes */
      nullptr,       /* Parameter passed into the task */
      1,             /* Task priority */
      &debugHandle); /* Pointer to store the task handle */
    vTaskCoreAffinitySet(debugHandle, (UBaseType_t)0x01);
  }



  // Delete "setup and loop" task
  vTaskDelete(NULL);
}

void setup1() {
  // Delete "setup1 and loop1" task
  vTaskDelete(NULL);
}

//-------------------------------- Loop ------------------------------------------------

void loop() {
  // Should never get to this point

  // Serial.println(actualMotorFrequency[2]);
  // vTaskDelay(pdMS_TO_TICKS(1000));
}

void loop1() {
  // Should never get to this point
}
