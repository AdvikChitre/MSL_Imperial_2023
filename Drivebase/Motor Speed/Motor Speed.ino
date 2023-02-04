/* 
TODO:
- Create rolling average of motor frequencies (weighted?)
- Finish PID control loop
- Add usbHandle()
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

//-------------------------------- Global Variables -------------------------------------

static const uint32_t pwmFrequency = 20000;                  // Sets the PWM frequency (in Hz)
static const uint8_t pwmResolution = 16;                     // Sets the PWM resolution (in bits), default is 16
static const uint16_t pwmRange = pow(2, pwmResolution) - 1;  // Calculates the PWM range based on the resolution
static const uint8_t adcResolution = 10;                     // Sets the ADC resolution
static const uint16_t adcRange = pow(2, adcResolution) - 1;  // Calculates the ADC range


static const uint16_t samples = 64;                                    // Number of samples collected before processing
static const uint16_t samplingFrequency = 1000;                        // Sampling frequency in Hz, default: 1000
static const uint8_t numberOfMotors = 3;                               // Number of motors, default: 3
static const int8_t motors[numberOfMotors] = { HALL1, HALL2, HALL3 };  // Vector to store the pins to sample for each motor
static const float thresholdStandardDeviation = 30;                    // Threshold in standard deviation to distinguish between noise and a signal

static volatile uint16_t sampleNumber = 0;          // Variable to store current sample number
static volatile uint16_t motorSamples[3][samples];  // Vector to store ADC samples from the motor 1 hall effect sensor

static volatile uint16_t controlFrequency = 10;                // Control loop frequenct in Hz, default: 10
static volatile double targetMotorFrequency[3] = { 0, 0, 0 };  // Vector to store the target motor frequencies (in Hz)
static volatile double actualMotorFrequency[3] = { 0, 0, 0 };  // Vector to store the motor freqiencies (in Hz)

static volatile bool samplesTransferred = false;  // Variable true when ADC data has been transferred to the processing task

// Sample queue
static QueueHandle_t sampleQueue;

// Data ready semaphore
//static SemaphoreHandle_t dataReadyBinSem;

static const int8_t leds[3] = { LED_BUILTIN, LED1, LED2 };
static volatile bool ledStatus[3] = { false, false, false };

static const char startCharacter = '!';
static char opcode[3];
static char readwrite;
static uint8_t data_length;

//-------------------------------- Functions --------------------------------------------

// Returns the standard deviation of an array of type uint16_t
float standardDeviation(uint16_t population[], uint16_t populationSize, uint16_t populationMean) {

  // Define local variables
  uint16_t currentSample;
  uint32_t sampleSum = 0;
  double populationStandardDeviation;

  // Calculate the sum(xi-mew)^2 part of the standard deviation
  for (int i = 0; i < populationSize; i++) {
    currentSample = population[i];
    sampleSum += (currentSample - populationMean) * (currentSample - populationMean);
  }

  // Calculate the final standard deviation
  populationStandardDeviation = sqrt(sampleSum / populationSize);

  // Return the standard deviation
  return populationStandardDeviation;
}

// Gets the frequency of a sinusoidal wave input as an array using its sampling frequency, average value and standard deviation
float getFrequency(uint16_t signal[], uint16_t sampleNumber, uint16_t mean, uint16_t samplingFrequency, float standardDeviation) {

  // Define local variables
  uint16_t previousSample = signal[0];
  uint16_t currentSample;
  uint16_t totalQuarterCycles = 0;
  float lowerCrossing = mean - (2 * standardDeviation);
  float upperCrossing = mean + (2 * standardDeviation);
  float frequency;

  // Goes through the samples
  for (uint16_t i = 1; i < sampleNumber; i++) {
    currentSample = signal[i];

    // Check for crossing the mean +- 2 standard devations
    if (((currentSample > upperCrossing) && (previousSample < upperCrossing)) || ((currentSample < upperCrossing) && (previousSample > upperCrossing)) || ((currentSample > lowerCrossing) && (previousSample < lowerCrossing)) || ((currentSample < lowerCrossing) && (previousSample > lowerCrossing))) {
      totalQuarterCycles += 1;
    }

    // Set the current sample to the previous sample and loop
    previousSample = currentSample;
  }

  // Calculate frequency
  frequency = (totalQuarterCycles * samplingFrequency) / (4 * sampleNumber);

  return frequency;
}

// Function to print the current status of each task, used for debugging
std::map<eTaskState, const char*> eTaskStateName{ { eReady, "Ready" }, { eRunning, "Running" }, { eBlocked, "Blocked" }, { eSuspended, "Suspended" }, { eDeleted, "Deleted" } };
void ps() {
  int tasks = uxTaskGetNumberOfTasks();
  TaskStatus_t* pxTaskStatusArray = new TaskStatus_t[tasks];
  unsigned long runtime;
  tasks = uxTaskGetSystemState(pxTaskStatusArray, tasks, &runtime);
  Serial.printf("# Tasks: %d\n", tasks);
  Serial.println("ID, NAME, STATE, PRIO, CYCLES");
  for (int i = 0; i < tasks; i++) {
    Serial.printf("%d: %-16s %-10s %d %lu\n", i, pxTaskStatusArray[i].pcTaskName, eTaskStateName[pxTaskStatusArray[i].eCurrentState], (int)pxTaskStatusArray[i].uxCurrentPriority, pxTaskStatusArray[i].ulRunTimeCounter);
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
uint16_t led(uint8_t status, uint8_t led = 0) {

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
void handle_serial() {

  // Reads the opcode, whether it is read or write, and the length of the data
  waitForSerialData((uint8_t)5);
  opcode[0] = Serial.read();
  opcode[1] = Serial.read();
  opcode[2] = Serial.read();
  readwrite = Serial.read();
  data_length = (uint8_t)Serial.read();

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
  Serial.print(data);
  // Print successfully read code
  Serial.print(220);

  // Decoding the instruction, and calling the appropriate function

  // LED command
  if ((opcode[0] == 'L') && (opcode[1] == 'E') && (opcode[2] == 'D')) {
    Serial.print(led(atoi(data)));
  }
}

//-------------------------------- Task Functions ----------------------------------------

// Task function to sample hall sensors and add the samples to an array
void readHallSensorsTask(void* pvParameters) {

  (void)pvParameters;

  // Set input pins
  pinMode(HALL1, INPUT);
  pinMode(HALL2, INPUT);
  pinMode(HALL3, INPUT);

  // Create local variables
  uint16_t currentSample;

  // Setup timer so this task executes at the frequency specified in samplingFrequency
  const TickType_t xFrequency = configTICK_RATE_HZ / samplingFrequency;
  TickType_t xLastWakeTime = xTaskGetTickCount();

  // Start the loop
  while (true) {

    // Pause the task until enough time has passed
    vTaskDelayUntil(&xLastWakeTime, xFrequency);

    // Sample each hall effect sensor and store in motorSamples
    for (uint8_t motor = 0; motor < numberOfMotors; motor++) {
      //      motorSamples[motor][sampleNumber] = analogRead(motors[motor]);
      currentSample = analogRead(motors[motor]);
      xQueueSend(sampleQueue, (void*)&currentSample, (TickType_t)0);
    }

    // Increment the sample number, and if the array is full then tell the processing function the samples are ready
    if (++sampleNumber > samples - 1) {

      // Wait until the samples have been read into the processing function
      while (!samplesTransferred) {
        vTaskDelay(pdMS_TO_TICKS(1));
      }

      // Resest variables before the next loop
      sampleNumber = 0;
      samplesTransferred = false;
    }
  }
}

// Task function to calculate the motor frequencies
void getMotorFrequencyTask(void* pvParameters) {

  (void)pvParameters;

  // Create local variables
  uint16_t data[numberOfMotors][samples];    // Non-volatile array to store the sample data
  float standardDeviations[numberOfMotors];  // Array to store standard deviations

  // Start the loop
  while (true) {
    Serial.print(actualMotorFrequency[0]);

    // Read the samples from the sample queue into a non volatile 2D array so they can be processed without the data changing
    for (uint8_t motor = 0; motor < numberOfMotors; motor++) {
      for (uint16_t sampleNumber = 0; sampleNumber < samples; sampleNumber++) {
        xQueueReceive(sampleQueue, (void*)&data[motor][sampleNumber], portMAX_DELAY);
      }
    }

    // Once the data has been copied, tell the readHallSensorsTask to resume
    samplesTransferred = true;

    // Loop through each motor and find frequencies
    for (uint8_t motor = 0; motor < numberOfMotors; motor++) {

      // Calculate standard deviation
      standardDeviations[motor] = standardDeviation(data[motor], samples, (pwmRange + 1) / 2);

      // If standard deviation greater than threshold value then a signal is present
      if (standardDeviations[motor] > thresholdStandardDeviation) {

        // If signal present, work out its frequency
        actualMotorFrequency[motor] = getFrequency(data[motor], samples, (adcRange + 1) / 2, samplingFrequency, standardDeviations[motor]);
      }

      // Else it is just noise
      else {
        actualMotorFrequency[motor] = 0;
      }
    }
  }
}

// Task function to run the pid control loop
void pidControllerTask(void* pvParameters) {

  (void)pvParameters;

  // Create local variables
  float previousError[numberOfMotors] = { 0, 0, 0 };
  float currentError[numberOfMotors] = { 0, 0, 0 };
  float totalError[numberOfMotors] = { 0, 0, 0 };
  float changeInError[numberOfMotors] = { 0, 0, 0 };

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


void usbTask(void* pvParameters) {

  (void)pvParameters;

  // Create local variables
  static char serialData;

  // Main loop
  while (true) {

    // Wait for any serial data
    waitForSerialData();

    // If there is data, read a character
    serialData = Serial.read();

    // If the first character is the start of a message then handle it
    if (serialData == startCharacter) {
      handle_serial();
    }
  }
}

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

//-------------------------------- Setups -----------------------------------------------

void setup() {

  // Setup USB
  Serial.begin(115200);

  // Define analogue parameters
  analogWriteFreq(pwmFrequency);
  analogWriteResolution(pwmResolution);
  analogReadResolution(adcResolution);

  // Create task handles
  // TaskHandle_t readHallSensorsHandle = nullptr;
  // TaskHandle_t getMotorFrequencyHandle = nullptr;
  // TaskHandle_t pidControllerHandle = nullptr;
  TaskHandle_t usbHandle = nullptr;
  TaskHandle_t updateLedsHandle = nullptr;

  // Create tasks
  // xTaskCreate(
  //   readHallSensorsTask,     /* Function that implements the task */
  //   "HALL_SAMP",           /* Text name for the task */
  //   20000,                    /* Stack size in words, not bytes */
  //   nullptr,                 /* Parameter passed into the task */
  //   1,                       /* Task priority */
  //   &readHallSensorsHandle); /* Pointer to store the task handle */
  // xTaskCreate(
  //   getMotorFrequencyTask,     /* Function that implements the task */
  //   "GET_FREQ",                /* Text name for the task */
  //   100000,                      /* Stack size in words, not bytes */
  //   nullptr,                   /* Parameter passed into the task */
  //   1,                         /* Task priority */
  //   &getMotorFrequencyHandle); /* Pointer to store the task handle */
  // xTaskCreate(
  //   pidControllerTask,     /* Function that implements the task */
  //   "PID_CTRL",            /* Text name for the task */
  //   1000,                  /* Stack size in words, not bytes */
  //   nullptr,               /* Parameter passed into the task */
  //   1,                     /* Task priority */
  //   &pidControllerHandle); /* Pointer to store the task handle */
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
    128,                /* Stack size in words, not bytes */
    nullptr,            /* Parameter passed into the task */
    1,                  /* Task priority */
    &updateLedsHandle); /* Pointer to store the task handle */

  // Set task affinities (0x00 -> no cores, 0x01 -> C0, 0x02 -> C1, 0x03 -> C0 and C1)
  // vTaskCoreAffinitySet(readHallSensorsHandle, (UBaseType_t)0x03);
  // vTaskCoreAffinitySet(getMotorFrequencyHandle, (UBaseType_t)0x03);
  // vTaskCoreAffinitySet(pidControllerHandle, (UBaseType_t)0x03);
  // vTaskCoreAffinitySet(usbHandle, (UBaseType_t)0x01);
  // vTaskCoreAffinitySet(updateLedsHandle, (UBaseType_t)0x03);

  // Create queues
  sampleQueue = xQueueCreate(samples * numberOfMotors, sizeof(uint16_t));  // Queue length is the number of samples, item size is 2 bytes (uint16_t)

  // Create semaphores
  //  dataReadyBinSem = xSemaphoreCreateBinary();

  // Delete "setup and loop" task
//  vTaskDelete(NULL);
}

void setup1() {}

//-------------------------------- Loop ------------------------------------------------

void loop() {
  vTaskDelay(pdMS_TO_TICKS(500));
//  ps();
  // Should never get to this point
}

void loop1() {
  vTaskDelay(pdMS_TO_TICKS(500));
//  ps();
  // Should never get to this point
}