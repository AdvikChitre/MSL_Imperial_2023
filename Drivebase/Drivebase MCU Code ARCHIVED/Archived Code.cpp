// Returns the standard deviation of an array of type uint16_t
float standardDeviation(uint16_t population[], uint16_t populationSize, uint16_t populationMean) {

  // Define local variables
  uint16_t currentSample;
  int deviation;
  long sampleSum = 0;
  float populationStandardDeviation;

  // Calculate the sum(xi-mew)^2 part of the standard deviation
  for (int i = 0; i < populationSize; i++) {
    currentSample = population[i];
    deviation = currentSample - populationMean;
    sampleSum += deviation * deviation;
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
  float lowerCrossing = mean - (0.5 * standardDeviation);  // Should be 2x standard deviation
  float upperCrossing = mean + (0.5 * standardDeviation);  // Should be 2x standard deviation
  float frequency;


  // Goes through the samples
  for (uint16_t i = 1; i < sampleNumber; i++) {
    currentSample = signal[i];
    Serial.print(lowerCrossing);
    Serial.print(", ");
    Serial.print(upperCrossing);
    Serial.print(", ");
    Serial.println(currentSample);
    vTaskDelay(pdMS_TO_TICKS(1));

    // Check for crossing the mean +- 2 standard devations
    if (((currentSample > upperCrossing) && (previousSample < upperCrossing)) || ((currentSample < upperCrossing) && (previousSample > upperCrossing)) || ((currentSample > lowerCrossing) && (previousSample < lowerCrossing)) || ((currentSample < lowerCrossing) && (previousSample > lowerCrossing))) {
      totalQuarterCycles += 1;
    }

    // Set the current sample to the previous sample and loop
    previousSample = currentSample;
  }

  // Calculate frequency
  frequency = (totalQuarterCycles * samplingFrequency) / ((float)(4 * samples));
  Serial.print(totalQuarterCycles);
  Serial.print(", ");
  Serial.println(frequency);
  return frequency;
}




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
  static uint16_t data[numberOfMotors][samples];    // Non-volatile array to store the sample data
  static float standardDeviations[numberOfMotors];  // Array to store standard deviations

  // Start the loop
  while (true) {

    // Read the samples from the sample queue into a non volatile 2D array so they can be processed without the data changing
    for (uint16_t sampleNumber = 0; sampleNumber < samples; sampleNumber++) {
      for (uint8_t motor = 0; motor < numberOfMotors; motor++) {
        xQueueReceive(sampleQueue, &data[motor][sampleNumber], portMAX_DELAY);
        // if (motor == 2) {
        //  Serial.println(data[motor][sampleNumber]);
        // }
      }
    }

    vTaskDelay(pdMS_TO_TICKS(5000));

    // Once the data has been copied, tell the readHallSensorsTask to resume
    samplesTransferred = true;

    // Loop through each motor and find frequencies
    for (uint8_t motor = 2; motor < numberOfMotors; motor++) {

      // Calculate standard deviation
      standardDeviations[motor] = standardDeviation(data[motor], samples, ((adcRange + 1) / 2) + 20);

      // If standard deviation greater than threshold value then a signal is present
      if (standardDeviations[motor] > thresholdStandardDeviation) {

        // If signal present, work out its frequency
        actualMotorFrequency[motor] = getFrequency(data[motor], samples, ((adcRange + 1) / 2) + 20, samplingFrequency, standardDeviations[motor]);
      }

      // Else it is just noise
      else {
        actualMotorFrequency[motor] = 0;
      }
    }
  }
}













// Task function to calculate the motor frequencies
void getMotorFrequencyTask(void* pvParameters) {

  (void)pvParameters;

  // Create local variables
  static bool previousSampleUnderUpperThreshold[numberOfHallSensors];      // Stores whether the previous sample was under the threshold or not
  static bool previousSampleAboveLowerThreshold[numberOfHallSensors];      // Stores whether the previous sample was above the lower threshold or not
  static uint16_t currentSample;                                           // Stores the current sample
  static unsigned long timePrevious[numberOfHallSensors][4];               // Stores the previous time a peak occured for each hall sensor
  static unsigned long timeCurrent;                                        // The time when the current sample was taken
  static unsigned long timeSinceLastPeak[4];                               // Stores the time since the last peak (float because floating point arithmatic is done with it)
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
        instantaniousMotorFrequency[i] = 250000 / ((float)timeSinceLastPeak[0]);

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

      timeout = timeoutConstant / instantaniousMotorFrequency[i];

      if (i == 0) {
        // Serial.print(timeout / 1024);
        // Serial.print(", ");
        // Serial.print(instantaniousMotorFrequency[i] * 100.0);
        // Serial.print(", ");
        // Serial.print(upperThreshold);
        // Serial.print(", ");
        // Serial.print(lowerThreshold);
        // Serial.print(", ");
        Serial.print(1023);
        Serial.print(", ");
        Serial.print(0);
        Serial.print(", ");
        Serial.print(currentSample);
        Serial.print(", ");
        Serial.println(targetMotorFrequency[i] * 100.0);
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






sampleQueue = xQueueCreate(samples * numberOfMotors, sizeof(uint16_t));  // Queue length is the number of samples, item size is 2 bytes (uint16_t)