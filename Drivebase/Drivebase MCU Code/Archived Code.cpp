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









sampleQueue = xQueueCreate(samples * numberOfMotors, sizeof(uint16_t));  // Queue length is the number of samples, item size is 2 bytes (uint16_t)