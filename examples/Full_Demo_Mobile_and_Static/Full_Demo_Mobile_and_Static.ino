#include <SmartUltrasonic.h>

// === INTERRUPT SETUP - Essential for non-blocking code on Arduino Uno/Nano ===
// NOTE: Only pins 2 and 3 can be used for attachInterrupt on Uno/Nano.
// Create pointers to our objects for the ISRs to use.
SmartUltrasonic* frontSensorPtr = nullptr;
SmartUltrasonic* parkingSensorPtr = nullptr;

// Create sensor objects. The user decides the type here.
// new SmartUltrasonic(trigPin, echoPin, isMobileSystem, eepromAddress)
SmartUltrasonic frontSensor(9, 2, true, 0);     // This is a MOBILE sensor
SmartUltrasonic parkingSensor(11, 3, false, 300); // This is a STATIC sensor

// ISR wrapper functions
void frontSensorISR() { if (frontSensorPtr) frontSensorPtr->handleInterrupt(); }
void parkingSensorISR() { if (parkingSensorPtr) parkingSensorPtr->handleInterrupt(); }


// Placeholder for robot control (should also be non-blocking in a real robot)
void moveRobotBackwards(float cm) {
  Serial.print("ROBOT ACTION: Moving backwards ");
  Serial.print(cm);
  Serial.println(" cm. (Blocking for demonstration)");
  delay(1000); // In a real system, you'd use a state machine for movement
}

// Simple non-blocking scheduler
unsigned long lastMeasureTime = 0;
const unsigned int MEASURE_INTERVAL = 150; // ms

void setup() {
  Serial.begin(115200);
  while (!Serial);

  Serial.println("Initializing User-Configurable Ultrasonic Sensors...");
  
  frontSensor.begin();
  parkingSensor.begin();

  // Link pointers to the objects for ISRs
  frontSensorPtr = &frontSensor;
  parkingSensorPtr = &parkingSensor;

  // Attach interrupts to the pins specified in the object creation
  attachInterrupt(digitalPinToInterrupt(2), frontSensorISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(3), parkingSensorISR, CHANGE);
  
  Serial.println("Setup complete. Running loop...");
  Serial.println("----------------------------------------------");
}

void loop() {
  // 1. ALWAYS update the sensor objects. This runs their internal state machines.
  frontSensor.update();
  parkingSensor.update();

  // 2. Schedule new measurements at a regular interval.
  if (millis() - lastMeasureTime > MEASURE_INTERVAL) {
    lastMeasureTime = millis();
    frontSensor.startMeasurement();   // Both start a non-blocking measurement
    parkingSensor.startMeasurement();
  }
  
  // 3. Check each sensor if it has a completed reading.
  if (frontSensor.isReady()) {
    processReading(frontSensor, "Front Sensor");
  }

  if (parkingSensor.isReady()) {
    processReading(parkingSensor, "Parking Sensor");
  }

  // 4. Do other stuff here! Your loop is never blocked.
  // For example: myRobot.updateMotors(); myRobot.checkBumpers();
}


// --- Generic function to handle the output from any sensor object ---
void processReading(SmartUltrasonic &sensor, String sensorName) {
  SensorReading reading = sensor.getReading();
  
  Serial.print("[" + sensorName + "] ");

  if (reading.isPredicted) {
    Serial.print("Prediction! Dist: " + String(reading.distance, 1) + " cm, Conf: " + String(reading.confidence) + "%. ");

    // ==========================================================
    // === HERE'S THE CORE LOGIC: CHECK THE SENSOR'S TYPE ===
    // ==========================================================
    if (sensor.isMobile()) {
      Serial.println("Action: Verifying via movement.");
      
      // The application code, NOT the library, decides to move the robot.
      moveRobotBackwards(DEFAULT_MOVEMENT_VERIFY_DIST);
      
      // Take a new reading to verify. This is a simplified, blocking example for clarity.
      sensor.startMeasurement();
      while (!sensor.isReady()) { sensor.update(); } // Wait for the new reading
      
      SensorReading verificationReading = sensor.getReading();
      Serial.print("   -> Verification reading: " + String(verificationReading.distance, 1) + " cm. Result: ");
      if (sensor.verifyLastPrediction(verificationReading.distance)) {
        Serial.println("VERIFIED.");
      } else {
        Serial.println("FAILED.");
      }
      
    } else { // It's a static sensor
      Serial.println("Action: Retrying automatically after delay...");
      // The retry logic is now handled internally by the library's state machine.
      // The user code doesn't need to do anything special here except wait for the next reading.
    }
    Serial.println("----------------------------------------------");

  } else { // It was a direct, valid reading
    Serial.println("Direct read: " + String(reading.distance, 1) + " cm");
  }
}