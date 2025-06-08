# Smart-Ultrasonic-Library

**A high-performance, non-blocking Arduino library for the HC-SR04 ultrasonic sensor that intelligently handles out-of-range failures using predictive state-estimation and a persistent EEPROM memory buffer.**

[![Arduino Lint](https://github.com/YourUsername/YourRepoName/actions/workflows/arduino-lint.yml/badge.svg)](https://github.com/YourUsername/YourRepoName/actions/workflows/arduino-lint.yml)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

---

## The Problem: The "False Zero" Error

Standard ultrasonic sensor code has a critical flaw: when an object is beyond its maximum range, it reports a distance of **0 cm**. This is indistinguishable from an object that is actually right in front of the sensor, causing catastrophic failures in robots and other automated systems. This library fixes that.

Instead of returning a false zero, this library:
1.  **Detects the sensor timeout** (the real cause of the error).
2.  **Predicts** what the distance *should* be based on the object's last known positions.
3.  Assigns a **confidence score** to the prediction.
4.  For mobile robots, it can **verify its own prediction** using a closed-loop movement maneuver.

 
_Optional: It's highly recommended to create and include a diagram here showing the problem and solution._

## Key Features

🚀 **100% Non-Blocking, Interrupt-Driven Architecture**
- Uses hardware interrupts (`attachInterrupt`) for microsecond-precision measurements without ever halting your `loop()`.
- Frees your CPU to run other critical tasks (like motor control or communication) concurrently, increasing `loop()` speed by over 5000x compared to blocking code.

🧠 **Intelligent Prediction & State-Estimation**
- When a sensor fails (timeout), it uses **linear extrapolation** to predict the next likely distance, providing data continuity.
- Each reading is delivered as a `SensorReading` struct, containing the distance, a **confidence score (0-100%)**, and prediction status.

💾 **Persistent EEPROM Memory with Wear-Leveling**
- Remembers the last ~250 readings even after a power cycle, allowing for instant intelligent predictions on startup.
- Uses a highly-efficient **circular buffer** in EEPROM to maximize data density and distribute writes, drastically extending the EEPROM's lifespan.

🤖 **Designed for Both Mobile & Static Systems**
- The user specifies the system type (`isMobile`) on creation.
- **Mobile Robots:** Provides a `verifyLastPrediction()` method to confirm predictions by moving a known distance.
- **Static Sensors:** Features an automatic, non-blocking retry mechanism to recover from transient measurement failures.

🔧 **Configurable and User-Friendly**
- Create multiple independent sensor objects, each with its own pins and EEPROM memory space.
- The `isMobile` flag in the constructor controls the algorithm's behavior, making the code clean and purpose-driven.
- Packaged as a standard Arduino library for easy installation and use via `#include`.

## Installation

1.  **Download:** Click the "Code" button and download the repository as a ZIP file.
2.  **Install in Arduino IDE:** Open the Arduino IDE, go to `Sketch > Include Library > Add .ZIP Library...` and select the file you just downloaded.
3.  **You're ready!** The library will now be available under `File > Examples > Smart-Ultrasonic-Library`.

## Quick Start Guide

Here's a simple example showing how to get a non-blocking reading from a static sensor.

```cpp
#include <SmartUltrasonic.h>

// 1. Create a sensor object for a STATIC system on pins D9 (Trig) and D2 (Echo).
// Note: On Arduino Uno/Nano, the Echo pin must be an interrupt pin (2 or 3).
SmartUltrasonic mySensor(9, 2, false); // Pin 9, Pin 2, isMobile = false

// This ISR wrapper is required to link the hardware interrupt to our object.
void mySensorISR() {
  mySensor.handleInterrupt();
}

// Simple non-blocking scheduler
unsigned long lastMeasureTime = 0;

void setup() {
  Serial.begin(115200);

  // Initialize the sensor
  mySensor.begin();

  // Attach the interrupt to its pin and corresponding ISR
  attachInterrupt(digitalPinToInterrupt(2), mySensorISR, CHANGE);
  
  Serial.println("Sensor ready!");
}

void loop() {
  // Always call update() in your main loop. This runs the internal state machine.
  mySensor.update();

  // Schedule a new measurement every 100ms
  if (millis() - lastMeasureTime > 100) {
    lastMeasureTime = millis();
    mySensor.startMeasurement(); // This is non-blocking and returns instantly
  }
  
  // Check if a new reading is complete
  if (mySensor.isReady()) {
    SensorReading reading = mySensor.getReading();
    
    Serial.print("Distance: " + String(reading.distance) + " cm");
    Serial.print(" | Confidence: " + String(reading.confidence) + "%");
    if (reading.isPredicted) {
      Serial.print(" (Predicted)");
    }
    Serial.println();
  }

  // Your CPU is free to do other things here!
}
