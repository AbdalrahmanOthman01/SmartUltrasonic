#ifndef SMART_ULTRASONIC_H
#define SMART_ULTRASONIC_H

#include "Arduino.h"
#include <EEPROM.h>

// Configuration Defaults (can be overridden by the user in their sketch if needed)
#define DEFAULT_SENSOR_TIMEOUT_MS 35
#define DEFAULT_STATIC_RETRY_MS 500
#define DEFAULT_EEPROM_BUFFER_SIZE 250
#define DEFAULT_RAM_BUFFER_SIZE 8
#define DEFAULT_EEPROM_WRITE_INTERVAL 10
#define DEFAULT_EEPROM_SCALE_FACTOR 2.0f
#define DEFAULT_MOVEMENT_VERIFY_DIST 10.0f
#define DEFAULT_VERIFY_TOLERANCE 3.0f

struct SensorReading {
    float distance;
    int confidence;
    bool isPredicted;
    bool isVerified;
};

enum class SensorState { IDLE, TRIGGERED, MEASURING, DONE, AWAITING_RETRY };

class SmartUltrasonic {
public:
    // **MODIFIED CONSTRUCTOR**: User specifies if the system is mobile.
    SmartUltrasonic(byte trigPin, byte echoPin, bool isMobileSystem, int eepromAddr = 0);

    void begin();
    void update(); // MUST be called frequently in the main loop()

    bool isReady() const { return _state == SensorState::DONE; }
    void startMeasurement();

    SensorReading getReading();
    
    // Public getter to check the sensor's configured type
    bool isMobile() const;

    // Verification method, used by mobile systems
    bool verifyLastPrediction(float newMeasurement);

private:
    byte _trigPin, _echoPin;
    bool _isMobile; // Stores the system type
    int _eepromStartAddr;

    // Internal State Machine
    volatile SensorState _state = SensorState::IDLE;
    volatile unsigned long _startTime = 0;
    volatile unsigned long _duration = 0;
    unsigned long _lastEventTime = 0;

    float _lastPrediction = 0.0f;
    float _ramBuffer[DEFAULT_RAM_BUFFER_SIZE];
    int _ramBufferIndex = 0;
    bool _ramBufferFull = false;

    int _eepromHeadPtr = 0;
    int _writeCounter = 0;

    SensorReading predictValue();
    void updateBuffers(float validReading);
    void loadFromEEPROM();
    void writeToEEPROM(float value);
    float calculateStdDev();
    int calculateConfidence(float stdDev);

public:
    void handleInterrupt(); // Must be public for the ISR wrapper
};

#endif // SMART_ULTRASONIC_H