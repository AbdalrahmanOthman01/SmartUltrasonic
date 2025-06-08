#include "SmartUltrasonic.h"

// Constructor now saves the system type
SmartUltrasonic::SmartUltrasonic(byte trigPin, byte echoPin, bool isMobileSystem, int eepromAddr)
{
    _trigPin = trigPin;
    _echoPin = echoPin;
    _isMobile = isMobileSystem;
    _eepromStartAddr = eepromAddr;
}

void SmartUltrasonic::begin()
{
    pinMode(_trigPin, OUTPUT);
    pinMode(_echoPin, INPUT);
    loadFromEEPROM();
}

void SmartUltrasonic::startMeasurement()
{
    if (_state != SensorState::IDLE && _state != SensorState::DONE)
        return;

    _lastEventTime = millis();
    _state = SensorState::TRIGGERED;
    digitalWrite(_trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(_trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(_trigPin, LOW);
}

// update() now handles the non-blocking retry for static systems
void SmartUltrasonic::update()
{
    if ((_state == SensorState::TRIGGERED || _state == SensorState::MEASURING) && (millis() - _lastEventTime > DEFAULT_SENSOR_TIMEOUT_MS))
    {
        _duration = 0;
        _state = SensorState::DONE;
    }

    if (_state == SensorState::AWAITING_RETRY && (millis() - _lastEventTime > DEFAULT_STATIC_RETRY_MS))
    {
        startMeasurement();
    }
}

// getReading() now triggers the AWAITING_RETRY state for static systems
SensorReading SmartUltrasonic::getReading()
{
    if (_state != SensorState::DONE)
    {
        return {0, 0, false, false};
    }

    noInterrupts();
    unsigned long duration = _duration;
    interrupts();

    float distance = (float)duration * 0.0343 / 2.0;

    if (duration == 0 || distance > 450 || distance < 2)
    {
        SensorReading prediction = predictValue();
        if (!_isMobile)
        {
            // If static, we automatically trigger a retry state. The user just waits for the next reading.
            _state = SensorState::AWAITING_RETRY;
            _lastEventTime = millis();
        }
        else
        {
            _state = SensorState::IDLE;
        }
        return prediction;
    }
    else
    {
        updateBuffers(distance);
        _state = SensorState::IDLE;
        return {distance, 100, false, false};
    }
}

// The new public getter
bool SmartUltrasonic::isMobile() const
{
    return _isMobile;
}

// Interrupt handler changes the state, which is read later by the non-blocking main loop
void SmartUltrasonic::handleInterrupt()
{
    if (digitalRead(_echoPin))
    {
        if (_state == SensorState::TRIGGERED)
        {
            _startTime = micros();
            _state = SensorState::MEASURING;
        }
    }
    else
    {
        if (_state == SensorState::MEASURING)
        {
            _duration = micros() - _startTime;
            _state = SensorState::DONE;
        }
    }
}

// ... all other private helper functions (predictValue, updateBuffers, etc.) remain unchanged ...
// They are exactly the same as the previous response. For brevity, I'm omitting the
// functions that are 100% identical. The complete .cpp file would include them.

// Stubs for the unchanged helper functions:
bool SmartUltrasonic::verifyLastPrediction(float newMeasurement) { /* ... same as before ... */ return abs(newMeasurement - (_lastPrediction + DEFAULT_MOVEMENT_VERIFY_DIST)) < DEFAULT_VERIFY_TOLERANCE; }
SensorReading SmartUltrasonic::predictValue()
{ /* ... same as before ... */
    if (!_ramBufferFull && _ramBufferIndex < 2)
        return {450, 0, true, false};
    float d_c = _ramBuffer[(_ramBufferIndex - 1 + DEFAULT_RAM_BUFFER_SIZE) % DEFAULT_RAM_BUFFER_SIZE], d_p = _ramBuffer[(_ramBufferIndex - 2 + DEFAULT_RAM_BUFFER_SIZE) % DEFAULT_RAM_BUFFER_SIZE];
    _lastPrediction = constrain(d_c + (d_c - d_p), 2, 450);
    int conf = calculateConfidence(calculateStdDev());
    return {_lastPrediction, conf, true, false};
}
void SmartUltrasonic::updateBuffers(float validReading)
{ /* ... same as before ... */
    _ramBuffer[_ramBufferIndex] = validReading;
    _ramBufferIndex = (_ramBufferIndex + 1) % DEFAULT_RAM_BUFFER_SIZE;
    if (!_ramBufferFull && _ramBufferIndex == 0)
        _ramBufferFull = true;
    if (++_writeCounter >= DEFAULT_EEPROM_WRITE_INTERVAL)
    {
        writeToEEPROM(validReading);
        _writeCounter = 0;
    }
}
void SmartUltrasonic::loadFromEEPROM()
{ /* ... same as before ... */
    _eepromHeadPtr = EEPROM.read(_eepromStartAddr);
    if (_eepromHeadPtr >= DEFAULT_EEPROM_BUFFER_SIZE)
        _eepromHeadPtr = 0;
    for (int i = 0; i < DEFAULT_RAM_BUFFER_SIZE; i++)
    {
        int addr = (_eepromHeadPtr - 1 - i + DEFAULT_EEPROM_BUFFER_SIZE) % DEFAULT_EEPROM_BUFFER_SIZE;
        byte rawVal = EEPROM.read(_eepromStartAddr + 1 + addr);
        _ramBuffer[DEFAULT_RAM_BUFFER_SIZE - 1 - i] = (float)rawVal * DEFAULT_EEPROM_SCALE_FACTOR;
    }
    _ramBufferIndex = 0;
    _ramBufferFull = true;
}
void SmartUltrasonic::writeToEEPROM(float value)
{ /* ... same as before ... */
    byte comp = (byte)round(value / DEFAULT_EEPROM_SCALE_FACTOR);
    EEPROM.write(_eepromStartAddr + 1 + _eepromHeadPtr, comp);
    _eepromHeadPtr = (_eepromHeadPtr + 1) % DEFAULT_EEPROM_BUFFER_SIZE;
    EEPROM.write(_eepromStartAddr, _eepromHeadPtr);
}
float SmartUltrasonic::calculateStdDev()
{ /* ... same as before ... */
    int c = _ramBufferFull ? DEFAULT_RAM_BUFFER_SIZE : _ramBufferIndex;
    if (c < 2)
        return 0.0f;
    float sum = 0.0, mean = 0.0, sq = 0.0;
    for (int i = 0; i < c; i++)
        sum += _ramBuffer[i];
    mean = sum / c;
    for (int i = 0; i < c; i++)
        sq += pow(_ramBuffer[i] - mean, 2);
    return sqrt(sq / c);
}
int Smart_Ultrasonic::calculateConfidence(float stdDev)
{ /* ... same as before ... */
    if (stdDev < 1)
        return 90;
    if (stdDev < 5)
        return 75;
    if (stdDev < 10)
        return 60;
    return 40;
}