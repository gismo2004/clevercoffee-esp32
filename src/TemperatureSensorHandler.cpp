#include "TemperatureSensorHandler.h"

TemperatureSensorHandler::TemperatureSensorHandler() {
    _errorLogInterval = 1000;
    _uLim = 150;
    _lLim = 0;
    _maxDeviationBetweenReadings = 5;
    _maxErrorActiveTime = 5000;
}

void TemperatureSensorHandler::Init(int gpio, SensorType type) {
    _gpio = gpio;
    _sensorType = type;
    if (_sensorType == DALLAS) {
        _sensors_dallas.setOneWire(&_oneWire);
        _oneWire.begin(_gpio);
        _sensors_dallas.begin();
        // we handle this...
        _sensors_dallas.setWaitForConversion(false);
        // add a bit of comfort level...
        _waitTime = _sensors_dallas.millisToWaitForConversion(11) * 1.1;
        _sensorAvailable = CheckNumOfDevicesOnBus();
    }

    if (_sensorType == TSIC_306) {
        _sensors_zacwire.reset(new ZACwire(_gpio, 306));
        _waitTime = 400; // ms
        // begin() should be started at least 2ms before the first .getTemp().
        _sensorAvailable = _sensors_zacwire->begin();
        delay(2);
    }
}

bool TemperatureSensorHandler::CheckNumOfDevicesOnBus() {
    int numberOfDevicesOnWire = _sensors_dallas.getDeviceCount();
    debugPrintf("CheckNumOfDevicesOnBus found %d devices on GPIO %d\n", numberOfDevicesOnWire, _gpio);
    if (numberOfDevicesOnWire > 1)
        debugPrintln("[WARNING] Only one dallas sensor supported per GPIO! --> using Sensor with index 0");
    return numberOfDevicesOnWire > 0;
}

void TemperatureSensorHandler::RefreshSensorData() {
    if (_sensorType == DALLAS)
        ReadTemperatureSensorDallas();
    if (_sensorType == TSIC_306)
        ReadTemperatureSensorTSIC();

    CheckErrors();
    if (_newData) {
        CalculateMovingAverage();
        InvokeDataChanged(Temperature, ChangeRate, SensorInErrorState);
        _newData = false;
    }
}

void TemperatureSensorHandler::ReadTemperatureSensorDallas() {
    if (_dataRequested == true && millis() - _previousRequestMillis > _waitTime) {
        float tempC = _sensors_dallas.getTempCByIndex(0);
        _dataRequested = false;
        _sensorBadReading = true;
        if (tempC > DEVICE_DISCONNECTED_C) {
            _sensorBadReading = false;
            _newData = true;
            // initialize correctly
            if (_tempAvailable) {
                _previousTemperature = Temperature;
            } else {
                _tempAvailable = true;
                _previousTemperature = tempC;
            }
            Temperature = tempC;

            return;
        }
    }

    if (_dataRequested == false) {
        _previousRequestMillis = millis();
        _sensors_dallas.requestTemperatures();
        _dataRequested = true;
    }
}

void TemperatureSensorHandler::ReadTemperatureSensorTSIC() {
    if (millis() - _previousRequestMillis > _waitTime) {
        _previousRequestMillis = millis();

        float tempC = _sensors_zacwire->getTemp();
        if (tempC != 222 && tempC != 221) {
            _sensorBadReading = false;
            Temperature = tempC;
        }
        _sensorBadReading = true;
    }
}

void TemperatureSensorHandler::SetSensorParams(float tempLowerLimit,
                                               float tempUpperLimit) {
    _lLim = tempLowerLimit;
    _uLim = tempUpperLimit;
}

void TemperatureSensorHandler::CalculateMovingAverage() {
    const int numValues = 15;
    static double tempValues[numValues];
    static unsigned long timeValues[numValues];
    static double tempChangeRates[numValues];
    static int valueIndex = 1;
    static bool movingAverageInitialized;

    if (!movingAverageInitialized) {
        for (int index = 0; index < numValues; index++) {
            tempValues[index] = Temperature;
            timeValues[index] = 0;
            tempChangeRates[index] = 0;
        }

        movingAverageInitialized = true;
    }

    timeValues[valueIndex] = millis();
    tempValues[valueIndex] = Temperature;

    double tempChangeRate = 0; // local change rate of temperature
    if (valueIndex == numValues - 1) {
        tempChangeRate = (tempValues[numValues - 1] - tempValues[0]) /
                         (timeValues[numValues - 1] - timeValues[0]) * 10000;
    } else {
        tempChangeRate = (tempValues[valueIndex] - tempValues[valueIndex + 1]) /
                         (timeValues[valueIndex] - timeValues[valueIndex + 1]) *
                         10000;
    }
    tempChangeRates[valueIndex] = tempChangeRate;

    double totalTempChangeRateSum = 0;

    for (int i = 0; i < numValues; i++) {
        totalTempChangeRateSum += tempChangeRates[i];
    }

    ChangeRate = totalTempChangeRateSum / numValues * 100;

    if (valueIndex >= numValues - 1) {
        // ...wrap around to the beginning:
        valueIndex = 0;
    } else {
        valueIndex++;
    }
}

void TemperatureSensorHandler::CheckErrors() {
    char logMessage[255];
    bool errorFound = false;

    // NOTE: messages are in prio order
    if (_tempAvailable && Temperature <= _lLim || Temperature >= _uLim) {
        sprintf(logMessage, "Sensor value out of limits (val: %f / min: %f / max: %f)", Temperature, _lLim, _uLim);
        errorFound = true;
    }

    if (fabs(Temperature - _previousTemperature) > _maxDeviationBetweenReadings) {
        sprintf(logMessage, "Too big deviation between current and last value (val: %f / lim: %f)", fabs(Temperature - _previousTemperature), _maxDeviationBetweenReadings);
        errorFound = true;
    }

    if (_sensorBadReading) {
        sprintf(logMessage, "Sensor returned invalid value...");
        errorFound = true;
    }

    if (!_sensorAvailable) {
        sprintf(logMessage, "No Sensor found, please check your hardware setup...");
        errorFound = true;
    }

    // first time error --> write log in any case and set event time
    if (!_errorActive && !SensorInErrorState && errorFound) {
        _errorLogLastWritten = millis();
        _errorStateDetectedMillis = _errorLogLastWritten;
        debugPrintln("first");
        debugPrintln(logMessage);
        _errorActive = true;
        return;
    }

    if (!SensorInErrorState && errorFound && millis() - _errorStateDetectedMillis > _maxErrorActiveTime) {
        SensorInErrorState = true;
        debugPrintln("ERROR: temperature sensor malfunction!");
        InvokeDataChanged(Temperature, ChangeRate, SensorInErrorState);
        return;
    }

    // just log
    if (errorFound && !SensorInErrorState && millis() - _errorLogLastWritten > _errorLogInterval) {
        _errorLogLastWritten = millis();
        debugPrintln(logMessage);
        return;
    }

    if (!errorFound)
        _errorActive = false;
}