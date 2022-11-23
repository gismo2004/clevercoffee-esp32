#ifndef TemperatureSensorHandler_h
#define TemperatureSensorHandler_h

#include <DallasTemperature.h>
#include <OneWire.h>
#include <ZACwire.h>

#include "Arduino.h"
#include "debugSerial.h"

class TemperatureSensorHandler
{
    typedef void (*callbackFunction)(const float arg1, const float arg2, bool arg3);

public:
    enum SensorType {

        DALLAS,  // SensorType DALLAS
        TSIC_306 // SensorType TSIC (306)
    };
    TemperatureSensorHandler();
    /**
     * @brief Callback to update sensor data if new values are available;
     * @param 'newFunction' function that is called
     */
    void OnDataChanged(callbackFunction newFunction) {
        _callbackFunction = newFunction;
    }
    void InvokeDataChanged(float temp, float rate, bool error) {
        _callbackFunction(temp, rate, error);
    }
    /**
     * @brief Refresh sensor data; Call this in loop();
     */
    void RefreshSensorData();
    /**
     * @brief Initialize sensor for given GPIO and Type;
     * @param 'gpio' pin number
     * @param 'type' ENUM 'SensorType' that specifies the type of the sensor
     */
    void Init(int gpio, SensorType type);
    /**
     * @brief Set parameters for the sensor
     * @param 'tempLowerLimit' sets the lower temperature limit
     * @param 'tempUpperLimit' sets the upper temperature limit
     */
    void SetSensorParams(float tempLowerLimit, float tempUpperLimit);
    bool SensorInErrorState;
    float Temperature, ChangeRate;

private:
    bool CheckNumOfDevicesOnBus();
    void ReadTemperatureSensorDallas();
    void ReadTemperatureSensorTSIC();
    void CheckErrors();
    void CalculateMovingAverage();

    OneWire _oneWire;
    DallasTemperature _sensors_dallas;
    std::unique_ptr<ZACwire> _sensors_zacwire;
    callbackFunction _callbackFunction;
    SensorType _sensorType;
    float _previousTemperature, _uLim, _lLim, _maxDeviationBetweenReadings;
    unsigned long _previousRequestMillis, _errorStateDetectedMillis, _errorLogLastWritten;
    int _gpio, _waitTime, _errorLogInterval, _maxErrorActiveTime;
    bool _dataRequested, _sensorAvailable, _tempAvailable, _sensorBadReading, _errorActive, _newData;
};

#endif
