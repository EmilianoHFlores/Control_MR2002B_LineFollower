// library to control line sensors
#ifndef LineSensor_h
#define LineSensor_h

#include <Arduino.h>
#include "Constants.h"

enum sensorPin{
    Left = Constants::kSensorLeft,
    CenterLeft = Constants::kSensorCenterLeft,
    CenterRight = Constants::kSensorCenterRight,
    Right = Constants::kSensorRight
    };

class LineSensor {
  private:
    uint8_t sensor_left_ = Constants::kSensorLeft;
    uint8_t sensor_center_left_ = Constants::kSensorCenterLeft;
    uint8_t sensor_center_right_ = Constants::kSensorCenterRight;
    uint8_t sensor_right_ = Constants::kSensorRight;

    int sensor_threshold_ = Constants::kSensorThreshold;

    int averages_[4] = {0,0,0,0};
    int sensor_values[4] = {0,0,0,0};

    public:
    LineSensor::LineSensor();
    void init();
    void calibrate(int readings);
    void readSensors();
    void readSensor(sensorPin sensor);
    void lineDetected(bool *line_detected);
    int linePosition();

};

#endif
