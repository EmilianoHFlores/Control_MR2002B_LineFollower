#include "LineSensor.h"

LineSensor::LineSensor(){
  sensor_left_ = Constants::kSensorLeft;
  sensor_center_left_ = Constants::kSensorCenterLeft;
  sensor_center_right_ = Constants::kSensorCenterRight;
  sensor_right_ = Constants::kSensorRight;

  sensor_threshold_ = Constants::kSensorThreshold;

  averages_[0] = 0;
  averages_[1] = 0;
  averages_[2] = 0;
  averages_[3] = 0;
}

void LineSensor::init(){
  pinMode(sensor_left_, INPUT);
  pinMode(sensor_center_left_, INPUT);
  pinMode(sensor_center_right_, INPUT);
  pinMode(sensor_right_, INPUT);
  calibrate(50);
}

void LineSensor::calibrate(int readings){
  for (int i = 0; i < readings; i++){
    readSensors();
    for (int j = 0; j < 4; j++){
      averages_[j] += sensor_values[j];
    }
  }
  for (int j = 0; j < 4; j++){
    averages_[j] /= readings;
  }
}

void LineSensor::readSensors(){
  readSensor(Left);
  readSensor(CenterLeft);
  readSensor(CenterRight);
  readSensor(Right);
}

void LineSensor::readSensor(sensorPin sensor){
  switch(sensor){
    case Left:
      sensor_values[0] = analogRead(sensor_left_);
      break;
    case CenterLeft:
      sensor_values[1] = analogRead(sensor_center_left_);
      break;
    case CenterRight:
      sensor_values[2] = analogRead(sensor_center_right_);
      break;
    case Right:
      sensor_values[3] = analogRead(sensor_right_);
      break;
  }
}

void LineSensor::lineDetected(bool *line_detected){
  readSensors();
  /* if calibrated over black line
  // center sensors should see black, so if they see white, line detected
  for (int i = 0; i < 4; i++){
    // brighter returns a lower value, so center sensors return 1 unless less than threshold
    if (i==1 || i==2){
        line_detected[i] = !(sensor_values[i] < averages_[i] - sensor_threshold_);
    }
    else{
        line_detected[i] = (sensor_values[i] > averages_[i] + sensor_threshold_);
    }
  }*/
  // if calibrated over white
  //Serial.println("----------------------");
  for (int i = 0; i < 4; i++){
    // brighter returns a lower value, so center sensors return 1 unless less than threshold
    //line_detected[i] = (sensor_values[i] > (averages_[i] + sensor_threshold_));
    line_detected[i] = (sensor_values[i] > 950);
    //Serial.print(sensor_values[i]);
    //Serial.print(" ");
  }
  //Serial.println();
}