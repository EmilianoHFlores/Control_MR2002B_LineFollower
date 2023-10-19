/*#include "Encoder.h"

void Encoder::updateTics(Motor *motor){
  motor->deltaPidTics(1);
  if(motor->getCurrentState() == MotorState::Forward){
    motor->deltaEncoderTics(1);
  }
  else if (motor->getCurrentState() == MotorState::Backward){
    motor->deltaEncoderTics(-1);
  }
  else {
    return;
  }
}

void Encoder::leftEncoder() {
  Serial.println("Motor left interrupt");
  updateTics(motor_left);
}
  
void Encoder::rightEncoder() {
  Serial.println("Motor right interrupt");
  updateTics(motor_right);
}*/