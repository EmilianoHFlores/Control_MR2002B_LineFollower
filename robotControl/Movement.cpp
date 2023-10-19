#include "Movement.h"


Movement::Movement(){
  motor_[MOTOR_LEFT] = Motor(kDigitalPinsLeftMotor[1], kDigitalPinsLeftMotor[0], 
                      kAnalogPinLeftMotor, kEncoderPinsLeftMotor[0], 
                      kEncoderPinsLeftMotor[1], MotorID::Left);   
  motor_[MOTOR_RIGHT] = Motor(kDigitalPinsRightMotor[1], kDigitalPinsRightMotor[0], 
                      kAnalogPinRightMotor, kEncoderPinsRightMotor[0], 
                      kEncoderPinsRightMotor[1], MotorID::Right);  
  Serial.println("Initializing robot"); 
  initRobot();
}

void Movement::initRobot(){
  for(int i=0; i<kMotorCount; i++){
    motor_[i].motorSetup();
    motor_[i].initEncoders();
    motor_[i].motorStop();
  }
}

void Movement::stop(){
  for(int i=0; i<kMotorCount; i++){
    motor_[i].motorStop();
  }
}

void Movement::cmdVelocity(int RPMs){
  motor_[MOTOR_LEFT].motorSpeedPID(kMovementRPMs);
  motor_[MOTOR_RIGHT].motorSpeedPID(kMovementRPMs); 
}
