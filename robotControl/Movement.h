#ifndef Movement_h
#define Movement_h

#include <Arduino.h>
#include <math.h>
#include "Motor.h"


// Motor Ids
#define MOTOR_LEFT 0
#define MOTOR_RIGHT 1

class Movement{
  private:
    // Motor.
    static constexpr int kMotorCount = 2;

    static constexpr uint8_t kDigitalPinsLeftMotor[2] = {24, 25};
    static constexpr uint8_t kAnalogPinLeftMotor = 6;
    static constexpr uint8_t kEncoderPinsLeftMotor[2] = {2, 26};//A,B

    static constexpr uint8_t kDigitalPinsRightMotor[2] = {23, 22};
    static constexpr uint8_t kAnalogPinRightMotor = 7;
    static constexpr uint8_t kEncoderPinsRightMotor[2] = {3, 26};//A,B

    //PID
    static constexpr double kP = 7; //60
    static constexpr double kI = 3; //55
    static constexpr double kD = 2; //40

    // Constant Speed kMovementRPMs
    static constexpr int kMovementRPMs = 60;
  public:
    // Motor Array.
    Motor motor_[2];
    
    // Constructor.
    Movement();
    
    // Methods.
    void initRobot();

    // Robot Control.
    void stop();

    // Encoder methods
    void resetEncoders();

    // Comand Movement, using PID and static velocity
    void cmdVelocity(int RPMs); 
};


#endif
