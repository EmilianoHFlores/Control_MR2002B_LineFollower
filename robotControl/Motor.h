  #ifndef Motor_h
#define Motor_h

#include <Arduino.h>
#include "MotorID.h"
#include "PID.h"

enum class MotorState{
  Backward = -1,
  Stop = 0,
  Forward = 1
};

class Motor{
  private:
    //Motor info
    uint8_t digital_one_;
    uint8_t digital_two_;
    int pwm_pin_;
    uint8_t encoderA_;
    uint8_t encoderB_;
    MotorID motorID_;
    MotorState current_state_;

    //VelocityData
    uint8_t pwm_ = 0;
    int pid_tics_ = 0;
    int tics_counter_ = 0;
    double current_speed_ = 0;
    double target_speed_ = 0;

  
    //Motor characteristics.
    static constexpr double kPulsesPerRevolution = 451.0;
    static constexpr double kWheelDiameter = 0.07;
    static constexpr double kRPM = 100;
    static constexpr double kRPS = kRPM / 60;
    static constexpr double kMaxVelocity = kRPS * M_PI * kWheelDiameter;
    static constexpr double kMaxPWM = 255;
    static constexpr double kPwmDeadZone = 0;
    static constexpr double kMinPwmForMovement = 0;
    static constexpr double kDistancePerRev = M_PI * kWheelDiameter;

    // PID.
    static constexpr uint8_t kPidMinOutputLimit = 30;
    static constexpr uint8_t kPidMaxOutputLimit = 255;
    static constexpr uint16_t kPidMaxErrorSum = 2000;
    static constexpr uint8_t kPidMotorTimeSample = 100;
    static constexpr double kOneSecondInMillis = 1000.0;
    static constexpr double kSecondsInMinute = 60;
    static constexpr double kPidCountTimeSamplesInOneSecond = kOneSecondInMillis/kPidMotorTimeSample;
    static constexpr double kPidCountTimeSamplesInOneMinute = kSecondsInMinute*kPidCountTimeSamplesInOneSecond;

    PID pid_;
    static constexpr double kP = 11; //60
    static constexpr double kI = 6; //55
    static constexpr double kD = 2; //40

  public:
    
    
    //Constructor
    Motor(uint8_t digitalOne, uint8_t digitalTwo, int pwmPin, uint8_t encoderA, uint8_t entcoderB, MotorID motorID);
    Motor();

    //Getters
    MotorState getCurrentState();
    double getCurrentSpeed();
    double getTargetSpeed();
    int getPidTics();
    int getEncoderTics();
    void setEncoderTics(int tics);

    //Encoders
    uint8_t getEncoderB();
    uint8_t getEncoderA();
    double getDistanceTraveled();

    //Methods
    void motorSetup();
    void deltaPidTics(int delta);
    void deltaEncoderTics(int delta);

    // Attach interrupt of encoders.
    void initEncoders();
    
    //Control
    void motorForward();
    void motorBackward();
    void motorStop();

    //Velocity Methods
    void setPWM(double PWM);
    double getPWM();
    double RPM2RPS(double velocity);
    void motorSpeedPID(double target_speed);
    void motorSpeedPWM(double new_pwm);
    void updateCurrentSpeed();

    //PID METHODS
    void PIDTunnigs(double kp, double ki, double kd);
};
#endif
