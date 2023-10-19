#include "Motor.h"
#include "Encoder.h"

//Constructores
Motor::Motor(){
  digital_one_ = 0;
  digital_two_ = 0;
  pwm_pin_ = 0;
  encoderA_ = 0;
  encoderB_ = 0;
}

Motor::Motor(uint8_t digitalOne, uint8_t digitalTwo, int pwmPin, uint8_t encoderA, uint8_t encoderB, MotorID motorID) :
pid_(kP, kI, kD, kPidMinOutputLimit, kPidMaxOutputLimit, kPidMaxErrorSum, kPidMotorTimeSample){
  digital_one_ = digitalOne;
  digital_two_ = digitalTwo;
  pwm_pin_ = pwmPin;
  motorID_ = motorID;
  encoderA_ = encoderA;
  encoderB_ = encoderB;
}


//Getters Definidos
uint8_t Motor::getEncoderA(){
  return encoderA_;
}
  
uint8_t Motor::getEncoderB(){
  return encoderB_;
}

MotorState Motor::getCurrentState(){
  return current_state_;
}
  
int Motor::getEncoderTics(){
  return tics_counter_;
}
  
double Motor::getCurrentSpeed(){
  return current_speed_;
}
  
double Motor::getTargetSpeed(){
  return RPM2RPS(target_speed_);
}

int Motor::getPidTics(){
  return pid_tics_;
}
  

//Metodos Definidos
void Motor::motorSetup(){
  pinMode(digital_one_, OUTPUT);
  pinMode(digital_two_, OUTPUT);
  pinMode(pwm_pin_, OUTPUT);
  pinMode(encoderA_, INPUT);
  pinMode(encoderB_, INPUT);
}

void Motor::initEncoders(){ 
  return;
  switch(motorID_){
    case MotorID::Left:
      
      Serial.print("Attached left encoder to ");
      Serial.println(encoderA_);
    break;
    case MotorID::Right:
      attachInterrupt(digitalPinToInterrupt(encoderA_), Encoder::rightEncoder, RISING);
      Serial.print("Attached right encoder to ");
      Serial.println(encoderA_);
    break;
  }
}

void Motor::deltaPidTics(int delta){
  pid_tics_ += delta; 
  //Serial.print((int)motorID_);
  //Serial.print(" pid tics: ");
  //Serial.println(pid_tics_);
}

void Motor::deltaEncoderTics(int delta){
  tics_counter_ += delta; 
}

//Control
void Motor::motorForward(){
  analogWrite(pwm_pin_, pwm_);

  if (current_state_ == MotorState::Forward){
    return;
  }
  
  digitalWrite(digital_one_, HIGH);
  digitalWrite(digital_two_, LOW);
  
  pid_.reset();

  current_state_ = MotorState::Forward;
}

void Motor::motorBackward(){
  analogWrite(pwm_pin_, pwm_);

  if (current_state_ == MotorState::Backward){
    return;
  }
  
  digitalWrite(digital_one_, LOW);
  digitalWrite(digital_two_, HIGH);

  pid_.reset();

  current_state_ = MotorState::Backward;
}
    
void Motor::motorStop(){
  analogWrite(pwm_pin_, LOW);

  if (current_state_ == MotorState::Stop){
    return;
  }
  
  digitalWrite(digital_one_, LOW);
  digitalWrite(digital_two_, LOW);

  pid_.reset();

  current_state_ = MotorState::Stop;
}

// Velocity. 
void Motor::setPWM(double PWM){
  pwm_ = PWM;
  switch(current_state_) {
    case MotorState::Forward:
      motorForward();
    break;
    case MotorState::Backward:
      motorBackward();
    break;
    case MotorState::Stop:
      motorStop();
    break;
  }
}

double Motor::RPM2RPS(double velocity){
  return velocity/kSecondsInMinute;
}

void Motor::updateCurrentSpeed(){
  current_speed_ = (pid_tics_ / kPulsesPerRevolution) * kPidCountTimeSamplesInOneSecond;
  pid_tics_ = 0;
}

void Motor::motorSpeedPID(double target_speed){
  int speed_sign = min(1, max(-1, target_speed * 1000));
  target_speed_ = fabs(target_speed);
  double tmp_pwm = pwm_;
  switch (speed_sign)
  {
  case 0:
    motorStop();
  break;
  case 1:
    motorForward();
  break;
  case -1:
    motorBackward();
  break;
  }
  
  pid_.computeSpeed(RPM2RPS(target_speed_), current_speed_, tmp_pwm, pid_tics_, kPulsesPerRevolution, kPidCountTimeSamplesInOneSecond);
  
  setPWM(tmp_pwm);
}

double Motor::getPWM(){
  return pwm_;
}

void Motor::motorSpeedPWM(double new_pwm){
  int speed_sign = min(1, max(-1, new_pwm * 1000));
  new_pwm = fabs(new_pwm);

  switch (speed_sign)
  {
  case 0:
    motorStop();
  break;
  case 1:
    motorForward();
  break;
  case -1:
    motorBackward();
  break;
  }

  updateCurrentSpeed();
  
  setPWM(new_pwm);
}


double Motor::getDistanceTraveled(){
  return (getEncoderTics()/kPulsesPerRevolution) * kDistancePerRev; 
}

void Motor::setEncoderTics(int tics){
  tics_counter_=tics;
}

void Motor::PIDTunnigs(double kp, double ki, double kd){
  pid_.setTunnings(kp,ki,kd);
}