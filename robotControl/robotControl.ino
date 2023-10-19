#include "PID.h"
#include "Motor.h"
#include "LineSensor.h"
#include "ArduinoPlotter.h"

LineSensor linesensor;

// Motor left
uint8_t motor_left_digital_one_ = 13;
uint8_t motor_left_digital_two_ = 12;
int motor_left_pwm_pin_ = 4;
uint8_t motor_left_encoderA_ = 3;
uint8_t motor_left_encoderB_ = 3;

// Motor right
uint8_t motor_right_digital_one_ = 7;
uint8_t motor_right_digital_two_ = 6;
int motor_right_pwm_pin_ = 5;
uint8_t motor_right_encoderA_ = 2;
uint8_t motor_right_encoderB_ = 2;

// PID objects
PID *pid_motor_left_ = nullptr;
PID *pid_motor_right_ = nullptr;

// PID tics
int motor_left_pid_tics_ = 0;
int motor_right_pid_tics_ = 0;

// PID variables
const double motor_right_kp =  0.5480; //15;
const double motor_right_ki = 0.2264; // 6;
const double motor_right_kd = 0.1822; // 2;

const double motor_left_kp =  0.5480; 
const double motor_left_ki = 0.2264; 
const double motor_left_kd = 0.1822; 

const long kSampleTime = 20;
unsigned long PRBS_sample_time_ = 0; 
unsigned long passed_time_ = 0; 
unsigned long PRBS_passed_time_ = 0; 
unsigned long curr_time_ = 0;

// Motor speed
int target_speed_ = 60;
double motor_left_current_speed_ = 0;
double motor_right_current_speed_ = 0;
bool state_ = false;

// Motor parameters
double motor_left_pwm_ = 0;
double motor_right_pwm_ = 0;
double motor_left_pwm_PRBS_ = 0;
double motor_right_pwm_PRBS_ = 0;
const double max_voltage_ = 5;
const double kMinPwm = 30.0;
const double kMaxPWM = 255.0;
const double kMaxRPM = 180.0;
const double kMaxPIDErrorSum = 10.0;
static constexpr double kPulsesPerRevolution = 451.0;
static constexpr double kOneSecondInMillis = 1000.0;
static constexpr double kSecondsInMinute = 60;
static constexpr double kPidCountTimeSamplesInOneSecond = kOneSecondInMillis/kSampleTime;
static constexpr double kPidCountTimeSamplesInOneMinute = kSecondsInMinute*kPidCountTimeSamplesInOneSecond;

void setup() {
  Serial.begin(115200);

  linesensor.init();

  delay(1000);

  PID init_motor_right(motor_left_kp, motor_left_ki, motor_left_kd, 0, 5, kMaxPIDErrorSum, kSampleTime);
  PID init_motor_left(motor_right_kp, motor_right_ki, motor_right_kd, 0, 5, kMaxPIDErrorSum, kSampleTime);
  
  // Initialize PID controllers
  pid_motor_left_ = &init_motor_left;
  pid_motor_right_ = &init_motor_right;

  // Motor Left
  pinMode(motor_left_digital_one_, OUTPUT);
  pinMode(motor_left_digital_two_, OUTPUT);
  pinMode(motor_left_pwm_pin_, OUTPUT);
  pinMode(motor_left_encoderA_, INPUT);
  pinMode(motor_left_encoderB_, INPUT);

  //Motor Stop 
  analogWrite(motor_left_pwm_pin_, LOW);
  digitalWrite(motor_left_digital_one_, LOW);
  digitalWrite(motor_left_digital_two_, LOW);

  // Motor Left
  pinMode(motor_right_digital_one_, OUTPUT);
  pinMode(motor_right_digital_two_, OUTPUT);
  pinMode(motor_right_pwm_pin_, OUTPUT);
  pinMode(motor_right_encoderA_, INPUT);
  pinMode(motor_right_encoderB_, INPUT);

  //Motor Stop 
  analogWrite(motor_right_pwm_pin_, LOW);
  digitalWrite(motor_right_digital_one_, LOW);
  digitalWrite(motor_right_digital_two_, LOW);

  // Pull up resistors
  digitalWrite(motor_left_encoderA_, HIGH); // Activate pull up resistors
  digitalWrite(motor_right_encoderA_, HIGH); // Activate pull up resistors

  // Attach interrupt 
  attachInterrupt(digitalPinToInterrupt(motor_left_encoderA_), update_left_tics, RISING);
  attachInterrupt(digitalPinToInterrupt(motor_right_encoderA_), update_right_tics, RISING);

  Serial.println("Attached interrupt pins successfully");

  //Motor left forward
  digitalWrite(motor_left_digital_one_, HIGH);
  digitalWrite(motor_left_digital_two_, LOW);

  //Motor right forward
  digitalWrite(motor_right_digital_one_, HIGH);
  digitalWrite(motor_right_digital_two_, LOW);


  arduinoPlot(0, 5, RPM2RPS(target_speed_), motor_left_current_speed_, motor_right_current_speed_, 0, 0,0,0,0);

  passed_time_ = millis();
  PRBS_passed_time_ = millis();
}

bool line_detected[4] = {false, false, false, false};
float line_error = 0;
float multiplier = 60;

void loop() {
  curr_time_ = millis();

  linesensor.lineDetected(line_detected);

  //arduinoPlot(0, 5, line_detected[0], line_detected[1], line_detected[2], line_detected[3], 0,0,0,0);
  arduinoPlot(0, 5, RPM2RPS(target_speed_), motor_left_current_speed_, motor_right_current_speed_, motor_left_pwm_, motor_right_pwm_,0,0,0);
  if (curr_time_ - PRBS_passed_time_ > 10000){
   target_speed_ += 30;
   PRBS_passed_time_ = millis();
  } 

  // positive if line is to the right
  line_error = 0;//line_detected[0] * multiplier - line_detected[3] * multiplier;
  
    
  pid_motor_left_->computeSpeed(RPM2RPS(target_speed_ - line_error), motor_left_current_speed_, motor_left_pwm_, motor_left_pid_tics_, kPulsesPerRevolution, kPidCountTimeSamplesInOneSecond);
  pid_motor_right_->computeSpeed(RPM2RPS(target_speed_ + line_error), motor_right_current_speed_, motor_right_pwm_, motor_right_pid_tics_, kPulsesPerRevolution, kPidCountTimeSamplesInOneSecond);

  analogWrite(motor_left_pwm_pin_, motor_left_pwm_*255/5);
  analogWrite(motor_right_pwm_pin_, motor_right_pwm_*255/5);
}

double RPM2RPS(double velocity){
  return velocity/kSecondsInMinute;
}

void update_left_tics(){
  motor_left_pid_tics_++;
}

void update_right_tics(){
  motor_right_pid_tics_++;
}
