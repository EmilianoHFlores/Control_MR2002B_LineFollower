#include "PID.h"
#include "Motor.h"
#include "LineSensor.h"
#include "ArduinoPlotter.h"
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#define MAX_STRING_SIZE 4 // Maximum string length you expect to receive

char received_buffer[MAX_STRING_SIZE];
uint8_t received_buffer_index = 0;

RF24 radio(9, 8);
const byte address[6] = "10001";
bool receivedData = false; // Flag to track received data

uint8_t rpms;

LineSensor linesensor;



bool line_detected[4] = {false, false, false, false};
float line_error = 0;
float multiplier;

enum lineState{
  lineLeft,
  lineCenterLeft,
  lineCenter,
  lineCenterRight,
  lineRight
};

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

// line leds
uint8_t line_leds_[4] = {30, 28, 26, 24};

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

const double motor_left_kp = 0.5480;// 0.5328; 
const double motor_left_ki = 0.2264; //0.2022; 
const double motor_left_kd = 0.1822;//0.1868; 

const long kSampleTime = 20;
unsigned long PRBS_sample_time_ = 0; 
unsigned long passed_time_ = 0; 
unsigned long PRBS_passed_time_ = 0; 
unsigned long curr_time_ = 0;

// Motor speed
int target_speed_ = 80;
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

  radio.begin();
  radio.openReadingPipe(1, address); // Use the same address as the transmitter
  radio.setPALevel(RF24_PA_MIN);
  radio.startListening(); // Start listening for incoming data

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

  // line leds
  for (int i=0; i<4; i++){
    pinMode(line_leds_[i], OUTPUT);
  }

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

lineState line_state = lineCenter;

void loop() {
  if (radio.available()) {
    radio.read(received_buffer, MAX_STRING_SIZE - 1);
    uint8_t value = received_buffer[0];
    target_speed_ = value;
    /*Serial.print("Value: ");
    Serial.println(value);*/
  }

  curr_time_ = millis();
  multiplier = target_speed_ / 2;

  linesensor.lineDetected(line_detected);

  for (int i=0; i<4; i++){
    if (line_detected[i]){
      digitalWrite(line_leds_[i], HIGH);
    } else {
      digitalWrite(line_leds_[i], LOW);
    }
  }

  //arduinoPlot(0, 5, line_detected[0], line_detected[1], line_detected[2], line_detected[3], 0,0,0,0);
  arduinoPlot(0, 5, RPM2RPS(target_speed_), motor_left_current_speed_, motor_right_current_speed_, motor_left_pwm_, motor_right_pwm_,0,0,0);
  /*if (curr_time_ - PRBS_passed_time_ > 10000){
   target_speed_ += 30;
   PRBS_passed_time_ = millis();
  } */

  // positive if line is to the right
  lineState prev_state = line_state;
  if (line_detected[0]){
    line_state = lineLeft;
  }
  else if (line_detected[1] && !line_detected[2]){
    line_state = lineCenterLeft;
  }
  else if (!line_detected[1] && line_detected[2]){
    line_state = lineCenterRight;
  }
  else if (line_detected[1] && line_detected[2]){
    line_state = lineCenter;
  }
  else if (line_detected[3]){
    line_state = lineRight;
  }

  switch (line_state){
    case lineLeft:
      line_error = -multiplier;
      break;
    case lineCenterLeft:
      if (prev_state == lineCenterLeft)
        line_error = -multiplier/2;
      else
        line_error = -multiplier/4;
      break;
    case lineCenter:
      line_error = 0;
      break;
    case lineCenterRight:
      if (prev_state == lineCenterRight)
        line_error = multiplier/2;
      else
        line_error = multiplier/4;
      break;
    case lineRight:
      line_error = multiplier;
      break;
  }

  /*Serial.print("Motor1: ");
  Serial.print(target_speed_ + line_error);
  Serial.print(" ");
  Serial.print("Motor2: ");
  Serial.println(target_speed_ - line_error);*/

  pid_motor_left_->computeSpeed(RPM2RPS(target_speed_ + line_error), motor_left_current_speed_, motor_left_pwm_, motor_left_pid_tics_, kPulsesPerRevolution, kPidCountTimeSamplesInOneSecond);
  pid_motor_right_->computeSpeed(RPM2RPS(target_speed_ - line_error), motor_right_current_speed_, motor_right_pwm_, motor_right_pid_tics_, kPulsesPerRevolution, kPidCountTimeSamplesInOneSecond);

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