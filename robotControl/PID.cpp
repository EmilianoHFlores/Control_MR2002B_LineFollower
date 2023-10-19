#include "PID.h"

PID::PID(){
  time_passed_ = millis();
}

PID::PID(const double kp, const double ki, const double kd, const double out_min, const double out_max, const double max_error_sum, const long sample_time){
  time_passed_ = millis();
  setTunnings(kp, ki, kd);
  sample_time_ = sample_time;

  max_error_ = max_error_sum;
  min_output_ = out_min;
  max_output_ = out_max;
}

void PID::infoPID(){
  //Serial.println("PID INFORMATION");
  //Serial.print("kP = ");
  //Serial.print(kp_);
  //Serial.print("  kI = ");
  //Serial.print(ki_);
  //Serial.print("  kD = ");
  //Serial.print(kd_);
  //Serial.print("  Sample time = ");
  //Serial.print(sample_time_);
  //Serial.print("  MaxError = ");
  //Serial.print(max_error_);
  //Serial.print("  OutputMIN = ");
  //Serial.print(min_output_);
  //Serial.print("  OutputMAX = ");
  //Serial.println(max_output_);
  //Serial.println(" ");
}

void PID::setTunnings(double kp, double ki, double kd){
  kp_ = kp;
  ki_ = ki;
  kd_ = kd;
}
//Pid_.compute(RPM2RPS(target_speed_), current_speed_, tmp_pwm, pid_tics_, kPulsesPerRevolution, kPidCountTimeSamplesInOneSecond);
void PID::computeSpeed(const double setpoint, double &input, double &output, int &reset_variable, const double pulses_per_rev,const double count_time_samples_in_one_second) {
  //infoPID();
  
  if(millis()-time_passed_ < sample_time_) {
      return;
  }
  
  input = (reset_variable / pulses_per_rev) * count_time_samples_in_one_second;
  reset_variable = 0;

  double angular_velocity = input * (2 * PI);

  const double error = (setpoint * 2 * PI) - angular_velocity;
  output = error * kp_ + error_sum_ * ki_ + (error - error_pre_) * kd_;
  
  error_pre_ = error;
  error_sum_ += error;

  error_sum_ = max(max_error_ * -1, min(max_error_, error_sum_));
  output = max(min_output_, min(max_output_, output));

  time_passed_ = millis();
}

void compute(const double setpoint, double &input, double &output){
  return;
}

void PID::reset(){
  error_sum_ = 0;
  error_pre_ = 0;
}
