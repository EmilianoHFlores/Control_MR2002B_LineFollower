#ifndef PID_h
#define PID_h

#include <Arduino.h>
#include <math.h>

class PID{
  private:
    double kp_ = 0;
    double ki_ = 0;
    double kd_ = 0;

    double error_sum_ = 0;
    double error_pre_ = 0;

    double max_error_;
    double min_output_;
    double max_output_;
    
    unsigned long time_passed_;
    unsigned long sample_time_;
    
  public:
    PID(const double kp, const double ki, const double kd, const double out_min, const double out_max, const double max_error_sum, const long sample_time);
    PID();

    void setTunnings(double kp, double ki, double kd);

    void computeSpeed(const double setpoint, double &input, double &output, int &reset_variable, const double pulses_per_rev,const double count_time_samples_in_one_second);
    void compute(const double setpoint, double &input, double &output);
    void reset();

    void infoPID();
};

#endif
