#ifndef Encoder_h
#define Encoder_h

#include <Arduino.h>
#include "Motor.h"
#include "Movement.h"

namespace Encoder{
  void updateTics(Motor *motor);
  
  void leftEncoder();
  void rightEncoder();
};

#endif 
