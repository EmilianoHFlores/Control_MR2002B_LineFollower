#include "ArduinoPlotter.h"

void arduinoPlot(double min, double max, double num1, double num2, double num3, double num4, double num5, double num6, double num7, double num8) {
  Serial.print(min);
    Serial.print(",");
    Serial.print(max);
    Serial.print(",");
    Serial.print(num1);
    Serial.print(",");
    Serial.print(num2);
    Serial.print(",");
    Serial.print(num3);
    Serial.print(",");
    Serial.print(num4);
    Serial.print(",");
    Serial.print(num5);
    Serial.print(",");
    Serial.print(num6);
    Serial.print(",");
    Serial.print(num7);
    Serial.print(",");
    Serial.print(num8);
    Serial.println();
}
