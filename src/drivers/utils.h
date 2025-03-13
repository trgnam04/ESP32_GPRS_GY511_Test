#ifndef UTILS_H  
#define UTILS_H

#include "utils.h"
#include "math.h"
#include "Arduino.h"

float extractVectorToNorthVolume(float velocity, float headingRadians);
float extractVectorToEastVolume(float velocity, float headingRadians);
void calculateNorthEastVelocity(float v_x, float v_y, float headingRadians, float &v_B, float &v_D);
void calculateNextLocation(double latitude, double longtitude, float v_B, float v_D, float delta, double &next_latitude, double &next_longtitude);

#endif //UTILS_H