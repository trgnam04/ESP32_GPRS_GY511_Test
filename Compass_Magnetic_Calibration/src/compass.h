#ifndef COMPASS_H
#define COMPASS_H
#include <Adafruit_MPU6050.h>
#include <Adafruit_HMC5883_U.h>
#include <Adafruit_Sensor.h>
struct mag_calibrate_t
{
    float mag_calib_x;
    float mag_calib_y;
    float mag_calib_z;
    float mag_heading;
};

void MotionCal_display(sensors_event_t  *accel_event, sensors_event_t  *gyro_event, sensors_event_t *mag_event);
void calibrateMagnetic(sensors_event_t *event, mag_calibrate_t *result);
float calculateHeading(float *mag_data_sensor);
#endif // COMPASS_H