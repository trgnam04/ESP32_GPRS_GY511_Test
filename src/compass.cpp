#include "compass.h"

const float hard_iron[3] = {
    -15.41, -3.7, 2.17};

const float soft_iron[3][3] = {
    {0.982, -0.006, 0.004},
    {-0.006, 0.949, 0.002},
    {0.004, 0.002, 1.073}}; 
const float mag_decl = 0.0116; // Ho Chi Minh City declination
static float hi_cal[3];
static float mag_data[3];

void MotionCal_display(sensors_event_t  *accel_event, sensors_event_t  *gyro_event, sensors_event_t *mag_event)
{
    /* Display the results (magnetic vector values are in micro-Tesla (uT)) */
    Serial.print("Raw:");
    Serial.print(int(accel_event->acceleration.x * 8192 / 9.8));
    Serial.print(",");
    Serial.print(int(accel_event->acceleration.y * 8192 / 9.8));
    Serial.print(",");
    Serial.print(int(accel_event->acceleration.z * 8192 / 9.8));
    Serial.print(",");

    Serial.print(int(gyro_event->gyro.x * (180.0 / 3.141592653589793238463) * 16));
    Serial.print(",");
    Serial.print(int(gyro_event->gyro.y * (180.0 / 3.141592653589793238463) * 16));
    Serial.print(",");
    Serial.print(int(gyro_event->gyro.z * (180.0 / 3.141592653589793238463) * 16));
    Serial.print(",");

    Serial.print(int(mag_event->magnetic.x * 10));
    Serial.print(",");
    Serial.print(int(mag_event->magnetic.y * 10));
    Serial.print(",");
    Serial.print(int(mag_event->magnetic.z * 10));
    Serial.println("");

    Serial.print("Uni:");
    Serial.print(accel_event->acceleration.x);
    Serial.print(",");
    Serial.print(accel_event->acceleration.y);
    Serial.print(",");
    Serial.print(accel_event->acceleration.z);
    Serial.print(",");

    Serial.print(gyro_event->gyro.x, 4);
    Serial.print(",");
    Serial.print(gyro_event->gyro.y, 4);
    Serial.print(",");
    Serial.print(gyro_event->gyro.z, 4);
    Serial.print(",");

    Serial.print(mag_event->magnetic.x);
    Serial.print(",");
    Serial.print(mag_event->magnetic.y);
    Serial.print(",");
    Serial.print(mag_event->magnetic.z);
    Serial.println("");
}
void calibrateMagnetic(sensors_event_t *event, mag_calibrate_t *result)
{
    mag_data[0] = event->magnetic.x;
    mag_data[1] = event->magnetic.y;
    mag_data[2] = event->magnetic.z;
    // Apply hard-iron calibration
    for (uint8_t i = 0; i < 3; i++)
    {
        hi_cal[i] = mag_data[i] - hard_iron[i];
    }
    // Apply soft-iron scaling
    for (uint8_t i = 0; i < 3; i++)
    {
        mag_data[i] = (soft_iron[i][0] * hi_cal[0]) +
                      (soft_iron[i][1] * hi_cal[1]) +
                      (soft_iron[i][2] * hi_cal[2]);
    }
    result->mag_calib_x = mag_data[0];
    result->mag_calib_y = mag_data[1];
    result->mag_calib_z = mag_data[2];
    result->mag_heading = calculateHeading(mag_data);
}
float calculateHeading(float *mag_data_sensor)
{
    float heading = 0;
    // Calculate angle for heading, assuming board is parallel to
    // the ground and  Y points toward heading.
    heading = atan2(mag_data_sensor[1], mag_data_sensor[0]);

    // Apply magnetic declination to convert magnetic heading
    // to geographic heading
    heading += mag_decl;

    heading = heading * 180 / M_PI;
    // Convert heading to 0..360 degrees
    if (heading < 0)
    {
        heading += 360;
    }
    else if (heading > 360)
    {
        heading -= 360;
    }
    return heading;
}