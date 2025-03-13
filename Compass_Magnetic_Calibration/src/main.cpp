#include <Adafruit_MPU6050.h>
#include <Adafruit_HMC5883_U.h>
#include <Adafruit_Sensor.h>
#include "compass.h"
#include "utils.h"
#include <Wire.h>

Adafruit_MPU6050 mpu;
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);
void setup(void) {
  Serial.begin(115200);
  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("Adafruit MPU6050 test!");

  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  // Enable Magnetometer
  mpu.setI2CBypass(true);

  if(!mag.begin())
  {
    /* There was a problem detecting the HMC5883 ... check your connections */
    Serial.println("Ooops, no HMC5883 detected ... Check your wiring!");
    while(1);
  }
  /* Display some basic information on this sensor */
  Serial.println("HMC5883 Found!");

  Serial.println("");
  delay(100);
}

void loop() {
    /* Get new sensor events with the readings */
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    sensors_event_t event; 
    mag.getEvent(&event);
    mag_calibrate_t result;
    calibrateMagnetic(&event, &result);
    Serial.print("Mag: ");
    Serial.print(result.mag_calib_x);
    Serial.print(",");
    Serial.print(result.mag_calib_y);
    Serial.print(",");
    Serial.print(result.mag_calib_z);
    Serial.print(",");
    Serial.println(result.mag_heading);
    // MotionCal_display(&a, &g, &event);

    delay(2);
}