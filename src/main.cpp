#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include <Adafruit_L3GD20_U.h>
#include <Adafruit_HMC5883_U.h>
#include <SPI.h>

#define SDA GPIO_NUM_21
#define SCL GPIO_NUM_22

Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);
Adafruit_L3GD20_Unified gyro = Adafruit_L3GD20_Unified(54321);


void acc_displayRange(void);

void acc_init();
void gyro_init();
void mag_init();

void setup(void)
{
  Serial.begin(9600);
  Wire.begin();

  // acc_init();
  gyro_init();
  mag_init();
}

void loop(void)
{
  /* Get a new sensor event */
  sensors_event_t e_acc, e_gyro, e_mag;
  accel.getEvent(&e_acc);
  gyro.getEvent(&e_gyro);

  /* Display the results (acceleration is measured in m/s^2) */
  Serial.print("X: "); Serial.print(e_acc.acceleration.x); Serial.print("  ");
  Serial.print("Y: "); Serial.print(e_acc.acceleration.y); Serial.print("  ");
  Serial.print("Z: "); Serial.print(e_acc.acceleration.z); Serial.print("  ");Serial.println("m/s^2 ");

  Serial.print("X: "); Serial.print(e_gyro.gyro.x); Serial.print("  ");
  Serial.print("Y: "); Serial.print(e_gyro.gyro.y); Serial.print("  ");
  Serial.print("Z: "); Serial.print(e_gyro.gyro.z); Serial.print("  ");Serial.println("rad/s ");

  delay(500);
}


void acc_init(){
  Serial.println("Accelerator Test"); Serial.println("");
  if(!accel.begin())
  {
    /* There was a problem detecting the ADXL343 ... check your connections */
    Serial.println("Ooops, no ADXL345 detected ... Check your wiring!");
    while(1);
  };
  // accel.setRange(ADXL343_RANGE_16_G);
  // accel.setRange(ADXL343_RANGE_8_G);
  // accel.setRange(ADXL343_RANGE_4_G);
  accel.setRange(ADXL345_RANGE_2_G);
  acc_displayRange();
};
void acc_displayRange(void)
{
  Serial.print  ("Range:         +/- "); 
  
  switch(accel.getRange())
  {
    case ADXL345_RANGE_16_G:
      Serial.print  ("16 "); 
      break;
    case ADXL345_RANGE_8_G:
      Serial.print  ("8 "); 
      break;
    case ADXL345_RANGE_4_G:
      Serial.print  ("4 "); 
      break;
    case ADXL345_RANGE_2_G:
      Serial.print  ("2 "); 
      break;
    default:
      Serial.print  ("?? "); 
      break;
  }  
  Serial.println(" g");  
}

void gyro_init(){
  Serial.println("Gyroscope Test"); Serial.println("");
  
  /* Enable auto-ranging */
  gyro.enableAutoRange(true);
  
  /* Initialise the sensor */
  if(!gyro.begin())
  {
    /* There was a problem detecting the L3GD20 ... check your connections */
    Serial.println("Ooops, no L3GD20 detected ... Check your wiring!");
    while(1);
  };
};
void mag_init(){

};