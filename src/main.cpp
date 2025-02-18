#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <U8g2lib.h>

#define SERIAL_BAUDRATE 115200

// Khai báo màn hình OLED SH1106 (I2C)
U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);
Adafruit_LSM303_Mag_Unified mag = Adafruit_LSM303_Mag_Unified(54321);
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(12345);

#define SDA 21
#define SCK 22

float Magx = 0;
float Magy = 0;
float Magz = 0;

float Ax = 0;
float Ay = 0;
float Az = 0;


xSemaphoreHandle xMutex;

// Supported for Display
void displayNum(float num, int8_t x, int8_t y);

// Supported Task Function
void displaySensorDetails(void);
void setupLCD(void);
void setupMagSensor(void);


// Task define 
void Task_testLCD(void* pvParameters);
void Task_testMagSensor(void* pvParameters);

void setup() {
    Serial.begin(SERIAL_BAUDRATE);   
    xMutex = xSemaphoreCreateBinary();
    if(xMutex != NULL){
        xSemaphoreGive(xMutex);
    }
    Wire.begin(SDA, SCK);
   

    // Tăng stack lên 4096 tránh lỗi reset
    xTaskCreatePinnedToCore(Task_testMagSensor, "Task_Test_Mag_Sensor", 10000, NULL, 1, NULL, 1);
    xTaskCreatePinnedToCore(Task_testLCD, "Task_Test_LCD", 10000, NULL, 2, NULL, 0);
}

void loop() {
    // Không làm gì trong loop vì đang chạy RTOS
}

/*----------------------------------------FUNC DEFINE--------------------------------------------------*/
void displayNum(float num, int8_t x, int8_t y) 
{
    char buffer[10];  
    dtostrf(num, 6, 2, buffer);  

    
    u8g2.setDrawColor(0);
    u8g2.drawBox(x, y - 10, 50, 12);

    
    u8g2.setDrawColor(1);
    u8g2.drawStr(x, y, buffer);
}

void setupMagSensor(void)
{
    if(!mag.begin() || !accel.begin())
    {
        /* There was a problem detecting the ADXL345 ... check your connections */
        // Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
        while(1);
    }

    /* Display some basic information on this sensor */
    displaySensorDetails();

}

// Hàm khởi tạo OLED
void setupLCD(void) 
{   
    u8g2.begin();
}

void displaySensorDetails(void)
{
  sensor_t sensor;
  mag.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" m/s^2");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" m/s^2");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" m/s^2");
  Serial.println("------------------------------------");
  Serial.println("");
  vTaskDelay(500);
}


// Task hiển thị trên OLED
void Task_testLCD(void* pvParameters) {
    setupLCD();
    u8g2.setFont(u8g2_font_ncenB08_tr); 
    u8g2.drawStr(10, 10, "ESP32 Test");
    u8g2.drawStr(10, 30, "X:");
    u8g2.drawStr(10, 45, "Y:");
    u8g2.drawStr(10, 60, "Z:");
    u8g2.sendBuffer();        
    while (1) {                        
        if(xSemaphoreTake(xMutex, portMAX_DELAY)){            
            displayNum(Magx, 25, 30);
            displayNum(Magy, 25, 45);
            displayNum(Magz, 25, 60);
            displayNum(Ax, 65, 30);
            displayNum(Ay, 65, 45);
            displayNum(Az, 65, 60);            
            u8g2.sendBuffer();                
            xSemaphoreGive(xMutex);
        }           
        // Dùng pdMS_TO_TICKS() để delay đúng thời gian
        vTaskDelay(100);
    }
}

void Task_testMagSensor(void* pvParameters)
{
    setupMagSensor();
    sensors_event_t eventMag;
    sensors_event_t eventAccel;

    while(1){                
        /* Display the results (acceleration is measured in m/s^2) */
        //   Serial.print("X: "); Serial.print(event.acceleration.x); Serial.print("  ");
        //   Serial.print("Y: "); Serial.print(event.acceleration.y); Serial.print("  ");
        //   Serial.print("Z: "); Serial.print(event.acceleration.z); Serial.print("  ");Serial.println("m/s^2 ");
        if(xSemaphoreTake(xMutex, portMAX_DELAY)){
            accel.getEvent(&eventAccel);
            mag.getEvent(&eventMag);
            
            Magx = eventMag.magnetic.x;
            Magy = eventMag.magnetic.y;
            Magz = eventMag.magnetic.z;                        
            Ax = eventAccel.acceleration.x;
            Ay = eventAccel.acceleration.y;
            Az = eventAccel.acceleration.z;

            xSemaphoreGive(xMutex);
        }        
        // Serial.print("__X: "); Serial.print(x); Serial.print("  ");
        // Serial.print("__Y: "); Serial.print(y); Serial.print("  ");
        // Serial.print("__Z: "); Serial.print(z); Serial.print("  ");Serial.println("Gauss ");
        /* Delay before the next sample */
        vTaskDelay(100);
    }
}
