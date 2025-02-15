#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <U8g2lib.h>

#define SERIAL_BAUDRATE 115200

// Khai báo màn hình OLED SH1106 (I2C)
U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);
Adafruit_LSM303_Mag_Unified mag = Adafruit_LSM303_Mag_Unified(54321);

#define SDA 21
#define SCK 22

float x = 0;
float y = 0;
float z = 0;

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
    // Serial.begin(SERIAL_BAUDRATE);   
    Wire.begin(SDA, SCK);
   

    // Tăng stack lên 4096 tránh lỗi reset
    xTaskCreatePinnedToCore(Task_testMagSensor, "Task_Test_Mag_Sensor", 10000, NULL, 1, NULL, 1);
    xTaskCreatePinnedToCore(Task_testLCD, "Task_Test_LCD", 10000, NULL, 2, NULL, 1);
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
    if(!mag.begin())
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
//   Serial.println("------------------------------------");
//   Serial.print  ("Sensor:       "); Serial.println(sensor.name);
//   Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
//   Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
//   Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" m/s^2");
//   Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" m/s^2");
//   Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" m/s^2");
//   Serial.println("------------------------------------");
//   Serial.println("");
  vTaskDelay(500);
}


// Task hiển thị trên OLED
void Task_testLCD(void* pvParameters) {
    setupLCD();
    u8g2.setFont(u8g2_font_ncenB08_tr); 
    while (1) {                
        u8g2.drawStr(10, 10, "ESP32 Test");

        displayNum(x, 10, 30);
        displayNum(y, 10, 45);
        displayNum(z, 10, 60);
        u8g2.sendBuffer();       

        
        // Dùng pdMS_TO_TICKS() để delay đúng thời gian
        vTaskDelay(100);
    }
}

void Task_testMagSensor(void* pvParameters)
{
    setupMagSensor();
    while(1){
        sensors_event_t event;
        mag.getEvent(&event);

        /* Display the results (acceleration is measured in m/s^2) */
        //   Serial.print("X: "); Serial.print(event.acceleration.x); Serial.print("  ");
        //   Serial.print("Y: "); Serial.print(event.acceleration.y); Serial.print("  ");
        //   Serial.print("Z: "); Serial.print(event.acceleration.z); Serial.print("  ");Serial.println("m/s^2 ");
        x = event.magnetic.x;
        y = event.magnetic.y;
        z = event.magnetic.z;
        // Serial.print("__X: "); Serial.print(x); Serial.print("  ");
        // Serial.print("__Y: "); Serial.print(y); Serial.print("  ");
        // Serial.print("__Z: "); Serial.print(z); Serial.print("  ");Serial.println("Gauss ");
        /* Delay before the next sample */
        vTaskDelay(500);
    }
}
