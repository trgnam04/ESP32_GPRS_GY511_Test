#include <Arduino.h>
#include <Wire.h>
#include <U8g2lib.h>

#define SERIAL_BAUDRATE 115200

// Khai báo màn hình OLED SH1106 (I2C)
U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);

#define OLED_SDA 21
#define OLED_SCK 22

void setupLCD(void);
void Task_testLCD(void* pvParameters);

void setup() {
    Serial.begin(SERIAL_BAUDRATE);
    setupLCD();

    // Tăng stack lên 4096 tránh lỗi reset
    xTaskCreate(Task_testLCD, "Task Test LCD", 4096, NULL, 2, NULL);
}

void loop() {
    // Không làm gì trong loop vì đang chạy RTOS
}

// Hàm khởi tạo OLED
void setupLCD(void) {
    Wire.begin(OLED_SDA, OLED_SCK);
    u8g2.begin();
}

// Task hiển thị trên OLED
void Task_testLCD(void* pvParameters) {
    while (1) {
        u8g2.clearBuffer(); 
        u8g2.setFont(u8g2_font_ncenB08_tr); 
        u8g2.drawStr(10, 20, "Hello ESP32!"); 
        u8g2.sendBuffer();           

        // Dùng pdMS_TO_TICKS() để delay đúng thời gian
        vTaskDelay(1000);
        u8g2.clearDisplay();
        vTaskDelay(1000);
    }
}
