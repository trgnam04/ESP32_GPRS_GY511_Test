#include <Arduino.h>
#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>

static const int RXPin = 17, TXPin = 18;
static const uint32_t GPSBaud = 9600;

// Đối tượng GPS
TinyGPSPlus gps;
HardwareSerial ss(2);

// Queue để lưu trữ dữ liệu GPS
QueueHandle_t gpsQueue;

void gpsTask(void *parameter)
{
    while (1)
    {
        while (ss.available())
        {
            gps.encode(ss.read()); // Giải mã dữ liệu từ GPS
        }
        vTaskDelay(pdMS_TO_TICKS(10)); // Giảm tải CPU, không chạy liên tục
    }
}

void processGpsTask(void *parameter)
{
    while (1)
    {
        if (gps.location.isUpdated()) // Nếu có dữ liệu mới
        {
            Serial.print("Latitude: ");
            Serial.print(gps.location.lat(), 6);
            Serial.print(", Longitude: ");
            Serial.println(gps.location.lng(), 6);
        }
        vTaskDelay(pdMS_TO_TICKS(1000)); // Mỗi giây xử lý một lần
    }
}

void setup()
{
    Serial.begin(9600);
    ss.begin(GPSBaud);

    Serial.println("GPS RTOS Example");

    // Tạo task đọc GPS
    xTaskCreate(gpsTask, "GPS Reader", 2048, NULL, 1, NULL);
    xTaskCreate(processGpsTask, "GPS Processor", 2048, NULL, 1, NULL);
}

void loop()
{
    // Không dùng loop() trong FreeRTOS
    vTaskDelay(pdMS_TO_TICKS(1000));
}
