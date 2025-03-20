#define CORE_DEBUG_LEVEL 5

#include <TinyGPSPlus.h>
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <U8g2lib.h>
#include <ThingsBoard.h>
#include <Arduino_MQTT_Client.h>
#include <WiFi.h>
#include <Adafruit_BNO055.h>

// Supported library
#include <rotary_encoder.h>
#include <compass.h>
#include <config.h>

SemaphoreHandle_t xI2CSemaphore;
QueueHandle_t ServerDataQueue;

// set up Server
constexpr char THINGSBOARD_SERVER[] = "app.coreiot.io";
constexpr uint16_t THINGSBOARD_PORT = 1883U;

constexpr uint16_t MAX_MESSAGE_SEND_SIZE = 512U;
constexpr uint16_t MAX_MESSAGE_RECEIVE_SIZE = 256U;

constexpr uint8_t MAX_RPC_SUBSCRIPTIONS = 2U;
constexpr uint8_t MAX_RPC_RESPONSE = 2U;

// Khai báo màn hình OLED SH1106 (I2C)
U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/U8X8_PIN_NONE);
// BNO055
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);
// GPRS
HardwareSerial hardware(2);
TinyGPSPlus gps;
// Thingsboard
WiFiClient espClient;
Arduino_MQTT_Client mqttClient(espClient);
ThingsBoard tb(mqttClient, MAX_MESSAGE_RECEIVE_SIZE, MAX_MESSAGE_SEND_SIZE, Default_Max_Stack_Size);

// Sensor
typedef struct
{
    float longtitude = 0.0;
    float latitude = 0.0;
} gps_data_t;

typedef struct
{
    float Magx = 0;
    float Magy = 0;
    float Magz = 0;
    float Gx = 0;
    float Gy = 0;
    float Gz = 0;
    double lat = 0;
    double lng = 0;
    float AccelxLinear = 0;
    float AccelyLinear = 0;
    float AccelzLinear = 0;
} sensor_data_t;

typedef struct
{
    sensor_data_t SensorData;
    gps_data_t GPSData;
    float velocityN = 0;
    float velocityE = 0;
    float deltaT = 0;
    uint16_t trip_number = 0;
    uint16_t routeID = 0;
    uint16_t stationID = 0;
} obj_data_t;

obj_data_t ObjData;

char buffer[100];
DynamicJsonDocument data(256);

size_t data_size;

// Supported for Display
void displayMessage(const char *message);
void displayCalibrationStatus();
void displayNum(float num, int8_t x, int8_t y);
void updateValues(void);
void displaySubPage(void);
void drawArrowSelect(uint8_t state);
void drawStaticMenu(void);
void updateValues(void);

void display_process_fsm(void);
void menu_process_fsm(void);
void setting_process_fsm(void);

// Supported for Wifi
void InitWiFi(void);
bool reconnect(void);

// Supported Task Function
void setupMagSensor(void);
void convertData(void);

// Task define
void Task_ReadSensor(void *pvParameters);
void Task_SendData(void *pvParameters);
void Task_CheckConnection(void *pvParameters);
void Task_MenuProcess(void *pvParameters);
void Task_ReadRotaryEncoder(void *pvParameters);
void Task_ReadGPS(void *pvParameters);
void Task_Debug(void *pvParameters);

typedef enum
{
    STATE_IDLE_MENU,
    STATE_SETTING,
    STATE_MEASURING
} menu_state_t;

typedef enum
{
    STATE_IDLE_DISPLAY,
    STATE_DISPLAY_PAGE1,
    STATE_DISPLAY_PAGE2
} display_state_t;

typedef enum
{
    STATE_IDLE_SETTING,
    STATE_TRIPNUMBER_SETTING,
    STATE_ROUTEID_SETTING
} setting_state_t;

setting_state_t SettingState = STATE_IDLE_SETTING;
display_state_t DisplayState = STATE_IDLE_DISPLAY;
menu_state_t MenuState = STATE_IDLE_MENU;

volatile uint16_t tripNumber = 0;
volatile uint16_t routeID = 0;
uint16_t screen_tick = 0;
uint16_t display_tick = 50; // switch screen every 5s

TaskHandle_t TaskHandle_ReadSensor;
TaskHandle_t TaskHandle_SendData;
TaskHandle_t TaskHandle_CheckConnection;
TaskHandle_t TaskHandle_MenuProcess;
TaskHandle_t TaskHandle_ReadRotary;
TaskHandle_t TaskHandle_ReadGPS;

sensors_event_t eventMag;
sensors_event_t eventGyro;
sensors_event_t eventTemp;
sensors_event_t eventAccel;

EventGroupHandle_t eventGroup;
const int MEASURE_BIT = (1 << 0);

void setup()
{
    Serial.begin(SERIAL_BAUDRATE);
    hardware.begin(9600, SERIAL_8N1, RX_PIN, TX_PIN);
    Wire.begin(SDA, SCK);
    u8g2.begin();

    // Synchronous
    eventGroup = xEventGroupCreate(); // Create event group
    ServerDataQueue = xQueueCreate(5, sizeof(sensor_data_t));
    xI2CSemaphore = xSemaphoreCreateMutex();

    if (!bno.begin())
    {
        Serial.println("Can't find BNO055, Check the connection !!");
        while (1)
            ;
    }

    bno.setExtCrystalUse(true);

    // Hiển thị trạng thái hiệu chỉnh
    while (true)
    {
        uint8_t sys, gyro, accel, mag;
        bno.getCalibration(&sys, &gyro, &accel, &mag);
        displayCalibrationStatus();

        if (sys == 3 && gyro == 3 && accel == 3 && mag == 3)
        {
            break;
        }
        delay(500);
    }

    displayMessage("Calibration Done!");

    delay(1000);

    // InitWiFi();

    // Tăng stack lên 4096 tránh lỗi reset
    xTaskCreatePinnedToCore(Task_ReadSensor, "Task_ReadSensor", 1024 * 5, NULL, 1, &TaskHandle_ReadSensor, 1);
    // Chờ cho đến khi quá trình measuring được gọi, mới bắt đầu được thực thi
    // vTaskSuspend(TaskHandle_ReadSensor);

    xTaskCreatePinnedToCore(Task_CheckConnection, "Task_CheckConnection", 1024 * 2, NULL, 2, &TaskHandle_CheckConnection, 0);
    // vTaskSuspend(TaskHandle_CheckConnection);

    xTaskCreatePinnedToCore(Task_SendData, "Task_SendData", 1024 * 3, NULL, 3, &TaskHandle_SendData, 0);
    // vTaskSuspend(TaskHandle_SendData);

    xTaskCreatePinnedToCore(Task_ReadGPS, "Task_ReadGPS", 1024, NULL, 1, &TaskHandle_ReadGPS, 1);

    xTaskCreatePinnedToCore(Task_MenuProcess, "Task_MenuProcess", 1024 * 2, NULL, 1, &TaskHandle_MenuProcess, 1);
    xTaskCreatePinnedToCore(Task_ReadRotaryEncoder, "Task_ReadRotary", 1024, NULL, 2, &TaskHandle_ReadRotary, 1);
}

void loop()
{
    // Không làm gì trong loop vì đang chạy RTOS
    // vTaskDelay(portMAX_DELAY);
}

/*----------------------------------------FUNC DEFINE--------------------------------------------------*/
void InitWiFi()
{
    Serial.println("Connecting to AP ...");
    // Attempting to establish a connection to the given WiFi network
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    while (WiFi.status() != WL_CONNECTED)
    {
        // Delay 500ms until a connection has been successfully established
        delay(500);
        Serial.print(".");
    }
    Serial.println("Connected to AP");
}

bool reconnect()
{
    // Check to ensure we aren't connected yet
    const wl_status_t status = WiFi.status();
    if (status == WL_CONNECTED)
    {
        return true;
    }

    // If we aren't establish a new connection to the given WiFi network
    InitWiFi();
    return true;
}

void convertData(void)
{
#ifdef HTTP
    snprintf(buffer, 100, "%.6f;%.6f;%.6f;%.6f;%.6f;%.6f;%.6f;%.6f", Sensor_Data.Magx, Sensor_Data.Magy, Sensor_Data.Magz, Sensor_Data.Ax,
             Sensor_Data.Ay, Sensor_Data.Az, Sensor_Data.lat, Sensor_Data.lng);
#endif
#ifdef MQTT
    data[COLLECTOR_KEY_TRIP_NUMBER] = ObjData.trip_number;
    data[COLLECTOR_KEY_ROUTE_ID] = ObjData.routeID;
    data[COLLECTOR_KEY_STATION_ID] = ObjData.stationID;
    data[COLLECTOR_KEY_LNG] = ObjData.GPSData.longtitude;
    data[COLLECTOR_KEY_LAT] = ObjData.GPSData.latitude;
    data[COLLECTOR_KEY_ACCEL_X] = ObjData.SensorData.AccelxLinear;
    data[COLLECTOR_KEY_ACCEL_Y] = ObjData.SensorData.AccelyLinear;
    data[COLLECTOR_KEY_ACCEL_Z] = ObjData.SensorData.AccelzLinear;

    data_size = Helper::Measure_Json(data);

    snprintf(buffer, 128, "%.6f\t%.6f\t%.6f\t%.6f\t%.6f",
             ObjData.SensorData.AccelxLinear, ObjData.SensorData.AccelyLinear, ObjData.SensorData.AccelzLinear,
             ObjData.GPSData.latitude, ObjData.GPSData.longtitude);
#endif
}

void displayMessage(const char *message)
{
    u8g2.firstPage();
    do
    {
        u8g2.setFont(u8g2_font_6x12_tf);
        u8g2.setCursor(20, 40);
        u8g2.print(message);
    } while (u8g2.nextPage());
}

void displayCalibrationStatus()
{
    uint8_t sys, gyro, accel, mag;
    bno.getCalibration(&sys, &gyro, &accel, &mag);

    u8g2.firstPage();
    do
    {
        u8g2.setFont(u8g2_font_6x12_tf);
        u8g2.setCursor(25, 20);
        u8g2.print("Calibrating...");
        u8g2.setCursor(10, 30);
        u8g2.print("Sys: ");
        u8g2.print(sys);
        u8g2.setCursor(10, 40);
        u8g2.print("Gyro: ");
        u8g2.print(gyro);
        u8g2.setCursor(10, 50);
        u8g2.print("Accel: ");
        u8g2.print(accel);
        u8g2.setCursor(10, 60);
        u8g2.print("Mag: ");
        u8g2.print(mag);
    } while (u8g2.nextPage());
}

void displayNum(float num, int8_t x, int8_t y)
{
    char buffer[10];
    dtostrf(num, 6, 2, buffer);

    u8g2.setDrawColor(0);
    u8g2.drawBox(x, y - 10, 50, 12);

    u8g2.setDrawColor(1);
    u8g2.drawStr(x, y, buffer);
}

void drawStaticMenu(void)
{
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_6x12_tf);
    u8g2.drawStr((128 - u8g2.getStrWidth("Menu")) / 2, 12, "Menu");
    u8g2.drawHLine(5, 16, 118);
    u8g2.drawStr(20, 35, "Trip Number");
    u8g2.drawStr(20, 50, "Route ID");
    u8g2.sendBuffer();
}

void drawArrowSelect(uint8_t state)
{
    u8g2.setDrawColor(0);
    u8g2.drawBox(3, 25, 10, 30);
    u8g2.setDrawColor(1);
    int arrowY = (state == 0) ? 50 : 35;
    u8g2.drawStr(5, arrowY, ">");
    u8g2.sendBuffer();
}

void displaySubPage(uint8_t idx)
{
    int rowHeight = 11;  // Chiều cao mỗi hàng (để cân đối)
    int paramWidth = 30; // Chiều rộng cột "Param" (Nhỏ hơn)
    int valueWidth = 80; // Chiều rộng cột "Value" (Lớn hơn)
    int startX = 5;      // Lề trái bảng
    int startY = 10;     // Lề trên bảng
    for (int i = 0; i <= 5; i++)
    {
        u8g2.drawHLine(startX, startY + (i * rowHeight), paramWidth + valueWidth);
    }

    u8g2.drawVLine(startX + paramWidth, startY, rowHeight * 5);
    u8g2.setFont(u8g2_font_6x10_tf);
    u8g2.drawStr(startX + 5, startY - 2, "P");                  // Cột 1 (Param nhỏ gọn)
    u8g2.drawStr(startX + paramWidth + 5, startY - 2, "Value"); // Cột 2
    const char *params[5];
    char values[5][10];
    switch (idx)
    {
    case 1:
    {
        params[0] = "ax";
        params[1] = "ay";
        params[2] = "az";
        params[3] = "tID";
        params[4] = "rID";
        dtostrf(ObjData.SensorData.AccelxLinear, 6, 2, values[0]);
        dtostrf(ObjData.SensorData.AccelyLinear, 6, 2, values[1]);
        dtostrf(ObjData.SensorData.AccelzLinear, 6, 2, values[2]);
        sprintf(values[3], "%d", ObjData.trip_number);
        sprintf(values[4], "%d", ObjData.routeID);
        break;
    }
    case 2:
    {
        params[0] = "mx";
        params[1] = "my";
        params[2] = "mz";
        params[3] = "lat";
        params[4] = "lng";
        dtostrf(ObjData.SensorData.Magx, 6, 2, values[0]);
        dtostrf(ObjData.SensorData.Magy, 6, 2, values[1]);
        dtostrf(ObjData.SensorData.Magz, 6, 2, values[2]);
        dtostrf(ObjData.GPSData.latitude, 6, 2, values[3]);
        dtostrf(ObjData.GPSData.longtitude, 6, 2, values[4]);
        break;
    }
    }

    for (int i = 0; i < 5; i++)
    {
        u8g2.drawStr(startX + 5, startY + (i + 1) * rowHeight - 2, params[i]);
        u8g2.drawStr(startX + paramWidth + 5, startY + (i + 1) * rowHeight - 2, values[i]);
    }

    u8g2.sendBuffer();
}

void updateValues()
{
    u8g2.setDrawColor(0);
    u8g2.drawBox(100, 25, 30, 40); // Xóa vùng giá trị số
    u8g2.setDrawColor(1);
    char buffer[10];

    sprintf(buffer, "%d", tripNumber);
    u8g2.drawStr(100, 35, buffer);

    sprintf(buffer, "%d", routeID);
    u8g2.drawStr(100, 50, buffer);

    u8g2.sendBuffer();
}

void flushData(void)
{
    tripNumber = 0;
    routeID = 0;
    u8g2.clearBuffer();
}

void setting_process_fsm(void)
{
    switch (SettingState)
    {
    case STATE_IDLE_SETTING:
    {
        if (1)
        {
            drawArrowSelect(1);
            SettingState = STATE_TRIPNUMBER_SETTING;
        }
        break;
    }
    case STATE_TRIPNUMBER_SETTING:
    {
        if (isDecrease())
        {
            if (tripNumber > 0)
            {
                tripNumber--;
            }
        }
        if (isIncrease())
        {
            tripNumber++;
        }
        if (isPressed())
        {
            drawArrowSelect(0);
            SettingState = STATE_ROUTEID_SETTING;
        }
        break;
    }
    case STATE_ROUTEID_SETTING:
    {
        if (isDecrease())
        {
            if (routeID > 0)
            {
                routeID--;
            }
        }
        if (isIncrease())
        {
            routeID++;
        }
        if (isPressed())
        {
            drawArrowSelect(1);
            SettingState = STATE_IDLE_SETTING;
        }
        break;
    }
    }
    updateValues();
}

void menu_process_fsm(void)
{
    switch (MenuState)
    {
    case STATE_IDLE_MENU:
    {
        if (1)
        {
            drawStaticMenu();
            MenuState = STATE_SETTING;
        }
        break;
    }
    case STATE_SETTING:
    {
        setting_process_fsm();
        if (isLongPressed())
        {
            SettingState = STATE_IDLE_SETTING;
            resetRotaryEncoder();
            u8g2.clearBuffer();
            xEventGroupSetBits(eventGroup, MEASURE_BIT);
            MenuState = STATE_MEASURING;
        }
        break;
    }
    case STATE_MEASURING:
    {
        if (isLongPressed())
        {
            flushData();
            resetRotaryEncoder();
            xEventGroupClearBits(eventGroup, MEASURE_BIT);
            MenuState = STATE_IDLE_MENU;
        }
        if (xSemaphoreTake(xI2CSemaphore, portMAX_DELAY))
        {
            display_process_fsm();
            xSemaphoreGive(xI2CSemaphore);
        }
        break;
    }
    }
}

void display_process_fsm(void)
{
    screen_tick = (screen_tick + 1) % display_tick;
    switch (DisplayState)
    {
    case STATE_IDLE_DISPLAY:
    {
        if (1)
        {
            DisplayState = STATE_DISPLAY_PAGE1;
        }
        break;
    }
    case STATE_DISPLAY_PAGE1:
    {
        displaySubPage(1);
        if (!screen_tick)
        {
            u8g2.clearBuffer();
            DisplayState = STATE_DISPLAY_PAGE2;
        }
        break;
    }
    case STATE_DISPLAY_PAGE2:
    {
        displaySubPage(2);
        if (!screen_tick)
        {
            u8g2.clearBuffer();
            DisplayState = STATE_DISPLAY_PAGE1;
        }
        break;
    }
    }
}

/*---------------------------------------------- Define task -------------------------------------------------*/

void Task_ReadSensor(void *pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();    

    ObjData.deltaT = 100 / 1000.0;
    

    uint8_t flag = 0;
    for (;;)
    {
        xEventGroupWaitBits(eventGroup, MEASURE_BIT, pdFALSE, pdTRUE, portMAX_DELAY);
        while (1)
        {
            if (xSemaphoreTake(xI2CSemaphore, portMAX_DELAY))
            {
                ObjData.trip_number = tripNumber;
                ObjData.routeID = routeID;

                // Mô phỏng tạm tín hiệu điểm ground truth
                if (flag)
                {
                    ObjData.stationID = 1;
                }
                else
                {
                    ObjData.stationID = 0;
                }

                if (isPressed())
                {
                    flag = (flag + 1) % 2;
                }

                if (isLongPressed())
                {
                    flag = 0;
                }

                if (gps.location.isUpdated())
                {
                    ObjData.GPSData.latitude = gps.location.lat();
                    ObjData.GPSData.longtitude = gps.location.lng();
                }

                convertData();
                Serial.println(buffer);
                // xQueueSendToBack(ServerDataQueue, &Sensor_Data, portMAX_DELAY);
                xSemaphoreGive(xI2CSemaphore);

                if (!(xEventGroupGetBits(eventGroup) & MEASURE_BIT))
                {
                    break; // Stop measuring
                }
                // Kalman Filter here
            }

            vTaskDelay(100);
            /* Delay before the next sample */
        }
    }
}

void Task_SendData(void *pvParameters)
{
    sensor_data_t ReceivedData;
    while (1)
    {
        xQueueReceive(ServerDataQueue, &ReceivedData, portMAX_DELAY);
        tb.sendTelemetryJson(data, data_size);
        vTaskDelay(10);
    }
}

void Task_CheckConnection(void *pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    while (1)
    {
        if (!tb.connected())
        {
            Serial.printf("Connecting to: (%s) with token (%s)\n", THINGSBOARD_SERVER, TOKEN);
            if (!tb.connect(THINGSBOARD_SERVER, TOKEN, THINGSBOARD_PORT))
            {
                Serial.println("Failed to connect");
            }
            else
            {
                Serial.println("Connected");
            }
        }

        tb.loop();
        vTaskDelayUntil(&xLastWakeTime, 1000);
    }
}

void Task_MenuProcess(void *pvParameters)
{
    drawStaticMenu();
    while (1)
    {
        menu_process_fsm();
        vTaskDelay(100);
    }
}

void Task_ReadRotaryEncoder(void *pvParameters)
{
    RotaryEncoder_setup();
    while (1)
    {
        // menu_process_fsm();
        RotaryEncoder_loop();
        vTaskDelay(TIME_READ);
    }
}

void Task_ReadGPS(void *pvParameters)
{
    while (1)
    {
        while (hardware.available())
        {
            gps.encode(hardware.read());
        }
        vTaskDelay(10);
    }
}