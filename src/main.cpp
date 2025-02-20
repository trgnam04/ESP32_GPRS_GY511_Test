#include <TinyGPSPlus.h>
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <U8g2lib.h>
#include <ThingsBoard.h>
#include <Arduino_MQTT_Client.h>
#include <WiFi.h>
#include <Server_Side_RPC.h>
#include <rotary_encoder.h>

#define SERIAL_BAUDRATE 9600
#define TX_PIN 17
#define RX_PIN 16

// set up Wifi
constexpr char WIFI_SSID[] = "271104E";
constexpr char WIFI_PASSWORD[] = "1234567890";
constexpr char TOKEN[] = "COLLECTOR";

// set up Server
constexpr char THINGSBOARD_SERVER[] = "app.coreiot.io";
constexpr uint16_t THINGSBOARD_PORT = 1883U;

constexpr uint16_t MAX_MESSAGE_SEND_SIZE = 256U;
constexpr uint16_t MAX_MESSAGE_RECEIVE_SIZE = 256U;

constexpr uint8_t MAX_RPC_SUBSCRIPTIONS = 2U;
constexpr uint8_t MAX_RPC_RESPONSE = 2U;
// set up key
constexpr char COLLECTOR_KEY_TRIP_NUMBER[] = "Trip-Number";
constexpr char COLLECTOR_KEY_ROUTE_ID[] = "Route-ID";
constexpr char COLLECTOR_KEY_LAT[] = "lat";
constexpr char COLLECTOR_KEY_LNG[] = "lng";
constexpr char COLLECTOR_KEY_ACCEL_X[] = "accX";
constexpr char COLLECTOR_KEY_ACCEL_Y[] = "accY";
constexpr char COLLECTOR_KEY_ACCEL_Z[] = "accZ";
constexpr char COLLECTOR_KEY_MAG_X[] = "magX";
constexpr char COLLECTOR_KEY_MAG_Y[] = "magY";
constexpr char COLLECTOR_KEY_MAG_Z[] = "magZ";



// Khai báo màn hình OLED SH1106 (I2C)
U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);
Adafruit_LSM303_Mag_Unified mag = Adafruit_LSM303_Mag_Unified(54321);
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(12345);
// GPRS
HardwareSerial hardware(2);
TinyGPSPlus gps;
// Set up for Callback
Server_Side_RPC<MAX_RPC_SUBSCRIPTIONS, MAX_RPC_RESPONSE> rpc;
const std::array<IAPI_Implementation*, 1U> apis = {
    &rpc
};
// Thingsboard
WiFiClient espClient;
Arduino_MQTT_Client mqttClient(espClient);
ThingsBoard tb(mqttClient, MAX_MESSAGE_RECEIVE_SIZE, MAX_MESSAGE_SEND_SIZE, Default_Max_Stack_Size, apis);

// status for subcribing
bool subscribed = false;


#define SDA 21
#define SCK 22

// CONFIG
#undef LCD 
#define _SERIAL 1

#define FREQENCE 1
#define MQTT    1
#undef  HTTP



// Sensor
typedef struct{
    float Magx = 0;
    float Magy = 0;
    float Magz = 0;
    float Ax = 0;
    float Ay = 0;
    float Az = 0;
    double lat = 0;
    double lng = 0;    
    uint16_t trip_number = 0;
    uint16_t routeID = 0;
} sensor_data_t;

sensor_data_t Sensor_Data;


char buffer[100];
DynamicJsonDocument data(256);

size_t data_size;
uint32_t timestamp = 0;

xSemaphoreHandle xMutex;

// Supported for Display
void displayNum(float num, int8_t x, int8_t y);
void updateValues(void);
void displaySubPage2(void);
void drawArrowSelect(uint8_t state);
void drawStaticMenu(void);


// Supported for Wifi
void InitWiFi(void);
bool reconnect(void);

// Supported Task Function
void displaySensorDetails(void);
void setupLCD(void);
void setupMagSensor(void);
void convertData(void);

// Task define 
void Task_Display(void* pvParameters);
void Task_ReadSensor(void* pvParameters);
void Task_SendData(void* pvParameters);
void Task_CheckConnection(void* pvParameters);
void Task_MenuProcess(void* pvParameters);

// Display Page
void Menu(void);
void Page1(void); // Accel
void Page2(void); // Mag 
void Page3(void); // GPS

typedef enum{
    IDLE_MENU
} menu_state_t;

menu_state_t MenuState = IDLE_MENU;

typedef enum{
    IDLE,
    PAGE1,
    PAGE2,
    PAGE3
} display_state_t ;

display_state_t DisplayState = IDLE;
int tripNumber = 0;
int routeID = 0;



void setup() {
    Serial.begin(SERIAL_BAUDRATE);   
    xMutex = xSemaphoreCreateBinary();
    if(xMutex != NULL){
        xSemaphoreGive(xMutex);
    }
    Wire.begin(SDA, SCK);
    hardware.begin(9600);

    RotaryEncoder_setup();
    delay(1000);
    
    InitWiFi();
   

    // Tăng stack lên 4096 tránh lỗi reset
    xTaskCreatePinnedToCore(Task_ReadSensor, "Task_ReadSensor", 4096, NULL, 1, NULL, 1);
    // xTaskCreatePinnedToCore(Task_Display, "Task_Display", 4096, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(Task_SendData, "Task_SendData", 4096, NULL, 2, NULL, 0);
    xTaskCreatePinnedToCore(Task_CheckConnection, "Task_CheckConnection", 4096, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(Task_MenuProcess, "Task_MenuProcess", 4096, NULL, 2, NULL, 0);
}

void loop() {
    // Không làm gì trong loop vì đang chạy RTOS
}

/*----------------------------------------FUNC DEFINE--------------------------------------------------*/
void InitWiFi() {
    Serial.println("Connecting to AP ...");
    // Attempting to establish a connection to the given WiFi network
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    while (WiFi.status() != WL_CONNECTED) {
        // Delay 500ms until a connection has been successfully established
        delay(500);
        Serial.print(".");
    }
    Serial.println("Connected to AP");
}

bool reconnect() {
    // Check to ensure we aren't connected yet
    const wl_status_t status = WiFi.status();
    if (status == WL_CONNECTED) {
      return true;
    }
  
    // If we aren't establish a new connection to the given WiFi network
    InitWiFi();
    return true;
}
  

void convertData(void){
#ifdef HTTP
  snprintf(buffer, 100, "%.6f;%.6f;%.6f;%.6f;%.6f;%.6f;%.6f;%.6f", Magx, Magy, Magz, Ax, Ay, Az, lat, lng);
#endif
#ifdef MQTT    
    data[COLLECTOR_KEY_TRIP_NUMBER] = Sensor_Data.trip_number;
    data[COLLECTOR_KEY_ROUTE_ID] = Sensor_Data.routeID;
    data[COLLECTOR_KEY_LNG] = Sensor_Data.lng;
    data[COLLECTOR_KEY_LAT] = Sensor_Data.lat;
    data[COLLECTOR_KEY_MAG_X] = Sensor_Data.Magx;
    data[COLLECTOR_KEY_MAG_Y] = Sensor_Data.Magy;
    data[COLLECTOR_KEY_MAG_Z] = Sensor_Data.Magz;
    data[COLLECTOR_KEY_ACCEL_X] = Sensor_Data.Ax;
    data[COLLECTOR_KEY_ACCEL_Y] = Sensor_Data.Ay;
    data[COLLECTOR_KEY_ACCEL_Z] = Sensor_Data.Az;
    data_size = Helper::Measure_Json(data);    
#endif


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

void setupMagSensor(void)
{
    if(!mag.begin() || !accel.begin())
    {
        /* There was a problem detecting the ADXL345 ... check your connections */        
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
  vTaskDelay(500);
}


// Task hiển thị trên OLED
void Task_Display(void* pvParameters) {    
    TickType_t xLastWakeTime = xTaskGetTickCount();    
#ifdef LCD
    setupLCD();
    u8g2.setFont(u8g2_font_ncenB08_tr); 
    // u8g2.drawStr(10, 10, "ESP32 Test");
    u8g2.drawStr(10, 30, "X:");
    u8g2.drawStr(10, 45, "Y:");
    u8g2 .drawStr(10, 60, "Z:");
    u8g2.sendBuffer();        
#endif
    while (1) {                      
          
        if(xSemaphoreTake(xMutex, portMAX_DELAY)){            
            // displayNum(timestamp_Accel, 25, 10);
            // displayNum(timestamp_Mag, 65, 10);            
#ifdef LCD
            displayNum(Magx, 25, 30);
            displayNum(Magy, 25, 45);
            displayNum(Magz, 25, 60);
            displayNum(Ax, 65, 30);
            displayNum(Ay, 65, 45);
            displayNum(Az, 65, 60);            
            u8g2.sendBuffer();                
#endif
#ifdef _SERIAL        
#ifdef TIMESTAMP_DEBUG
            Serial.print(timestamp); Serial.print("| Accel__X: "); Serial.print(Ax);
            Serial.print(" Accel__Y: "); Serial.print(Ay); Serial.print(" Accel__Z: "); Serial.println(Az);        
            Serial.print(timestamp); Serial.print("| Mag__X: "); Serial.print(Magx);
            Serial.print(" Mag__Y: "); Serial.print(Magy); Serial.print(" Mag__Z: "); Serial.println(Magz);                
#else
            Serial.println(buffer);
#endif
#endif            
            xSemaphoreGive(xMutex);
        }           
        // Dùng pdMS_TO_TICKS() để delay đúng thời gian
        vTaskDelayUntil(&xLastWakeTime, 500);
    }
}


void drawStaticMenu(void){
    u8g2.clearBuffer();    
    u8g2.setFont(u8g2_font_6x12_tf);
    u8g2.drawStr((128 - u8g2.getStrWidth("Menu")) / 2, 12, "Menu");      
    u8g2.drawHLine(5, 16, 118);    
    u8g2.drawStr(20, 35, "Trip Number");
    u8g2.drawStr(20, 50, "Route ID");  
    u8g2.sendBuffer();
}

void drawArrowSelect(uint8_t state){  
  u8g2.setDrawColor(0); 
  u8g2.drawBox(3, 25, 10, 30);
  u8g2.setDrawColor(1);
  int arrowY = (state == 0) ? 50 : 35;
  u8g2.drawStr(5, arrowY, ">");
  u8g2.sendBuffer();
}

void displaySubPage2(void){    
    int rowHeight = 11;   // Chiều cao mỗi hàng (để cân đối)
    int paramWidth = 30;  // Chiều rộng cột "Param" (Nhỏ hơn)
    int valueWidth = 80;  // Chiều rộng cột "Value" (Lớn hơn)
    int startX = 5;       // Lề trái bảng
    int startY = 10;      // Lề trên bảng    
    for (int i = 0; i <= 5; i++) {
      u8g2.drawHLine(startX, startY + (i * rowHeight), paramWidth + valueWidth);
    }
      
    u8g2.drawVLine(startX + paramWidth, startY, rowHeight * 5);  
    u8g2.setFont(u8g2_font_6x10_tf);    
    u8g2.drawStr(startX + 5, startY - 2, "P");  // Cột 1 (Param nhỏ gọn)
    u8g2.drawStr(startX + paramWidth + 5, startY - 2, "Value");  // Cột 2
      
    const char* params[] = {"ax", "ay", "az", "lat", "lng"};
    const char* values[] = {"1.23", "-0.98", "0.50", "10.1234", "106.5678"};
  
    for (int i = 0; i < 5; i++) {
      u8g2.drawStr(startX + 5, startY + (i + 1) * rowHeight - 2, params[i]);
      u8g2.drawStr(startX + paramWidth + 5, startY + (i + 1) * rowHeight - 2, values[i]);
    }
      
    u8g2.sendBuffer();
}

void updateValues() {    
    u8g2.setDrawColor(0);
    u8g2.drawBox(100, 25, 30, 40);  // Xóa vùng giá trị số    
    u8g2.setDrawColor(1);
    char buffer[10];
    
    sprintf(buffer, "%d", tripNumber);
    u8g2.drawStr(100, 35, buffer);
  
    sprintf(buffer, "%d", routeID);
    u8g2.drawStr(100, 50, buffer);
  
    u8g2.sendBuffer();
}


void Task_ReadSensor(void* pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    setupMagSensor();
    sensors_event_t eventMag;
    sensors_event_t eventAccel;    
    timestamp = 0;
    while(1){                
        /* Display the results (acceleration is measured in m/s^2) */                
        if(xSemaphoreTake(xMutex, portMAX_DELAY)){
            accel.getEvent(&eventAccel);
            mag.getEvent(&eventMag);            
            
            Sensor_Data.trip_number = tripNumber;
            Sensor_Data.routeID = routeID;
            
            Sensor_Data.Magx = eventMag.magnetic.x;
            Sensor_Data.Magy = eventMag.magnetic.y;
            Sensor_Data.Magz = eventMag.magnetic.z;                        
            Sensor_Data.Ax = eventAccel.acceleration.x;
            Sensor_Data.Ay = eventAccel.acceleration.y;
            Sensor_Data.Az = eventAccel.acceleration.z;       
            
            if(hardware.available() > 0){             
              gps.encode(hardware.read());              
              Sensor_Data.lat = gps.location.lat();
              Sensor_Data.lng = gps.location.lng();
            }

            
            convertData();
            timestamp = millis();
            xSemaphoreGive(xMutex);            
        }                    
        vTaskDelayUntil(&xLastWakeTime, 500);
        /* Delay before the next sample */        
    }
}

void Task_SendData(void* pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    vTaskDelay(1000);
    while(1){
        tb.sendTelemetryJson(data, data_size);
        vTaskDelayUntil(&xLastWakeTime, 1000);
    }
}

void Task_CheckConnection(void* pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    while(1){
        if (!tb.connected()) {            
            // Reconnect to the ThingsBoard server,
            // if a connection was disrupted or has not yet been established
            Serial.printf("Connecting to: (%s) with token (%s)\n", THINGSBOARD_SERVER, TOKEN);
            if (!tb.connect(THINGSBOARD_SERVER, TOKEN, THINGSBOARD_PORT)) {                          
                Serial.println("Failed to connect");                
            }
            else{
                Serial.println("Connected");
            }
        }                
        
        tb.loop();
        vTaskDelayUntil(&xLastWakeTime, 1000);
    }
}

void Task_MenuProcess(void* pvParameters)
{
    while(1){
        RotaryEncoder_loop();

        vTaskDelay(5);
    }
}


