// #define CORE_DEBUG_LEVEL 5

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
#define SDA 21
#define SCK 22
#define MQTT    1
#undef  HTTP

SemaphoreHandle_t xI2CSemaphore;


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
volatile bool isSending = false;

// Supported for Display
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
void Task_ReadSensor(void* pvParameters);
void Task_SendData(void* pvParameters);
void Task_CheckConnection(void* pvParameters);
void Task_MenuProcess(void* pvParameters);
void Task_ReadRotaryEncoder(void* pvParameters);
void Task_ReadGPS(void* pvParameters);


typedef enum{
    STATE_IDLE_MENU,
    STATE_SETTING,
    STATE_MEASURING
} menu_state_t;

typedef enum{
    STATE_IDLE_DISPLAY,
    STATE_DISPLAY_PAGE1,
    STATE_DISPLAY_PAGE2    
} display_state_t ;

typedef enum{
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
uint16_t display_tick = 250; // switch screen every 5s

TaskHandle_t TaskHandle_ReadSensor;
TaskHandle_t TaskHandle_SendData;
TaskHandle_t TaskHandle_CheckConnection;
TaskHandle_t TaskHandle_MenuProcess;
TaskHandle_t TaskHandle_ReadRotary;
TaskHandle_t TaskHandle_ReadGPS;


void setup() {
    Serial.begin(SERIAL_BAUDRATE);       
    delay(100);
    hardware.begin(9600);    
    delay(100);
    Wire.begin(SDA, SCK);    
    delay(1000);
    
    // InitWiFi();

    xI2CSemaphore = xSemaphoreCreateMutex();    
   

    // Tăng stack lên 4096 tránh lỗi reset
    xTaskCreatePinnedToCore(Task_ReadSensor, "Task_ReadSensor", 4096, NULL, 1, &TaskHandle_ReadSensor, 0);
    vTaskSuspend(TaskHandle_ReadSensor);

    // xTaskCreatePinnedToCore(Task_CheckConnection, "Task_CheckConnection", 2048, NULL, 2, &TaskHandle_CheckConnection, 0);
    // vTaskSuspend(TaskHandle_CheckConnection);

    // xTaskCreatePinnedToCore(Task_SendData, "Task_SendData", 2048, NULL, 3, &TaskHandle_SendData, 0);    
    // vTaskSuspend(TaskHandle_SendData);

    xTaskCreatePinnedToCore(Task_ReadGPS, "Task_ReadGPS", 1024, NULL, 4, &TaskHandle_ReadGPS, 0);
    vTaskSuspend(TaskHandle_ReadGPS);
    
    xTaskCreatePinnedToCore(Task_MenuProcess, "Task_MenuProcess", 2048, NULL, 1, &TaskHandle_MenuProcess, 1);    
    xTaskCreatePinnedToCore(Task_ReadRotaryEncoder, "Task_ReadRotary", 1024, NULL, 2, &TaskHandle_ReadRotary, 1);
    
    delay(5000);
    vTaskResume(TaskHandle_ReadSensor);
    vTaskResume(TaskHandle_CheckConnection);
    vTaskResume(TaskHandle_SendData);
    vTaskResume(TaskHandle_ReadGPS);
}

void loop() {
    // Không làm gì trong loop vì đang chạy RTOS
    // vTaskDelay(portMAX_DELAY);
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

void displaySubPage(uint8_t idx){    
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
    const char* params[5];
    char values[5][10];
    switch(idx){
        case 1:{
            params[0] = "ax";
            params[1] = "ay";
            params[2] = "az";        
            params[3] = "tID";
            params[4] = "rID";
            dtostrf(Sensor_Data.Ax, 6, 2, values[0]);
            dtostrf(Sensor_Data.Ay, 6, 2, values[1]);
            dtostrf(Sensor_Data.Az, 6, 2, values[2]);
            sprintf(values[3], "%d", Sensor_Data.trip_number);
            sprintf(values[4], "%d", Sensor_Data.routeID);
            break;
        }
        case 2:{
            params[0] = "mx";
            params[1] = "my";
            params[2] = "mz";        
            params[3] = "lat";
            params[4] = "lng";
            dtostrf(Sensor_Data.Magx, 6, 2, values[0]);
            dtostrf(Sensor_Data.Magy, 6, 2, values[1]);
            dtostrf(Sensor_Data.Magz, 6, 2, values[2]);
            dtostrf(Sensor_Data.lat, 6, 2, values[3]);
            dtostrf(Sensor_Data.lng, 6, 2, values[4]);
            break;
        }
    }        
  
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

void flushData(void){
    tripNumber = 0;
    routeID = 0;
    u8g2.clearBuffer();
}

void setting_process_fsm(void){
    switch(SettingState){
        case STATE_IDLE_SETTING:{
            if(1){
                drawArrowSelect(1);
                SettingState = STATE_TRIPNUMBER_SETTING;
            }
            break;
        }
        case STATE_TRIPNUMBER_SETTING:{
            if(isDecrease()){                
                if(tripNumber > 0){
                    tripNumber--;
                }                                            
            }
            if(isIncrease()){                
                tripNumber++;                
            }
            if(isPressed()){                
                drawArrowSelect(0);
                SettingState = STATE_ROUTEID_SETTING;
            }
            break;
        }
        case STATE_ROUTEID_SETTING:{
            if(isDecrease()){            
                if(routeID > 0){
                    routeID--;
                }                
            }
            if(isIncrease()){                
                routeID++;                
            }
            if(isPressed()){                
                drawArrowSelect(1);
                SettingState = STATE_IDLE_SETTING;
            }
            break;
        }
    }        
    updateValues();
}

void menu_process_fsm(void){
    switch(MenuState){
        case STATE_IDLE_MENU:{
            if(1){
                drawStaticMenu();
                MenuState = STATE_SETTING;
            }
            break;
        }
        case STATE_SETTING:{
            setting_process_fsm();
            if(isLongPressed()){
                SettingState = STATE_IDLE_SETTING;
                xTaskNotifyGive(TaskHandle_SendData);                                
                isSending = true;                                                                
                resetRotaryEncoder();
                u8g2.clearBuffer();
                MenuState = STATE_MEASURING;
            }
            break;
        }
        case STATE_MEASURING:{
            display_process_fsm();
            if(isLongPressed()){
                flushData();                
                resetRotaryEncoder();                
                isSending = false;                                               
                MenuState = STATE_IDLE_MENU;
            }
            break;
        }
    }
}

void display_process_fsm(void){
    screen_tick = (screen_tick + 1) % display_tick;
    switch(DisplayState){
        case STATE_IDLE_DISPLAY:{
            if(1){
                DisplayState = STATE_DISPLAY_PAGE1;
            }
            break;
        }
        case STATE_DISPLAY_PAGE1:{            
            displaySubPage(1);            
            if(!screen_tick){
                u8g2.clearBuffer();    
                DisplayState = STATE_DISPLAY_PAGE2;
            }            
            break;
        }
        case STATE_DISPLAY_PAGE2:{
            displaySubPage(2);
            if(!screen_tick){
                u8g2.clearBuffer();    
                DisplayState = STATE_DISPLAY_PAGE1;
            }            
            break;
        }
    }

}



/*---------------------------------------------- Define task -------------------------------------------------*/


void Task_ReadSensor(void* pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    setupMagSensor();
    sensors_event_t eventMag;
    sensors_event_t eventAccel;         
    
    while(1){                
        /* Display the results (acceleration is measured in m/s^2) */                                
        if (xSemaphoreTake(xI2CSemaphore, portMAX_DELAY)){                    
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

            convertData();        
            xSemaphoreGive(xI2CSemaphore);
        }
        
        vTaskDelayUntil(&xLastWakeTime, 500);
        /* Delay before the next sample */        
    }
}


void Task_SendData(void* pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();  // Cập nhật thời gian trước vòng lặp
    while(1) {        
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);                            
        while(isSending) {            
            if (!isSending) {                
                xLastWakeTime = xTaskGetTickCount(); 
                break;
            }                            
            tb.sendTelemetryJson(data, data_size);  
            vTaskDelayUntil(&xLastWakeTime, 1000);
        }
    }
}

void Task_CheckConnection(void* pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    while(1){
        if (!tb.connected()) {                        
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
    u8g2.begin();
    drawStaticMenu();    
    while(1){        
        if(xSemaphoreTake(xI2CSemaphore, portMAX_DELAY)){
            menu_process_fsm();                    
            xSemaphoreGive(xI2CSemaphore);
        }   
        vTaskDelay(100);     
    }
}

void Task_ReadRotaryEncoder(void* pvParameters){
    RotaryEncoder_setup();
    while(1){        
        // menu_process_fsm();
        RotaryEncoder_loop();
        vTaskDelay(TIME_READ);
    }
}

void Task_ReadGPS(void* pvParameters){
    while(1){
        if(hardware.available() > 0){                     
            gps.encode(hardware.read());                   
            if(gps.location.isValid()){
                Sensor_Data.lat = gps.location.lat();
                Sensor_Data.lng = gps.location.lng();    
            }            
            else{
                Serial.println("INVALID");
            }
        }        
        vTaskDelay(10);
    }
}