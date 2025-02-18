#include <TinyGPSPlus.h>
#include <Arduino.h>
#define TX_PIN 17
#define RX_PIN 16
#define GPS_BRATE 9600
#define FREQUENCY 1
HardwareSerial hardware(2);
TinyGPSPlus gps;
xSemaphoreHandle xMutex;
void displayInfo();
char information[20];
void Task_Display(void* pvParameters);
void Task_Serial(void* pvParameters);
void convertData(void);


double lat;
double lng;
char buffer[20];

void setup() {
  // put your setup code here, to run once:
  xMutex = xSemaphoreCreateBinary();
  if(xMutex != NULL){
      xSemaphoreGive(xMutex);
  }
  Serial.begin(115200);
  hardware.begin(9600);
  xTaskCreatePinnedToCore(Task_Display, "Task_Test_Mag_Sensor", 4096, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(Task_Serial, "Task_Test_LCD", 4096, NULL, 1, NULL, 0);
}
void loop()
{
}

void convertData(void){
  snprintf(buffer, 20, "%.6f;%.6f", lat, lng);
}

void displayInfo()
{
  convertData();
  Serial.print(buffer);
  // Serial.print(F("Location: ")); 
  // if (gps.location.isValid())
  // {
  //   lat = gps.location.lat();
  //   lng = gps.location.lng();
  //   Serial.print(lat, 6);
  //   Serial.print(F(","));
  //   Serial.print(lng, 6);
    
  // }
  // else
  // {
  //   Serial.print(F("INVALID"));
  // }

  // Serial.print(F("  Date/Time: "));
  // if (gps.date.isValid())
  // {
  //   Serial.print(gps.date.month());
  //   Serial.print(F("/"));
  //   Serial.print(gps.date.day());
  //   Serial.print(F("/"));
  //   Serial.print(gps.date.year());
  // }
  // else
  // {
  //   Serial.print(F("INVALID"));
  // }

  // Serial.print(F(" "));
  // if (gps.time.isValid())
  // {
  //   if (gps.time.hour() < 10) Serial.print(F("0"));
  //   Serial.print(gps.time.hour());
  //   Serial.print(F(":"));
  //   if (gps.time.minute() < 10) Serial.print(F("0"));
  //   Serial.print(gps.time.minute());
  //   Serial.print(F(":"));
  //   if (gps.time.second() < 10) Serial.print(F("0"));
  //   Serial.print(gps.time.second());
  //   Serial.print(F("."));
  //   if (gps.time.centisecond() < 10) Serial.print(F("0"));
  //   Serial.print(gps.time.centisecond());
  // }
  // else
  // {
  //   Serial.print(F("INVALID"));
  // }

  Serial.println();
}
void Task_Display(void* pvParameters)
{
    while(1){                
        Serial.print(millis());
        /* Delay before the next sample */
        if(xSemaphoreTake(xMutex, portMAX_DELAY)){
          displayInfo();
          xSemaphoreGive(xMutex);
        }
        vTaskDelay(1000/FREQUENCY - 2);
        
    }
}
void Task_Serial(void* pvParameters)
{
    while(1){                
      if(hardware.available() > 0){
        if(xSemaphoreTake(xMutex, portMAX_DELAY)){
          gps.encode(hardware.read());
          xSemaphoreGive(xMutex);
        }          
      }
      vTaskDelay(10);
    }
}