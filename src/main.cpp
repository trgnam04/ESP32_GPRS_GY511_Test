#include <U8g2lib.h>
#include <Wire.h>
#include "rotary_encoder.h"

#define SDA 21
#define SCK 22

U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);

typedef struct {
    uint16_t trip_number = 0;
    uint16_t route_id = 0;
} data_t;

data_t Data;

void drawStaticMenu(void){
    u8g2.clearBuffer();    
    u8g2.setFont(u8g2_font_6x12_tf);
    u8g2.drawStr((128 - u8g2.getStrWidth("Menu")) / 2, 12, "Menu");      
    u8g2.drawHLine(5, 16, 118);    
    u8g2.drawStr(20, 35, "Trip Number");
    u8g2.drawStr(20, 50, "Route ID");  
    u8g2.sendBuffer();
}


void updateValues(data_t tData) {
    // Xóa giá trị cũ
    u8g2.setDrawColor(0);
    u8g2.drawBox(100, 25, 30, 40);  // Xóa vùng giá trị số
  
    // Hiển thị giá trị mới
    u8g2.setDrawColor(1);
    char buffer[10];
    
    sprintf(buffer, "%d", tData.trip_number);
    u8g2.drawStr(100, 35, buffer);
  
    sprintf(buffer, "%d", tData.route_id);
    u8g2.drawStr(100, 50, buffer);
  
    u8g2.sendBuffer();
}


TaskHandle_t readRotaryEncTask_handle;
TaskHandle_t DisplayProcessTask_handle;
TaskHandle_t DataProcessTask_handle;

QueueHandle_t DataQueue;
QueueHandle_t Data2Queue;


void readRotaryEncTask(void* vParameters){
    RotaryEncoder_setup();
    uint8_t state = 0;

    for(;;){
        switch(state){
            case 0:{
                if(isLongPressed()){
                    vTaskResume(DataProcessTask_handle);
                    state = 1;
                }
                break;
            }
            case 1:{
                if(isLongPressed()){
                    vTaskSuspend(DataProcessTask_handle);
                    state = 0;
                }
                break;
            }
        }                
        RotaryEncoder_loop();
        vTaskDelay(TIME_READ);
    }

}

void DisplayProcessTask(void* vParameters){
    Wire.begin(SDA, SCK);
    u8g2.begin();    
    data_t ReceivedData;
    drawStaticMenu();

    for(;;){                 
        xQueueReceive(DataQueue, &ReceivedData, portMAX_DELAY);
        updateValues(ReceivedData);
        vTaskDelay(100);
    }

}

void DataProcessTask(void* vParameters){
    for(;;){
        Data.route_id ++;
        Data.trip_number ++;
        xQueueSendToBack(DataQueue, &Data, portMAX_DELAY);
        vTaskDelay(1000);
    }
}


void setup() {  
    Serial.begin(9600);
                
    DataQueue = xQueueCreate(5, sizeof(data_t));
    Data2Queue = xQueueCreate(5, sizeof(data_t));
    
    xTaskCreatePinnedToCore(DisplayProcessTask, "Display Process", (1024 * 2), NULL, 1, &DisplayProcessTask_handle, 0);

    xTaskCreatePinnedToCore(DataProcessTask, "Datalay Process", (1024 * 1), NULL, 1, &DataProcessTask_handle, 0);
    vTaskSuspend(DataProcessTask_handle);

    xTaskCreatePinnedToCore(readRotaryEncTask, "Read Rotary Encoder", (1024 * 1), NULL, 1, &readRotaryEncTask_handle, 0);
    

}

void loop() {        
}    
