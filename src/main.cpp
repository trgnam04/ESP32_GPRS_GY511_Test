#include <U8g2lib.h>
#include <Wire.h>
#include "rotary_encoder.h"

#define SDA 21
#define SCK 22

U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);

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
    // Kích thước bảng tối ưu
    int rowHeight = 11;   // Chiều cao mỗi hàng (để cân đối)
    int paramWidth = 30;  // Chiều rộng cột "Param" (Nhỏ hơn)
    int valueWidth = 80;  // Chiều rộng cột "Value" (Lớn hơn)
    int startX = 5;       // Lề trái bảng
    int startY = 10;      // Lề trên bảng
  
    // Vẽ các đường ngang (chia 5 hàng)
    for (int i = 0; i <= 5; i++) {
      u8g2.drawHLine(startX, startY + (i * rowHeight), paramWidth + valueWidth);
    }
  
    // Vẽ đường dọc chia 2 cột
    u8g2.drawVLine(startX + paramWidth, startY, rowHeight * 5);
  
    // Chọn font chữ nhỏ để hiển thị tối ưu
    u8g2.setFont(u8g2_font_6x10_tf);
  
    // Hiển thị tiêu đề cột
    u8g2.drawStr(startX + 5, startY - 2, "P");  // Cột 1 (Param nhỏ gọn)
    u8g2.drawStr(startX + paramWidth + 5, startY - 2, "Value");  // Cột 2
  
    // Hiển thị các thông số
    const char* params[] = {"ax", "ay", "az", "lat", "lng"};
    const char* values[] = {"1.23", "-0.98", "0.50", "10.1234", "106.5678"};
  
    for (int i = 0; i < 5; i++) {
      u8g2.drawStr(startX + 5, startY + (i + 1) * rowHeight - 2, params[i]);
      u8g2.drawStr(startX + paramWidth + 5, startY + (i + 1) * rowHeight - 2, values[i]);
    }
  
    // Gửi buffer lên màn hình
    u8g2.sendBuffer();
}

int tripNumber = 0;
int routeID = 0;

void updateValues() {
    // Xóa giá trị cũ
    u8g2.setDrawColor(0);
    u8g2.drawBox(100, 25, 30, 40);  // Xóa vùng giá trị số
  
    // Hiển thị giá trị mới
    u8g2.setDrawColor(1);
    char buffer[10];
    
    sprintf(buffer, "%d", tripNumber);
    u8g2.drawStr(100, 35, buffer);
  
    sprintf(buffer, "%d", routeID);
    u8g2.drawStr(100, 50, buffer);
  
    u8g2.sendBuffer();
}

uint8_t press;
uint8_t longpress;
uint8_t inc;
uint8_t dec;

void updateButtonState(void){
    press = isPressed();
    longpress = isLongPressed();
    inc = isIncrese();
    dec = isDecrease();
}

uint8_t state = 3;

void setup() {  
    Wire.begin(SDA, SCK);
    u8g2.begin();
    RotaryEncoder_setup();
    drawStaticMenu();        
}

void loop() {        
    switch(state){
        case 3:{
            if(1){
                drawArrowSelect(1);
                state = 1;
            }
            break;
        }
        case 1:{
            if(isDecrease()){                
                tripNumber--;
                break;    
            }
            if(isIncrese()){                
                tripNumber++;
                break;
            }
            if(isPressed()){                
                drawArrowSelect(0);
                state = 0;
            }
            break;
        }
        case 0:{
            if(isDecrease()){            
                routeID--;
                break;
            }
            if(isIncrese()){                
                routeID++;
                break;
            }
            if(isPressed()){                
                drawArrowSelect(1);
                state = 1;
            }
            break;
        }
    }    
    // if(isIncrese()){
    //     tripNumber++;
    // }
    // if(isDecrease()){
    //     tripNumber--;
    // }        
    updateValues();
    RotaryEncoder_loop();    
    delay(5);
}    
