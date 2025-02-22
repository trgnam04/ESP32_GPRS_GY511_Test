#ifndef ROTARY_ENCODER_H
#define ROTARY_ENCODER_H

#define CLK_PIN 32
#define DT_PIN 35
#define SW_PIN 34

#define PRESSED_STATE 0
#define NORMAL_STATE 1

#define CLK_INITIAL_STATE 1
#define DT_INITIAL_STATE 1

#define TIME_OUT_FOR_LONG_PRESS 2000
#define TIME_READ 5

void RotaryEncoder_setup();
void RotaryEncoder_loop();
bool isPressed();
bool isLongPressed();
bool isIncrease();
bool isDecrease();
void resetRotaryEncoder();



#endif // ROTARY_ENCODER_H