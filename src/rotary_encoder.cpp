#include "rotary_encoder.h"
#include <Arduino.h>

int swBuffer[4] = {NORMAL_STATE, NORMAL_STATE, NORMAL_STATE, NORMAL_STATE};
int timeOutForPressed = TIME_OUT_FOR_LONG_PRESS / TIME_READ;
int swPressFlag = 0;
int swLongPressFlag = 0;
int clockwiseFlag = 0;
int counterClockwiseFlag = 0;

int clkLastState = CLK_INITIAL_STATE;
int dtLastState = DT_INITIAL_STATE;
int clkState = CLK_INITIAL_STATE;
int dtState = DT_INITIAL_STATE;

void RotaryEncoder_setup()
{
    pinMode(CLK_PIN, INPUT);
    pinMode(DT_PIN, INPUT);
    pinMode(SW_PIN, INPUT);

    clkLastState = digitalRead(CLK_PIN);
    dtLastState = digitalRead(DT_PIN);
}
void RotaryEncoder_loop()
{
    swBuffer[2] = swBuffer[1];
    swBuffer[1] = swBuffer[0];
    swBuffer[0] = digitalRead(SW_PIN);
    if ((swBuffer[0] == swBuffer[1]) && (swBuffer[1] == swBuffer[2]))
    {
        if (swBuffer[2] != swBuffer[3])
        {
            swBuffer[3] = swBuffer[2];
            if (swBuffer[3] == PRESSED_STATE)
            {
                timeOutForPressed = TIME_OUT_FOR_LONG_PRESS / TIME_READ;
                swPressFlag = 1;
            }
        }
        else // swBuffer[2] == swBuffer[3]
        {
            --timeOutForPressed;
            if (timeOutForPressed <= 0)
            {
                timeOutForPressed = TIME_OUT_FOR_LONG_PRESS / TIME_READ;
                if (swBuffer[3] == PRESSED_STATE)
                {
                    swLongPressFlag = 1;
                }
            }
        }
    }

    clkState = digitalRead(CLK_PIN);
    dtState = digitalRead(DT_PIN);
    if (clkState != clkLastState)
    {
        if (clkState != dtState)
        {
            // Clockwise
            clockwiseFlag = 1;
            counterClockwiseFlag = 0;

            clkLastState = clkState;
            dtLastState = dtState;
        }
        else
        {
            // Counter clockwise
            counterClockwiseFlag = 1;
            clockwiseFlag = 0;

            clkLastState = clkState;
            dtLastState = dtState;
        }
    }
}
bool isPressed()
{
    if (swPressFlag)
    {
        swPressFlag = 0;
        return 1;
    }
    return 0;
}
bool isLongPressed()
{
    if (swLongPressFlag)
    {
        swLongPressFlag = 0;
        return 1;
    }
    return 0;

}
bool isIncrease()
{
    if (clockwiseFlag)
    {
        clockwiseFlag = 0;
        return 1;
    }
    return 0;

}
bool isDecrease()
{
    if (counterClockwiseFlag)
    {
        counterClockwiseFlag = 0;
        return 1;
    }
    return 0;

}

void resetRotaryEncoder()
{
    swPressFlag = 0;
    swLongPressFlag = 0;
    clockwiseFlag = 0;
    counterClockwiseFlag = 0;
}