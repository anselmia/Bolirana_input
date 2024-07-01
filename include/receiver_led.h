#ifndef RECEIVER_LED_h
#define RECEIVER_LED_h
#include "Arduino.h"

class RECEIVER_LED
{
private:
    byte num;
    byte evPin;
    byte evState;
    int leap_year(int rtc_year);
    void calculate_next_day(MYEEPROM eeprom, byte rtc_day, byte rtc_month, int rtc_year);
    void ON();
    void OFF();

public:
    EV(byte pin, byte numEV);
    byte nextDayOn;
    int remainingTimeOn;
    void updateRemainingTime();
    void updateTimeOn(MYEEPROM eeprom, byte rtc_hour, byte rtc_min, byte rtc_day, byte rtc_month, int rtc_year);
    void init();
    void update_state();
};

#endif
