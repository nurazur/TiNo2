// Blink LED with Core Independent Peripherals
//
// This is a demo on how to use core independent peripherals to
// blink an LED whithout using the CPU. The periodic interrupt timer (PIT)
// directly toggles pin PD7 (which is connected to the builtin LED) via
// channel0 of the event system (EVSYS). Because the CPU isn't needed anymore,
// it is put into sleep mode. The PIT continues to run and wakes up the CPU
// every 8 seconds. To dempnstrate start and stop blinking, every third wake up
// the blinking is toggled. 
//
// Controller:  ATmega4808
// Core:        MegaCoreX (https://github.com/MCUdude/MegaCoreX)
// Clock:       doesn't matter
//
// Original sketch:
// 2020 by Stefan Wagner 
// Project Files (EasyEDA): https://easyeda.com/wagiminator
// Project Files (Github):  https://github.com/wagiminator
//
// adopted to TiNo2:
// 2024 nurazur
// https://github.com/tino2
//

#include "Arduino.h"
//#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/sleep.h>

// original code
/*
int main(void) {
  PORTA.DIRSET = PIN7_bm;                       // builtin LED as output
  RTC.PITCTRLA = RTC_PITEN_bm;                  // enable PIT
  EVSYS.CHANNEL0 = EVSYS_GENERATOR_RTC_PIT0_gc; // PIT div 8192 as generator
  EVSYS.USEREVOUTA = EVSYS_CHANNEL_CHANNEL0_gc; // port A as user
  PORTMUX.EVSYSROUTEA = PORTMUX_EVOUT0_bm;      // pin 7  as user
  SLPCTRL.CTRLA |= SLPCTRL_SMODE_PDOWN_gc;      // set sleep mode power down
  SLPCTRL.CTRLA |= SLPCTRL_SEN_bm;              // set sleep enable
  sleep_cpu();                                  // go to sleep forever
}
*/

#define SERIAL_SWAP 0
#define SERIAL_BAUD 57600

#define UPDATE_PERIOD 5

uint16_t watchdog_counter=0;
bool blink=false;
bool tick = true;


// interrupt service routine for RTC counter overflow
ISR(RTC_CNT_vect)
{
  RTC.INTFLAGS = 0xFF;              // clear interrupt flag
  tick = true;                      // raise tick flag
}

// interrupt service routine for RTC periodic interrupt timer (PIT)
ISR(RTC_PIT_vect)
{
    RTC.PITINTFLAGS = RTC_PI_bm;              // clear interrupt flag
    //watchdog_counter++;
}



void RTC_init(void) 
{
  RTC.CTRLA = RTC_RUNSTDBY_bm               // enable RTC in standby
            | RTC_PRESCALER_DIV1024_gc;    // prescale for 1 second
  while (RTC.STATUS & RTC_CTRLABUSY_bm);    // wait for done
  RTC.PER = UPDATE_PERIOD;                  // set RTC period
  while (RTC.STATUS & RTC_PERBUSY_bm);      // wait for done
  RTC.INTFLAGS = 0xFF;                      // clear interrupt flags
  RTC.INTCTRL = RTC_OVF_bm;                 // enable interrupt on overflow
  RTC.CTRLA |= RTC_RTCEN_bm;                // enable RTC
  while (RTC.STATUS & RTC_CTRLABUSY_bm);    // wait for done
}


// only possible pins are PF2(Arduino 22), PD2(14 TiNo2 Interrupt from RFM), PD7(Arduino 19 TiNo2 LED), PA2(2, SDA), PA7(7, SS)
void CIPBlink_LED_Start1(uint8_t speed)// speed 0-7, 0 = langsam, 7 = schnell
{
    #if defined (MEGACOREX)
    if (speed&0x4)// channel 1
    {
        EVSYS.CHANNEL1 = 8 + (speed&0x3);  //PIT_DIV512, PIT_DIV256, PIT_DIV128, PIT_DIV64
        EVSYS.USEREVOUTD = EVSYS_CHANNEL_CHANNEL1_gc; // connect user to port D
    }
    else // channel 0)
    {
        EVSYS.CHANNEL0 = 8 + (speed&0x3);
        EVSYS.USEREVOUTD = EVSYS_CHANNEL_CHANNEL0_gc; // connect user to port D
    }
    
    PORTMUX.EVSYSROUTEA = PORTMUX_EVOUT3_bm;      // pin 7  as user
    
    #elif defined (ARDUINO_avrdd)
    if (speed&0x4)// channel 1
    {
        EVSYS.CHANNEL1 = 8 + (speed&0x3);
        EVSYS.USEREVSYSEVOUTD = EVSYS_USER_CHANNEL1_gc;
    }
    else // channel 0)
    {
        EVSYS.CHANNEL0 = 8 + (speed&0x3);
        EVSYS.USEREVSYSEVOUTD = EVSYS_USER_CHANNEL0_gc;
    }
    
    PORTMUX.EVSYSROUTEA = PORTMUX_EVOUTD_ALT1_gc;   // Pin PD7 (19)
    #endif
}

void CIPBlink_LED_Start(void)
{
    #if defined (MEGACOREX)
    EVSYS.CHANNEL0 = EVSYS_GENERATOR_RTC_PIT3_gc; // PIT div 1024 as generator
    //EVSYS.CHANNEL0 = 9; // 8,9,10,11 11=fast, 8=slow
    // EVSYS_CHANNEL_CHANNEL0_gc == 1
    // EVSYS_CHANNEL_CHANNEL1_gc == 2
    EVSYS.USEREVOUTD = EVSYS_CHANNEL_CHANNEL1_gc; // connect user to port D
    PORTMUX.EVSYSROUTEA = PORTMUX_EVOUT3_bm;      // pin 7  as user
    
    #elif defined (ARDUINO_avrdd)
    EVSYS.CHANNEL0 = EVSYS_CHANNEL0_RTC_PIT_DIV1024_gc;
    EVSYS.USEREVSYSEVOUTD = EVSYS_USER_CHANNEL0_gc;
    PORTMUX.EVSYSROUTEA = PORTMUX_EVOUTD_ALT1_gc;   // Pin PD7 (19)
    #endif
}


void CIPBlink_LED_Stop(void)
{
    #if defined (MEGACOREX)
    EVSYS.CHANNEL0 = EVSYS_GENERATOR_OFF_gc;    // generator off
    EVSYS.CHANNEL1 = EVSYS_GENERATOR_OFF_gc;
    EVSYS.USEREVOUTD = EVSYS_CHANNEL_OFF_gc;    // deconnect user to port D
    PORTMUX.EVSYSROUTEA = PORTMUX_EVOUT3_bm;    // pin 7  as user
    
    #elif defined (ARDUINO_avrdd)
    EVSYS.CHANNEL0 = EVSYS_CHANNEL0_OFF_gc;
    EVSYS.USEREVSYSEVOUTD = EVSYS_USER_OFF_gc;
    PORTMUX.EVSYSROUTEA = PORTMUX_EVOUTD_ALT1_gc;   // Pin PD7 (19)
    #endif
}

uint8_t blink_speed=5;

void setup()
{
    Serial.swap(SERIAL_SWAP);
    Serial.begin(SERIAL_BAUD);
    Serial.println(__FILE__);

    // builtin LED (on TiNo2) is Arduino Pin 19 = PD7 as output
    pinMode(19, OUTPUT);

    // PIT Setup (same as TiNo)
    // RTC.PITINTCTRL = RTC_PI_bm; 
    RTC.PITCTRLA =  RTC_PERIOD_CYC8192_gc | RTC_PITEN_bm; //8s
    //RTC.PITCTRLA =  RTC_PERIOD_CYC16384_gc | RTC_PITEN_bm;      
    RTC.CLKSEL = RTC_CLKSEL_INT1K_gc;
    RTC_init();
    
    sei();
    //set_sleep_mode(SLEEP_MODE_PWR_DOWN);      // set sleep mode
    set_sleep_mode(SLEEP_MODE_STANDBY);
    sleep_enable(); 

    //CIPBlink_LED_Start();
    CIPBlink_LED_Start1(blink_speed);
    blink = false;
    
    Serial.flush();
}
// this example 
  
void loop ()
{
    //if (watchdog_counter % 3 == 0)
    if (tick)
    {
        Serial.println("toggle blink");
        if (blink)
        {
            CIPBlink_LED_Stop();
            blink= false;
        }
        else
        {
            CIPBlink_LED_Start1(blink_speed);
            blink= true;
        }
        tick=false;
    }
    Serial.flush();
    sleep_cpu();      
}

