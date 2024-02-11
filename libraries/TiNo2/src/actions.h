
#include "pitctrl.h"






/*
if action.node is equal to the node in the received frame, then the flag word in the frame is evaluated.
4 flags can be set (bits 1 to bit 4 in the flag byte). Each flag can be mapped to a pin using the mask byte.
The pin is defined in the pin[i] array (up to 4 different pins for the four flags)
Actions are: ON, OFF, TOGGLE pin, PULSE with length 2^B * 0.125 seconds.

Example: Node 15,  pin 7 shall turn on on flag 02, pin 7 shall turn off on flag 04. pin 8 shall be toggled on flag 0x08, pin 5 shall be pulsed for 0.5 seconds on flag 0x10

node = 15;
pin = 7;
mask = 0x02;
mode = 0x1; //ON
duration =0; // doesn't matter
default_val=0; (off is default)

node = 15;
pin = 7;
mask = 0x04;
mode = 0x0; //OFF
duration =0; // doesn't matter
default_val=0; (off is default)

node = 15;
pin = 8;
mask = 0x08;
mode = 0x10; // TOGGLE
duration =0; // doesn't matter
default_val=0; (off is default)

node = 15;
pin = 5;
mask = 0x10;
mode = 0x3; // PULSE
duration = 0x2 B=2, 2^B = 4 * 0.125s = 0.5s
default_val=0; (off is default)
*/

/*****************************************************************************/
/***                       Actions Struct                                  ***/
/*****************************************************************************/
typedef struct {
  uint8_t node;             // the node to trigger on (the node from which the signal comes)
  uint8_t pin;             // the pin to trigger
  uint8_t mask;
  uint8_t mode:2;           //type of action:  OFF (00) ON(01) TOGGLE (10) PULSE (11)
  uint8_t duration:5;       //  Pulse length = 2^duration
  uint8_t default_val:1;    //default on power up, 1= on, 0 = off
} action;


uint16_t checksum_crc16(uint8_t *data, uint8_t data_len);

void doaction (uint8_t nodeid, uint8_t flags, action actions[], uint8_t num_actions, PITControl* Pit);

void init_actions(HardwareSerial* mySerial);
