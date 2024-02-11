
#include <wiring_private.h>
#include "PinchangeInterrupt.h"


extern uint8_t event_triggered;

#if defined (MEGACOREX)
// max numbers of pin change interrupts
#define MAXNUMPCI 4

static volatile voidFuncPtr pci_isr[4]; //typedef void (*voidFuncPtr)(void); in wiring_private.h
uint8_t pci_mode[4];
uint8_t pci_pin[4];


void register_pci(uint8_t intnum, uint8_t pin, void (*userfunc)(void), uint8_t mode)
{
    if (intnum >= MAXNUMPCI) return;

    pci_mode[intnum] = mode;
    pci_isr[intnum] = userfunc;
    pci_pin[intnum] = pin;

    if (pin==10 || pin==14 || pin==18 || pin==22)
    {
        attachInterrupt(pin, userfunc, mode); // fully asynchronuous pins
    }
    else // normal, pin, only can wake up MCU with mode==CHANGE
    {
        if (mode == CHANGE)
        {
            attachInterrupt(pin, userfunc, CHANGE); // normal pin change interrupt pin
        }
        else // rising or falling
        {
            // check if it is a falling or rising edge, therefore call another function
            switch (intnum)
            {
                case 0:
                    attachInterrupt(pin, wakeUp0_pci, CHANGE);
                    break;
                case 1:
                    attachInterrupt(pin, wakeUp1_pci, CHANGE);
                    break;
                case 2:
                    attachInterrupt(pin, wakeUp2_pci, CHANGE);
                    break;
                case 3:
                    attachInterrupt(pin, wakeUp3_pci, CHANGE);
                    break;
            }

            pci_mode[intnum] = mode &0x1;
        }
    }

}


void wakeUp0_pci()
{
    if(digitalRead(pci_pin[0])==pci_mode[0])
    {
        pci_isr[0]();
    }
    else
        event_triggered |= 0x80; // indicate its a false wakeup reason
}

void wakeUp1_pci()
{
    if(digitalRead(pci_pin[1])==pci_mode[1])
    {
        pci_isr[1]();
    }
    else
        event_triggered |= 0x80;
}

void wakeUp2_pci()
{
    if(digitalRead(pci_pin[2])==pci_mode[2])
    {
        pci_isr[2]();
    }
    else
        event_triggered |= 0x80;
}

void wakeUp3_pci()
{
    if(digitalRead(pci_pin[3])==pci_mode[3])
    {
        pci_isr[3]();
    }
    else
        event_triggered |= 0x80;
}

#endif