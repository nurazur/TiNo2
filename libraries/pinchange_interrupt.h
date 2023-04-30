#include "Arduino.h"

void register_pci(uint8_t intnum, uint8_t pin, void (*userfunc)(void), uint8_t mode);
void wakeUp0_pci(void);
void wakeUp1_pci(void);
void wakeUp2_pci(void);
void wakeUp3_pci(void);