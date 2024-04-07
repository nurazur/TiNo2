#include "print_things.h"

void print_humidity_sensor_values(const char* sensorname, float t, float h, HardwareSerial* S)
{
    // in the receiver all messages are subject to mess up the decoder that should store the received values in a csv file
    // so we set the parameter S to 0
    // in the sender we use the pointer to the applicable Serialx Object
    if (S)
    {
        S->print(sensorname);
        S->print(": ");
        S->print(t, 2);
        S->print(" degC, ");
        S->print(h, 2);
        S->println(" %rH");
    }
}


void print_serial_number(HardwareSerial* S)
{
    uint8_t* sernum_addr = (uint8_t*)&SIGROW.SERNUM0;
    int i;
    S->print("Serial Number ");
    #if defined (ARDUINO_avrdd)

        #if defined __AVR_AVR64DD32__
        S->print("AVR64DD32: ");
        #elif defined __AVR_AVR64DD28__
        S->print("AVR64DD28: ");
        #else
        S->print("AVRxxDDxx: ");
        #endif

    // AVRxxDDxx have 16 Bytes
    for (i=0;  i<16; i++)
    {
        uint8_t sni = *sernum_addr;

        if (i%4 == 0)
             S->print(" ");
        if (sni <16)
            S->print("0");
        S->print(sni,HEX);

        sernum_addr++;
    }
    S->println();

    #elif defined (ARDUINO_avrda)
        #if defined __AVR_AVR64DA32__
        S->print("AVR64DA32: ");
        #elif defined __AVR_AVR64DA28__
        S->print("AVR64DA28: ");
        #else
        S->print("AVRxxDAxx: ");
        #endif
    for (i=0;  i<16; i++)
    {
        uint8_t sni = *sernum_addr;
        S->print(sni);
        S->print(" ");
        sernum_addr++;
    }
    S->println();

    #else
    #if defined (__AVR_ATmega4808__)
    S->print("ATMEGA4808: ");
    #endif
    SerialNumber SN;
    for (i=0; i<6; i++)
    {
        SN.prefix[i] = (char) *sernum_addr;
        sernum_addr++;
    }
    SN.prefix[6]=0;

    for (i=0; i<4; i++)
    {
        SN.sn_char[i] = *sernum_addr;
        sernum_addr++;
    }

    S->print(SN.prefix); S->print(" "); S->println(SN.sn);
    #endif
}


void print_flag(uint8_t flag, Stream* S)
{
    if (flag &0x1)
    {
        S->println("\r\nHeartBeat");
    }

    flag >>=1;
    if (flag)
    {
        S->print("event: PCI");
        for (int i=0; i<4; i++)
        {
            if (flag&0x1)
            {
                S->print((char)('0'+i));
            }
            flag >>= 1;
        }
    }
    S->println();
}

void print_init_result(bool result_pass, const char* sensor_str, HardwareSerial* S)
{
    static const char* pass = "success\n";
    static const char* fail = "failed\n";

    S->print(sensor_str);
    S->print(": init() ");
    if (result_pass)
    {
        S->print(pass);
    }
    else
    {
        S->print(fail);
    }
}


/*********************/
//#if DEBUG == 2

// TODO: needs to be completed
void print_eeprom(Configuration& Config, Stream *serial)
{

    serial->print("Nodeid = ");        serial->print(Config.Nodeid);          serial->println();
    serial->print("Networkid = ");     serial->print(Config.Networkid);       serial->println();
    serial->print("Gatewayid =  ");    serial->print(Config.Gatewayid);       serial->println();
    serial->print("VccAtCalmV  = ");   serial->print(Config.VccAtCalmV);      serial->println();
    serial->print("AdcCalValue = ");   serial->print(Config.AdcCalValue);     serial->println();
    serial->print("Senddelay = ");     serial->print(Config.Senddelay);       serial->println();
    /*
    //serial->print("Frequencyband = "); serial->print(Config.Frequencyband);   serial->println();
    //serial->print("frequency = ");     serial->print(Config.frequency);       serial->println();
    //serial->print("TxPower = ");       serial->print(Config.TxPower);         serial->println();
    //serial->print("RequestAck = ");    serial->print(Config.RequestAck);      serial->println();
    //serial->print("LedCount = ");      serial->print(Config.LedCount);        serial->println();
    //serial->print("LedPin = ");        serial->print(Config.LedPin);          serial->println();
    //serial->print("RxPin = ");         serial->print(Config.RxPin);           serial->println();
    //serial->print("TxPin = ");         serial->print(Config.TxPin);           serial->println();
    //serial->print("SDAPin = ");        serial->print(Config.SDAPin);          serial->println();
    //serial->print("SCLpin = ");        serial->print(Config.SCLPin);          serial->println();
    //serial->print("I2CPowerPin = ");   serial->print(Config.I2CPowerPin);     serial->println();
    //serial->print("checksum = ");      serial->print(Config.checksum);        serial->println();

    serial->print(offsetof(Configuration,PCI0Pin));serial->print(" Interrupt 0 Pin = ");serial->print(Config.PCI0Pin);        serial->println();
    serial->print(offsetof(Configuration,PCI1Pin));serial->print(" Interrupt 1 Pin = ");serial->print(Config.PCI1Pin);        serial->println();
    serial->print(offsetof(Configuration,PCI2Pin));serial->print(" Interrupt 2 Pin = ");serial->print((int)Config.PCI2Pin);        serial->println();
    serial->print(offsetof(Configuration,PCI3Pin));serial->print(" Interrupt 3 Pin = ");serial->print((int)Config.PCI3Pin);        serial->println();
    //serial->print("Interrupt 0 Type = ");serial->print(show_trigger(Config.PCI0Trigger)); serial->print(", ");  serial->print(show_pinmode(Config.PCI0Trigger)); serial->println();
    //serial->print("Interrupt 1 Type = ");serial->print(show_trigger(Config.PCI1Trigger)); serial->print(", ");  serial->print(show_pinmode(Config.PCI1Trigger)); serial->println();
    //serial->print("Interrupt 2 Type = ");serial->print(show_trigger(Config.PCI2Trigger)); serial->print(", ");  serial->print(show_pinmode(Config.PCI2Trigger)); serial->println();
    //serial->print("Interrupt 3 Type = ");serial->print(show_trigger(Config.PCI3Trigger)); serial->print(", ");  serial->print(show_pinmode(Config.PCI3Trigger)); serial->println();
    //serial->flush();
    serial->print(offsetof(Configuration,UseCrystalRtc));serial->print(" Use Crystal = ");     serial->print(Config.UseCrystalRtc);  serial->println();
    //serial->print("Encryption = ");     serial->print(Config.EncryptionEnable);  serial->println();
    //serial->print("FEC = ");            serial->print(Config.FecEnable);  serial->println();
    //serial->print("Interleave = ");     serial->print(Config.InterleaverEnable);  serial->println();
    serial->print(offsetof(Configuration,EepromVersionNumber)); serial->print(" EEPROM Version= ");  serial->print(Config.EepromVersionNumber);  serial->println();
    serial->print(offsetof(Configuration,SoftwareversionNumber));serial->print(" Software Version = ");  serial->print(Config.SoftwareversionNumber);  serial->println();
    serial->print(offsetof(Configuration,TXGaussShaping));serial->print(" Gauss shaping= ");   serial->print(Config.TXGaussShaping);  serial->println();
    serial->print(offsetof(Configuration,SerialEnable));serial->print(" Serial Port Enable = "); serial->print(Config.SerialEnable);  serial->println();
    serial->print(offsetof(Configuration,IsRFM69HW));serial->print(" RF Chip = "); Config.IsRFM69HW ?    serial->print("RFM69HCW") : serial->print("RFM69CW");  serial->println();
    serial->print(offsetof(Configuration,PaBoost));serial->print(" PA Boost = ");      serial->print(Config.PaBoost);  serial->println();
    serial->print(offsetof(Configuration,FedvSteps));serial->print(" Fdev (Steps) = ");      serial->print(Config.FedvSteps);  serial->println();
    serial->flush();
    */
}
//#endif
