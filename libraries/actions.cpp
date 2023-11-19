#include <Arduino.h>
#include "actions.h"
#include "configuration.h"
#include "EEPROM.h"



#define ACTION_ON 1
#define ACTION_OFF 0
#define ACTION_TOGGLE 2
#define ACTION_PULSE 3

#define ADR_NUM_ACTIONS sizeof(Configuration)
#define ADR_ACTIONS   ADR_NUM_ACTIONS + 1
#define MAX_NUM_ACTIONS 40


/* Globals */
action *actions;
uint8_t num_actions;
// globals are bad. so later, this will become a class, and we don't need gloals anymore.


uint16_t checksum_crc16(uint8_t *data, uint8_t data_len)
{
    uint16_t crc = 0xFFFF;

    if (data_len == 0)
        return 0;

    for (unsigned int i = 0; i < data_len; ++i)
    {
        uint16_t dbyte = data[i];
        crc ^= dbyte << 8;

        for (unsigned char j = 0; j < 8; ++j)
        {
            uint16_t mix = crc & 0x8000;
            crc = (crc << 1);
            if (mix)
                crc = crc ^ 0x1021;
        }
    }

    return crc;
}




void doaction (uint8_t nodeid, uint8_t flags, action actions[], uint8_t num_actions, PITControl* Pit)
{
    for (uint8_t i=0; i< num_actions; i++)
    {
        if(actions[i].node == nodeid) // nodeid matches the actions's nodeId
        {
            if (actions[i].mask & flags ) // the action's trigger bit(s) is set
            {
                switch (actions[i].mode)// Action mode
                {
                    case ACTION_OFF:     // OFF
                        digitalWrite(actions[i].pin, 0);
                        break;
                    case ACTION_ON:     // ON
                        digitalWrite(actions[i].pin, 1);
                        break;
                    case ACTION_TOGGLE:     // Toggle
                        digitalWrite(actions[i].pin, !digitalRead(actions[i].pin));
                        break;
                    case ACTION_PULSE:     //Pulse
                        digitalWrite(actions[i].pin, !actions[i].default_val);
                        uint16_t dur = 1<<actions[i].duration;
                        dur++;
                        Pit->start(actions[i].pin, actions[i].default_val, dur);
                        break;
                }
            }
        }
    }
}

void init_actions(HardwareSerial* S)
{
    EEPROM.get(ADR_NUM_ACTIONS, num_actions);

    if (num_actions > 0 && num_actions <= MAX_NUM_ACTIONS)
    {
        S->print("number of actions: "); S->println(num_actions);
        actions = new action[num_actions];
    }
    else
    {
        num_actions = 0;
        actions = NULL;
    }

    for(uint8_t i=0; i< num_actions; i++)
    {
        EEPROM.get(ADR_ACTIONS + i*sizeof(action), actions[i]);
    }

    if (num_actions > 0)
    {
        /* verify the checksum */

        // read checksum stored in EEPROM
        uint16_t cs_from_eeprom;
        EEPROM.get(ADR_ACTIONS + MAX_NUM_ACTIONS * sizeof(action), cs_from_eeprom);

        // calculate checksum from data copied from EEPROM
        uint16_t cs_from_data = checksum_crc16((uint8_t*) actions, sizeof(action) * num_actions);

        // verify
        if ((cs_from_eeprom ^ cs_from_data) != 0)
        {
            S->println("Checksum incorrect for Actions.");
            num_actions =0;
        }


        for(uint8_t i=0; i< num_actions; i++)
        {
            pinMode(actions[i].pin, OUTPUT);
            digitalWrite(actions[i].pin, actions[i].default_val);
            /*
            S->print("\nnode :"); S->println(actions[i].node);
            S->print("mask :"); S->println(actions[i].mask);
            S->print("pin :"); S->println(actions[i].pin);
            S->print("dur. :"); S->println(actions[i].duration);
            S->print("mode :"); S->println(actions[i].mode);
            S->print("def. :"); S->println(actions[i].default_val);
            */
        }

    }

}