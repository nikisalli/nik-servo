#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <Arduino.h>

// command header
#define HEADER_BYTE                        uint8_t(0x55)
#define SERVO_HEADER                       uint8_t(0x55), uint8_t(0x55)

// broadcast id
#define SERVO_BROADCAST_ID                 254

// commands
#define SERVO_MOVE_TIME_WRITE_ID            1
#define SERVO_MOVE_TIME_WRITE_LENGTH        7

#define SERVO_MOVE_TIME_READ_ID             2
#define SERVO_MOVE_TIME_READ_LENGTH_TX      3
#define SERVO_MOVE_TIME_READ_LENGTH_RX      7

#define SERVO_ID_WRITE_ID                   13
#define SERVO_ID_WRITE_LENGTH               4

#define SERVO_ID_READ_ID                    14
#define SERVO_ID_READ_LENGTH_TX             3
#define SERVO_ID_READ_LENGTH_RX             4

#define SERVO_ANGLE_OFFSET_ADJUST_ID        17
#define SERVO_ANGLE_OFFSET_ADJUST_LENGTH    4

#define SERVO_ANGLE_OFFSET_WRITE_ID         18
#define SERVO_ANGLE_OFFSET_WRITE_LENGTH     3

#define SERVO_ANGLE_OFFSET_READ_ID          19
#define SERVO_ANGLE_OFFSET_READ_LENGTH_TX   3
#define SERVO_ANGLE_OFFSET_READ_LENGTH_RX   4

#define SERVO_POS_READ_ID                   28
#define SERVO_POS_READ_LENGTH_TX            3
#define SERVO_POS_READ_LENGTH_RX            5

#define SERVO_LOAD_OR_UNLOAD_WRITE_ID       31
#define SERVO_LOAD_OR_UNLOAD_WRITE_LENGTH   4

#define SERVO_LOAD_OR_UNLOAD_READ_ID        32
#define SERVO_LOAD_OR_UNLOAD_READ_LENGTH_TX 3
#define SERVO_LOAD_OR_UNLOAD_READ_LENGTH_RX 4

#define SERVO_LED_CTRL_WRITE_ID             33
#define SERVO_LED_CTRL_WRITE_LENGTH         4

#define SERVO_LED_CTRL_READ_ID              34
#define SERVO_LED_CTRL_READ_LENGTH_TX       3
#define SERVO_LED_CTRL_READ_LENGTH_RX       4

// eeprom locations
#define EEPROM_ID                          0
#define EEPROM_ANGLE_OFFSET                1

float fmap(float x, float in_min, float in_max, float out_min, float out_max);

class handler{
    public:
        uint8_t id = 0;
        int16_t angle_offset;
        volatile float set_point = 150; // this is needed inside the control loop isr
        volatile bool loaded = false; // this too
        bool led = false;

        handler();                 // initialize with eeprom vals
        void handle();             // parse serial bus
        void write_id();
        void write_angle_offset();
        void write_pos();
        void write_loaded();
        void write_led();
        void write_move_time();
};

#endif