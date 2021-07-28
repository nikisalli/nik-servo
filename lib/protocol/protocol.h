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

#define SERVO_CUR_READ_ID                   37
#define SERVO_CUR_READ_LENGTH_TX            3
#define SERVO_CUR_READ_LENGTH_RX            5

#define SERVO_TORQUE_LIMIT_WRITE_ID         38
#define SERVO_TORQUE_LIMIT_WRITE_LENGTH     5

#define SERVO_TORQUE_LIMIT_READ_ID          39
#define SERVO_TORQUE_LIMIT_READ_LENGTH_TX   3
#define SERVO_TORQUE_LIMIT_READ_LENGTH_RX   5

#define SERVO_POS_PARAMS_WRITE_ID           40
#define SERVO_POS_PARAMS_WRITE_LENGTH       7

#define SERVO_POS_PARAMS_READ_ID            41
#define SERVO_POS_PARAMS_READ_LENGTH_TX     3
#define SERVO_POS_PARAMS_READ_LENGTH_RX     7

#define SERVO_TORQUE_PARAMS_WRITE_ID        42
#define SERVO_TORQUE_PARAMS_WRITE_LENGTH    7

#define SERVO_TORQUE_PARAMS_READ_ID         43
#define SERVO_TORQUE_PARAMS_READ_LENGTH_TX  3
#define SERVO_TORQUE_PARAMS_READ_LENGTH_RX  7

// eeprom locations
#define EEPROM_ID                (uint8_t*) 0
#define EEPROM_ANGLE_OFFSET      (uint8_t*) 1
#define EEPROM_POS_KC_H          (uint8_t*) 2
#define EEPROM_POS_KC_L          (uint8_t*) 3
#define EEPROM_POS_INV_TI_H      (uint8_t*) 4
#define EEPROM_POS_INV_TI_L      (uint8_t*) 5
#define EEPROM_TORQUE_KC_H       (uint8_t*) 6
#define EEPROM_TORQUE_KC_L       (uint8_t*) 7
#define EEPROM_TORQUE_INV_TI_H   (uint8_t*) 8
#define EEPROM_TORQUE_INV_TI_L   (uint8_t*) 9

// global helpers
float fmap(float x, float in_min, float in_max, float out_min, float out_max);

class handler{
    public:
        uint8_t id = 0;
        int16_t angle_offset;
        bool led = false;

        // volatiles to be used in the ISR
        volatile float set_point = 150;
        volatile bool loaded = false;
        volatile float max_torque = 2;

        volatile float pos_kc = 10;
        volatile float pos_inv_ti = 0;
        volatile float torque_kc = 0;
        volatile float torque_inv_ti = 0;

        volatile uint16_t bposition;
        volatile float position;
        volatile uint16_t bcurrent;
        volatile float current;
        volatile float torque;

        handler();                 // initialize with eeprom vals
        void handle();             // parse serial bus
        void update_data();
        void write_id();
        void write_angle_offset();
        void write_pos();
        void write_loaded();
        void write_led();
        void write_move_time();
        void write_cur();
        void write_torque_limit();
        void write_pos_params();
        void write_torque_params();
};

#endif