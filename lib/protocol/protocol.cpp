#include <avr/io.h>
#include <avr/eeprom.h>
#include <avr/wdt.h>

#include "protocol.h"

// hardware
float get_pos(){
    return analogRead(A1) / 2.841666666f; // 0..1023 -> 0..360
}

float get_cur(){
    return fmap(analogRead(A0), 0, 1023, -12.5, 12.5);
}

float get_torque(){
    return fmap(analogRead(A0), 0, 1023, -8.75, 8.75);
}

// helpers
uint8_t cpl2int(uint8_t val){
    return 0x80 & val ? (0x7F & val) - 0x80 : val;  // fast inverse 2's complement
}

uint8_t int2cpl(uint8_t val){
    return val > 0 ? val : (~val) + 1;
}

float fmap(float x, float in_min, float in_max, float out_min, float out_max) {
  return (float)(x - in_min) * (out_max - out_min) / (float)(in_max - in_min) + out_min;
}

// uart handling
void servo_tx_enb() {
    PORTC |= _BV(PC3);
}

void servo_rx_enb() {
	PORTC &= ~_BV(PC3);
}

uint8_t wait_for_bytes(uint8_t num, uint8_t timeout){
    uint16_t iters = 0;
    while(Serial.available() < num){
        if(iters++ > timeout){
            return 0;
        }
        delayMicroseconds(1000);
    }
    return 1;
}

// packet construction
uint8_t _write(uint8_t b) {
	Serial.write(b);
	return b;
}

template<typename... Args>
uint8_t _write(uint8_t b, Args... args) {
	return _write(b) + _write(args...);
}

template<typename... Args>
void servo_write(Args... args) {
    servo_tx_enb();
	_write(SERVO_HEADER);
	uint8_t checksum = ~_write(uint8_t(args)...);
	_write(checksum);
    Serial.flush();
    servo_rx_enb();
}

// constructor
handler::handler(){
    servo_rx_enb(); // set rx mode
    // read stored params from eeprom
    id = eeprom_read_byte(EEPROM_ID);
    angle_offset = cpl2int(eeprom_read_byte(EEPROM_ANGLE_OFFSET));
    pos_kc = (eeprom_read_byte(EEPROM_POS_KC_L) | ((eeprom_read_byte(EEPROM_POS_KC_H) << 8))) / 100.f;
    torque_inv_ti = (eeprom_read_byte(EEPROM_TORQUE_INV_TI_L) | ((eeprom_read_byte(EEPROM_TORQUE_INV_TI_H) << 8))) / 100.f;
}

// packets
void handler::write_id(){
    servo_write(
        id,
        SERVO_ID_READ_LENGTH_RX,
        SERVO_ID_READ_ID,
        id
	);
}

void handler::write_angle_offset(){
    servo_write(
        id,
        SERVO_ANGLE_OFFSET_READ_LENGTH_RX,
        SERVO_ANGLE_OFFSET_READ_ID,
        int2cpl(angle_offset)
	);
}

void handler::write_pos(){
    // this purposefully overflows to let the master know it is a negative value
    uint16_t pos = (uint16_t)(fmap(analogRead(A1), 0, 1023, -250, 1250));
    servo_write(
        id,
        SERVO_POS_READ_LENGTH_RX,
        SERVO_POS_READ_ID,
        (uint8_t)(pos & 0xFF),
        (uint8_t)(pos >> 8)
	);
}

void handler::write_loaded(){
    servo_write(
        id,
        SERVO_LOAD_OR_UNLOAD_READ_LENGTH_RX,
        SERVO_LOAD_OR_UNLOAD_READ_ID,
        loaded
	);
}

void handler::write_led(){
    servo_write(
        id,
        SERVO_LED_CTRL_READ_LENGTH_RX,
        SERVO_LED_CTRL_READ_ID,
        digitalRead(A4)
	);
}

void handler::write_move_time(){
    // this purposefully overflows to let the master know it is a negative value
    uint16_t pos = (uint16_t)(fmap(set_point, 0, 360, -250, 1250));
    servo_write(
        id,
        SERVO_MOVE_TIME_READ_LENGTH_RX,
        SERVO_MOVE_TIME_READ_ID,
        (uint8_t)(pos & 0xFF),
        (uint8_t)(pos >> 8),
        0, // we do not use the time parameter
        0
	);
}

void handler::write_cur(){
    uint16_t cur = analogRead(A0);
    servo_write(
        id,
        SERVO_CUR_READ_LENGTH_RX,
        SERVO_CUR_READ_ID,
        (uint8_t)(cur & 0xFF),
        (uint8_t)(cur >> 8)
	);
}

void handler::write_torque_limit(){
    uint16_t torque_limit = max_torque * 1000;
    servo_write(
        id,
        SERVO_TORQUE_LIMIT_READ_LENGTH_RX,
        SERVO_TORQUE_LIMIT_READ_ID,
        (uint8_t)(torque_limit & 0xFF),
        (uint8_t)(torque_limit >> 8)
	);
}

void handler::write_pos_params(){
    uint16_t kc = pos_kc * 100;
    uint16_t ti = pos_inv_ti * 100;
    servo_write(
        id,
        SERVO_POS_PARAMS_READ_LENGTH_RX,
        SERVO_POS_PARAMS_READ_ID,
        (uint8_t)(kc & 0xFF),
        (uint8_t)(kc >> 8),
        (uint8_t)(ti & 0xFF),
        (uint8_t)(ti >> 8)
	);
}

void handler::write_torque_params(){
    uint16_t kc = torque_kc * 100;
    uint16_t ti = torque_inv_ti * 100;
    servo_write(
        id,
        SERVO_TORQUE_PARAMS_READ_LENGTH_RX,
        SERVO_TORQUE_PARAMS_READ_ID,
        (uint8_t)(kc & 0xFF),
        (uint8_t)(kc >> 8),
        (uint8_t)(ti & 0xFF),
        (uint8_t)(ti >> 8)
	);
}

// handler
void handler::handle(){
    wait_for_bytes(3, 10);                   // wait for 3 bytes with 10ms timeout
    if(Serial.read() != HEADER_BYTE) return; // match first header byte
    if(Serial.read() != HEADER_BYTE) return; // match second header byte
    uint8_t res = Serial.read();             // if the header is matched get id
    if(res != id && res != SERVO_BROADCAST_ID) return; // check if the command is directed to us or broadcasted

    wait_for_bytes(2, 10);
    uint8_t length = Serial.read();         // read length
    uint8_t cmd = Serial.read();            // read cmd id
    uint8_t _id = res;

    uint8_t params[7] = {};                    // read n parameters

    uint8_t sum = _id + length + cmd;

    wait_for_bytes(length - 2, 10);

    for(uint8_t i = 0; i < length - 2; i++){ //iterate until the packet has been fully read
        params[i] = Serial.read();
        if(i != length - 3) sum += params[i];   //do not sum checksum
    }

    if(((~sum) & 0xFF) != (params[length - 3] & 0xFF)) return; //match checksum
    
    if (_id == id) { // if the command is directed to us
        switch (cmd) {
            case SERVO_ID_WRITE_ID: // set id
                eeprom_write_byte(EEPROM_ID, params[0]); // parameter 1 is our new id
                id = params[0]; // update our id
                break;
            case SERVO_ANGLE_OFFSET_ADJUST_ID: // set angle offset
                angle_offset = cpl2int(params[0]);
                break;
            case SERVO_ANGLE_OFFSET_WRITE_ID: // write angle offset to eeprom
                eeprom_write_byte(EEPROM_ANGLE_OFFSET, int2cpl(angle_offset));
                break;
            case SERVO_ANGLE_OFFSET_READ_ID:
                write_angle_offset();
                break;
            case SERVO_POS_READ_ID:
                write_pos();
                break;
            case SERVO_LOAD_OR_UNLOAD_WRITE_ID:
                loaded = params[0];
                break;
            case SERVO_LOAD_OR_UNLOAD_READ_ID:
                write_loaded();
                break;
            case SERVO_LED_CTRL_WRITE_ID:
                led = params[0];
                digitalWrite(A4, led);
                break;
            case SERVO_LED_CTRL_READ_ID:
                write_led();
                break;
            case SERVO_MOVE_TIME_WRITE_ID:{
                int32_t pos = params[0] | ((params[1]) << 8); // get 16 bit unsigned value
                pos = pos > 32767 ? pos - 65536 : pos; // make it negative if overflowed
                set_point = fmap(pos, -250, 1250, 0, 360);  // -250..1250 -> 0..360
                // time is ignored
                break;
            }
            case SERVO_MOVE_TIME_READ_ID:
                write_move_time();
                break;
            case SERVO_CUR_READ_ID:
                write_cur();
                break;
            case SERVO_TORQUE_LIMIT_WRITE_ID:
                max_torque = fmap(params[0] | ((params[1]) << 8), 0, 2000, 0, 2);
                break;
            case SERVO_TORQUE_LIMIT_READ_ID:
                write_torque_limit();
                break;
            case SERVO_POS_PARAMS_WRITE_ID:
                pos_kc = (params[0] | ((params[1]) << 8)) / 100.f;
                pos_inv_ti = (params[2] | ((params[3]) << 8)) / 100.f;
                eeprom_write_byte(EEPROM_POS_KC_L, params[0]);
                eeprom_write_byte(EEPROM_POS_KC_H, params[1]);
                eeprom_write_byte(EEPROM_POS_INV_TI_L, params[2]);
                eeprom_write_byte(EEPROM_POS_INV_TI_H, params[3]);
                break;
            case SERVO_POS_PARAMS_READ_ID:
                write_pos_params();
                break;
            case SERVO_TORQUE_PARAMS_WRITE_ID:
                torque_kc = (params[0] | ((params[1]) << 8)) / 100.f;
                torque_inv_ti = (params[2] | ((params[3]) << 8)) / 100.f;
                eeprom_write_byte(EEPROM_TORQUE_KC_L, params[0]);
                eeprom_write_byte(EEPROM_TORQUE_KC_H, params[1]);
                eeprom_write_byte(EEPROM_TORQUE_INV_TI_L, params[2]);
                eeprom_write_byte(EEPROM_TORQUE_INV_TI_H, params[3]);
                break;
            case SERVO_TORQUE_PARAMS_READ_ID:
                write_torque_params();
                break;
        }
    } else if(_id == 0xFE && cmd == SERVO_ID_READ_ID) {
        write_id();
    }   
}