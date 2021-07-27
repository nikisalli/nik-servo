#include <avr/io.h>
#include <avr/eeprom.h>
#include <avr/wdt.h>

#include "protocol.h"

// helpers
uint8_t cpl2int(uint8_t val){
    return 0x80 & val ? (0x7F & val) - 0x80 : val;  // fast inverse 2's complement
}

uint8_t int2cpl(uint8_t val){
    return val > 0 ? val : (~val) + 1;
}

// uart handling
void servo_tx_enb() {
    PORTC |= _BV(PC3);
}

void servo_rx_enb() {
	PORTC &= ~_BV(PC3);
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

// packets

handler::handler(){
    Serial.begin(115200); // initialize uart at 1Mbaud
    pinMode(A3, OUTPUT); // uart tx-rx controller switch
    pinMode(A4, OUTPUT);
    servo_rx_enb();
    pinMode(A1, INPUT); // position feedback
    // read stored data from eeprom
    id = eeprom_read_byte((uint8_t*)EEPROM_ID);
    angle_offset = cpl2int(eeprom_read_byte((uint8_t*)EEPROM_ANGLE_OFFSET));
}

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
    // int16_t intpos = (map(analogRead(A1), 0, 1023, -250, 1250)); // 0..240 to 0..1000
    uint16_t uintpos = (uint16_t)(map(analogRead(A1), 0, 1023, -250, 1250));
    servo_write(
        id,
        SERVO_POS_READ_LENGTH_RX,
        SERVO_POS_READ_ID,
        (uint8_t)(uintpos & 0xFF),
        (uint8_t)(uintpos >> 8)
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
        led
	);
}

// handler
void handler::handle(){
    while(!Serial.available());              // wait for one byte at least
    if(Serial.read() != HEADER_BYTE) return; // match first header byte
    while(!Serial.available());              // wait for another byte
    if(Serial.read() != HEADER_BYTE) return; // match second header byte
    while(!Serial.available());              // wait for another byte
    uint8_t res = Serial.read();             // if the header is matched get id
    if(res != id && res != SERVO_BROADCAST_ID) return; // check if the command is directed to us or broadcasted

    while(Serial.available() < 2);          // wait for length and cmd if still not there
    uint8_t length = Serial.read();         // read length
    uint8_t cmd = Serial.read();            // read cmd id
    uint8_t _id = res;

    uint8_t params[7] = {};                    // read n parameters

    uint8_t sum = _id + length + cmd;

    while(Serial.available() < length - 2);  //wait for remaining bytes

    for(uint8_t i = 0; i < length - 2; i++){ //iterate until the packet has been fully read
        params[i] = Serial.read();
        if(i != length - 3) sum += params[i];   //do not sum checksum
    }

    // delayMicroseconds(100);

    if(((~sum) & 0xFF) != (params[length - 3] & 0xFF)) return; //match checksum
    
    if (_id == id) { // if the command is directed to us
        switch (cmd) {
            case SERVO_ID_WRITE_ID: // set id
                eeprom_write_byte((uint8_t*)EEPROM_ID, params[0]); // parameter 1 is our new id
                id = params[0]; // update our id
                break;
            case SERVO_ANGLE_OFFSET_ADJUST_ID: // set angle offset
                angle_offset = cpl2int(params[0]);
                break;
            case SERVO_ANGLE_OFFSET_WRITE_ID: // write angle offset to eeprom
                eeprom_write_byte((uint8_t*)EEPROM_ANGLE_OFFSET, int2cpl(angle_offset));
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
        }
    } else if(_id == 0xFE && cmd == SERVO_ID_READ_ID) {
        write_id();
    }   
}