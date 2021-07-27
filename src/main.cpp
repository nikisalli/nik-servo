#include <Arduino.h>
#include "protocol.h"
#include "pi.h"

#define F_CPU 16000000L

// motor control
void set_motor(int16_t val){
    if(val > 0){
        OCR1A = 0;
        OCR1B = val;
    } else {
        OCR1A = -val;
        OCR1B = 0;
    }
}

int main(void){
    // init arduino framework
    init();

    // init uart
    Serial.begin(115200);

    // init pinout
    pinMode(9,  OUTPUT); // motor control A
    pinMode(10, OUTPUT); // motor control B
    pinMode(A3, OUTPUT); // uart tx-rx controller switch
    pinMode(A4, OUTPUT); // onboard led
    pinMode(A1, INPUT);  // position feedback
    pinMode(A0, INPUT);  // current feedback
    pinMode(A6, INPUT);  // vcc/2 calibration input

    // init servo pwm timer
    TCCR1A = _BV (WGM10) | _BV (WGM12) | _BV (COM1A1) | _BV (COM1B1);
    TCCR1B &= 0b11111000;  // clear prescaler
    TCCR1B |= _BV (CS10);  // prescaler of 8
    set_motor(0);
    

    handler proto; // protocol handler instance

    while(1){
        proto.handle();
    }
    return 0;
}
