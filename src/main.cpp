#include <Arduino.h>
#include "protocol.h"

#define F_CPU 16000000L

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
    init();
    TCCR1A = _BV (WGM10) | _BV (WGM12) | _BV (COM1A1) | _BV (COM1B1);
    TCCR1B &= ~7;  // clear prescaler
    TCCR1B |= _BV (CS11);  // prescaler of 8
    pinMode (9, OUTPUT);
    pinMode (10, OUTPUT);

    handler proto; // protocol handler instance

    while(1){
        proto.handle();
    }
    return 0;
}
