#include <Arduino.h>
#include "protocol.h"
#include "pi.h"

#define F_CPU 16000000L

handler proto; // protocol handler instance
pi_controller pos_controller;
pi_controller cur_controller;

// hardware control
void set_motor(int16_t val){
    if(val > 0){
        OCR1A = 0;
        OCR1B = val;
    } else {
        OCR1A = -val;
        OCR1B = 0;
    }
}

float get_pos(){
    return fmap(analogRead(A1), 0, 1023, 0, 360);
}

float get_cur(){
    return fmap(analogRead(A0), 0, 1023, -12.5, 12.5);
}

// control loop update isr
ISR(TIMER2_COMPA_vect){
    PORTC |= _BV(PC4);

    float error = proto.set_point - get_pos();
    set_motor(pos_controller.update(error));

    PORTC &= ~(_BV(PC4));
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

    // init timer interrupt for the main control loop
    TCCR2A = _BV(WGM21);   // set timer to ctc mode
    TCCR2B = _BV(CS22) | _BV(CS21);    // set prescaler to 32
    OCR2A = 0x7C;          // set compare match threshold to obtain 500Hz
    TIMSK2 |= _BV(OCIE2A); // enable timer 2 overflow interrupt

    // initialize pi controllers
    pos_controller.kc = 0.1;
    pos_controller.ti = INFINITY;
    cur_controller.kc = 0.;
    cur_controller.ti = INFINITY;

    while(1){
        proto.handle();
    }
    return 0;
}
