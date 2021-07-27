#include <Arduino.h>
#include "protocol.h"

#define F_CPU 16000000L

handler proto; // protocol handler instance

float pos_integral = 0;
float cur_integral = 0;

// hardware control
void set_motor(int16_t val){
    // clip to working range
    val = val > 255 ? 255 : val;
    val = val < -255 ? -255 : val;
    if(val > 0){
        OCR1A = 0;
        OCR1B = val;
    } else {
        OCR1A = -val;
        OCR1B = 0;
    }
}

// control loop update isr
ISR(TIMER2_COMPA_vect){
    PORTC |= _BV(PC4);

    float pos_error = get_pos() - proto.set_point;
    // float torque_error = get_torque() - proto.max_torque;

    pos_integral += pos_error * 0.002;  // dt is constant and equal to 1/500Hz = 0.002s
    float pos_out = proto.pos_kc * (pos_error + proto.pos_inv_ti * pos_integral);
    set_motor(pos_out);

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

    while(1){
        proto.handle();
    }
    return 0;
}
