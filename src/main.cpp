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
    val += 255;
    OCR1B = val / 2;
    OCR1A = 255 - ((val + 1) / 2);
}

// pwm overflow isr
ISR(ADC_vect){
    proto.current = ADC;
}

// timer 1 overflow isr
ISR(TIMER1_OVF_vect){
    TIFR1 |= _BV (TOV1); // manually clear timer 1 interrupt flag
}

int main(void){
    // init arduino framework
    init();

    // init motor
    DDRB |= _BV (PB1) | _BV (PB2); // set pwm pins to output

    // init uart
    DDRC |= _BV (PC3);
    Serial.begin(115200);

    // init led
    DDRC |= _BV (PC4);

    // init servo pwm timer
    TCCR1A = _BV (COM1A1) | _BV (COM1B1); // set output high on compare match
    TCCR1B &= 0b11000000;  // clear prescaler and wgm
    TCCR1B |= _BV (WGM13); // set waveform generation mode to phase and frequency correct
    TCCR1B |= _BV (CS10);  // prescaler of 8
    ICR1 = 0xFF;           // TOP value
    TIMSK1 |= _BV (TOIE1); // enable timer 1 overflow interrupt

    // init adc
    PORTC &= ~(_BV (PC0) | _BV(PC1)); // disable pullups
    DDRC &= ~(_BV (PC0) | _BV(PC1)); // set adc pins to input
    ADCSRA = _BV (ADEN) // enable adc
           | _BV (ADATE) // enable auto adc triggering
           | _BV (ADIE) // enable adc end conversion interrupt
           | _BV (ADIF) // clear adc interrupt
           | _BV (ADPS0) // set prescaler to 128 to get 125khz adc clock
           | _BV (ADPS1)
           | _BV (ADPS2); 
    ADCSRB = _BV (ADTS1) | _BV (ADTS2); // adc auto trigger on timer 1 overflow
    DIDR0 = _BV (ADC0D) | _BV (ADC1D); // disable input port logic for adc gpios
    ADMUX = _BV (REFS0); // set ref to avcc and mux to ADC0
    ADCSRA |= _BV (ADSC); // let the hardware initialize itself by starting the first 25 clock cycle conversion

    unsigned long prev_time = 0;

    while(1){
        if(micros() - prev_time > 2000){
            prev_time = micros();
            // read and update position
            ADCSRA &= ~(_BV (ADATE) | _BV (ADIE)); // disable auto triggering and adc interrupt vector
            while(ADCSRA & _BV (ADSC)); // wait for conversion to end
            ADMUX |= _BV (MUX0); // set mux to PC1 gpio
            ADCSRA |= _BV (ADSC); // manually trigger the adc
            while(ADCSRA & _BV (ADSC)); // wait for conversion to end
            proto.position = ADC; // read value
            ADMUX &= ~(_BV (MUX0)); // reset mux to PC0 gpio
            ADCSRA |= _BV (ADATE) | _BV (ADIE); // reenable auto triggering and adc interrupt vector

            float pos_error = (proto.position / 2.841666666f) - proto.set_point;
            // float torque_error = proto.torque - proto.max_torque;

            pos_integral += pos_error * 0.002;  // dt is constant and equal to 1/500Hz = 0.002s
            float pos_out = proto.pos_kc * (pos_error + proto.pos_inv_ti * pos_integral);
            set_motor(pos_out);
        }
        proto.handle();
    }
    return 0;
}
