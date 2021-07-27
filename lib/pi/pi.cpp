#include <Arduino.h>

#include "pi.h"

float pi_controller::update(float error){
    float dt = (micros() - prev_time) / 1.e6f;
    integral += error * dt;
    return kc * (error + (1 / ti) * integral);
}