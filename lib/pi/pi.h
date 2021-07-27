#ifndef PI_H
#define PI_H

class pi_controller{
    float ti; // integral time constant
    float kc; // controller gain
    float setpoint;
    float integral;
    unsigned long prev_time;

    public:
        float update(float input);
};

#endif