#ifndef PI_H
#define PI_H

class pi_controller{
    public:
        float ti; // integral time constant
        float kc; // controller gain
        float integral;
        unsigned long prev_time;

        float update(float input);
};

#endif