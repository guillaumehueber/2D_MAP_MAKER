#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include "main.h"

//This structure is what we're going to send to the computer
struct dist_rot{
    float pos_x;
    float pos_y;
    float angle;
    float distance;
};

//Inits the motors, values and motor control thread
void init_motor_controller(void);


#endif /* MOTOR_CONTROLLER_H */

