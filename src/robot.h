#ifndef ROBOT_H
#define ROBOT_H

#include <iostream>
#include <bitset>
#include <robot_instr.h>
#include <robot_link.h>
#include <robot_delay.h>
#include <stopwatch.h>

#define RAMP_TIME 255 //Value from 0-254 (255 is default value) to increase time to ramp up
#define ROTATION_CALIBRATION 10 //Converts angles into motor times by multiplying, assuming max speed
#define DISTANCE_CALIBRATION 1/127 //Converts distances into motor times, assuming speed is set to 1, so must multiply by speed
#define FOLLOWER_GAIN 40 //determines how much line follower needs to react

extern robot_link rlink;

enum State
{
    DEFAULT,
    GO_TO_PICKUP 
};

class robot
{
public:
    robot();
    void follow_line_straight(int distance);
    void turn_angle(int degrees);
    void go_time(int distance, unsigned char speed);
    unsigned char read_line_sensors();
    void go(unsigned char speed);
    void turn(bool right, unsigned char speed);
    State state;
};


#endif /* ROBOT_H */
