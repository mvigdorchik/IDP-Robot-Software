#ifndef ROBOT_H
#define ROBOT_H

#include <iostream>
#include <bitset>
#include <robot_instr.h>
#include <robot_link.h>
#include <robot_delay.h>
#include <stopwatch.h>
#include "table.h"

#define ROBOT_RAMP_TIME 255 //Value from 0-254 (255 is default value) to increase time to ramp up
#define ROTATION_CALIBRATION 14 //Converts angles into motor times by multiplying, assuming max speed
#define DISTANCE_CALIBRATION 20.0 //Converts distances into motor times, assuming speed is set to 127
#define INERTIA_CALIBRATION 20 //Number of mm that robot travels after stopping from full speed
#define DISTANCE_TO_CENTER 60 //Distance in mm to center of rotation from the line sensors
#define FOLLOWER_KP 40 //determines how much line follower needs to react when one sensor off line
#define FOLLOWER_KP2 72 //Determines how much gain when 2 sensors are off the line
#define FOLLOWER_KI 0.2 //Integral control for the line follower

extern robot_link rlink;

enum State
{
    DEFAULT,
    GO_TO_PICKUP,
    LOST_LINE
};

class robot
{
public:
    robot();
    void follow_line_straight(int expected_distance);
    void take_path(int path[], int size);
    void turn_angle(int degrees);
    bool turn_to_line(bool right, bool move_forward, bool delay_sensing);
    void go_time(int distance, unsigned char speed);
    unsigned char read_line_sensors();
    void go(unsigned char speed);
    void go_to_line(int timeout);
    void turn(bool right, unsigned char speed);
    std::string recover_line(std::string old_loc, unsigned char latest_reading);
    
    State state;
    table map;
    std::string current_loc;
};


#endif /* ROBOT_H */
