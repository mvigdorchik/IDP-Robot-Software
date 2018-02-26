#ifndef TURNTABLE_H
#define TURNTABLE_H


#include <iostream>
#include <bitset>
#include <robot_instr.h>
#include <robot_link.h>
#include <cmath>
#include <stopwatch.h>

#define TURNTABLE_ROTATION_CALIBRATION 1.88976377953 //Converts angular speed from degrees/s into motor speed
#define TURNTABLE_ROTATION_SPEED 0.01 //Converts from degrees to time
#define TURNTABLE_INERTIA_CALIBRATION //Number of degrees it makes more after stopping from full speed
#define TURNTABLE_Ki 0.5 //Integral parameter for PID controller
#define TURNTABLE_Kp 0.1 //Proportional parameter for PID controller
#define TURNTABLE_Kd 0.01 //Derivative parameter for PID controller
#define TURNTABLE_dt 0.1 //Timestep for PID controller
#define TURNTABLE_tol 0.5 //Tolerance for ending turning
#define DOUBLE_LINE_TIMEOUT 4000 //Timeout for detecting a new line
#define MIN_TIME_DETECTION 2000 //Min time so that it doesn't register the same line twice
#define TIME_TO_REVERSE 5000 //Time to center on the nest for initial alignment
#define TOTAL_NUMBER_NESTS 8 //Number of nests

extern robot_link rlink;

/**
 * This class stores all of the values and functions related to sorting eggs. This includes
 * functions to catagorize eggs as they are collected, moving the onboard turntable, tracking
 * all eggs on board as well as required eggs, and delivering eggs.
 */
class turntable
{
public:
    /**
     * Default constructor, initializing the state of the turntable
     */
    turntable();

    /**
     * Turns the onboard turntable a specified direction and speed.

     * @param clockwise Determines direction of rotation.
     * @param speed Determines speed, from 0-127.
     */
    void turn(bool clockwise, int speed);

    /**
     * Turns the turntable from the current nest to to desired nest.
     
     * @param next_nest Number of the desired nest to deposit into.
     */
    void turn_to_nest(int next_nest);

    /**
     * Turns the turntable a given angle using a PID controller and a continuous sensor reading.
     * This function currently will not work as there is no potentiostat on board, but may be useful in the future.

     * @param clockwise Determines direction to turn.
     * @param degrees Number of degrees to turn.
     */
    void turn_angle_pid(bool clockwise, int degrees);

    /**
     * Turns the turntable for a given number of degrees based on time and a calibration constant.

     * @param clockwise Determines the direction to turn.
     * @param degrees Number of degrees to turn.
     */
    void turn_angle_time(bool clockwise, int degrees);

    /**
     * Lines up the turntable with the first nest based on white lines drawn on the turntable.
     * Since there is no potentiometer in the supplies, it is only able to determine its position by first
     * turning the the spot where there are 3 white lines, and then counting lines it intersects to get to the next nest.
     
     * @see turn_to_nest()
     */
    void initial_align();

    /**
     * Reads the 4th line following sensor used to determine the position of the turtable and receive feedback.
     * This is the 4th LSB in READ_PORT_0. This value is either 1 or 0.

     * @return A 1 if there is a line above the sensor or a 0 otherwise.
     */
    unsigned char read_sensor();
    
    
private:
    int current_nest;
};

#endif /* TURNTABLE_H */
