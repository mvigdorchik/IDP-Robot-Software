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

class turntable
{
public:
	turntable();
	void turn(bool clockwise, int speed);
	void turn_to_nest(int current_nest, int next_nest);
	void turn_angle_pid(bool clockwise, int degrees);
	void turn_angle_time(bool clockwise, int degrees);
	void initial_align();
	unsigned char read_sensor();


private:
	int current_nest;
};

#endif /* TURNTABLE_H */
