#ifndef TURNTABLE_H
#define TURNTABLE_H


#include <iostream>
#include <bitset>
#include <robot_instr.h>
#include <robot_link.h>
#include <stopwatch.h>

#define TURNTABLE_ROTATION_CALIBRATION 14 //Converts angles into motor times by multiplying, assuming max speed
#define TURNTABLE_Ki 0.5 //Integral parameter for PID controller
#define TURNTABLE_Kp 0.1 //Proportional parameter for PID controller
#define TURNTABLE_Kd 0.01 //Derivative parameter for PID controller
#define TURNTABLE_dt 0.1 //Timestep for PID controller
#define TURNTABLE_tol 0.5 //Tolerance for ending turning

extern robot_link rlink;

class turntable
{
public:
	turntable();
	void turn_angle(bool clockwise, int degrees);
	void grab_nest(int nest_number);
	void turn(bool clockwise, int speed);

private:
}

#endif /* TURNTABLE_H */
