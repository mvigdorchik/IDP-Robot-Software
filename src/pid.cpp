#include "pid.h"

PID::PID(double dt, double max, double min, double Kp, double Kd, double Ki):
	dt(dt), 
	max(max),
	min(min),
	Kp(Kp),
	Kd(Kd),
	Ki(Ki),
	pre_error(0),
	integral(0)
{
	
}
double PID::calculate(double setpoint, double pv)
{
	// Calculate error
	double error = setpoint - pv;

	// Proportional term
	double Pout = Kp * error;

	// Integral term
	integral += error * dt;
	double Iout = Ki * integral;

	// Derivative term
	double derivative = (error - pre_error) / dt;
	double Dout = Kd * derivative;

	// Calculate total output
	double output = Pout + Iout + Dout;

	// Restrict to max/min
	if (output > max)
		output = max;
	else if (output < min)
		output = min;

	// Save error to previous error
	pre_error = error;

	return output;
}