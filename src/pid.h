#ifndef PID_H
#define PID_H
#include <iostream>

class PID
{
public:
	// Kp -  proportional gain
	// Ki -  Integral gain
	// Kd -  derivative gain
	// dt -  loop interval time
	// max - maximum value of manipulated variable
	// min - minimum value of manipulated variable
	PID(double dt, int max, int min, double Kp, double Kd, double Ki);

	// Returns the manipulated variable given a setpoint and current process value
	double calculate(double setpoint, double pv);

private:
	double dt;
	int max;
	int min;
	double Kp;
	double Kd;
	double Ki;
	double pre_error;
	double integral;
};

#endif // PID_H