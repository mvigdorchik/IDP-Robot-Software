#ifndef PID_H
#define PID_H
#include <iostream>

/**
 * A Class for a general purpose PID controller to use with any motor or actuator
 * that has some form of feedback this is not discrete like line following is.
 */
class PID
{
public:
    /**
     * Constructor for the PID class. Sets all of the standard PID controller constants
     * as well as a maximum and minimum output (for motor limits). Since data is taken at 
     * discrete times, a time delay value between sensor readings is also needed.
     
     * @param dt Time between sensor readings.
     * @param max Maximum output value.
     * @param min Minimum output value.
     * @param Kp Proportional control constant.
     * @param kd Derivative control constant.
     * @param ki Integral control constant. 
     */
    PID(double dt, double max, double min, double Kp, double Kd, double Ki);

    /**
     * Calculates the output required based on the PID control parameters. 
     
     * @param setpoint The target value requested.
     * @param pv The actual measured value.
     * @return The output of the PID loop (e.g. requested motor speed)
     */
    double calculate(double setpoint, double pv);

private:
    double dt;
    double max;
    double min;
    double Kp;
    double Kd;
    double Ki;
    double pre_error;
    double integral;
};

#endif // PID_H
