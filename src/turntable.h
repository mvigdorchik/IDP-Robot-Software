#ifndef TURNTABLE_H
#define TURNTABLE_H


#include <iostream>
#include <bitset>
#include <robot_instr.h>
#include <robot_link.h>
#include <robot_delay.h>
#include <cmath>
#include <vector>
#include <stopwatch.h>

#define DEBUG 1 //If defined all of the print code will run, otherwise it won't

#define TURNTABLE_RAMP_TIME 10 //Ramp time for the turntable motor
#define TURNTABLE_ROTATION_CALIBRATION 1.85 //Converts angular speed from degrees/s into motor speed
#define TURNTABLE_ROTATION_SPEED 0.01 //Converts from degrees to time
#define TURNTABLE_INERTIA_CALIBRATION 10 //Number of degrees it makes more after stopping from full speed
#define TURNTABLE_Ki 0.0 //Integral parameter for PID controller
#define TURNTABLE_Kp 12.0 //Proportional parameter for PID controller
#define TURNTABLE_Kd 0.0 //Derivative parameter for PID controller
#define TURNTABLE_dt 5 //Timestep for PID controller
#define TURNTABLE_tol 1.0 //Tolerance for ending turning
#define TURNTABLE_SLOW_SPEED 40 // Speed set to reverse slowly for initial alignment
#define TURNTABLE_FIRST_DECREASED_SPEED 80 // Value of first speed reduced, i.e. at one line before target
#define TURNTABLE_SECOND_DECREASED_SPEED 40 // Value of second speed reduced, i.e. at one white space before target
#define DOUBLE_LINE_TIMEOUT_SMALL 83 //Timeout for detecting a new line on turntable with thin lines
#define DOUBLE_LINE_TIMEOUT_BIG 167 //Timeout for detecting a new line on turntable with thick lines
#define DOUBLE_LINE_TIMEOUT_CALIBRATION 96 //Timeout for detecting a new line on turtntable with thin lines and calibration lines
#define MIN_TIME_DETECTION_SMALL 27 //Min time so that it doesn't register the same thin line twice
#define MIN_TIME_DETECTION_BIG 47 //Min time so that it doesn't register the same thick line twice
#define MIN_TIME_DETECTION_CALIBRATION 23 //Min time so that it doesn't register the same calibration line twice
#define TIME_TO_REVERSE 130 //Time to center on the nest for initial alignment
#define TOTAL_NUMBER_NESTS 9 //Number of nests
#define BLACK_HI 1 // If set to 1, sensors will give 1 when see black
#define POT_START_OFFSET 5 // Value indicated by pot at the first nest
#define POT_MAX_VALUE 237 // The limit of POT corresponding to 360 degrees
#define COLLECT_TO_PUSH_OFFSET 66 //ADC reading difference between collecting and depositing nest
extern robot_link rlink;

/**
* Enum to handle the four different kinds of eggs.
*/
enum Egg
{
	SMALL_YELLOW,
	BIG_YELLOW,
	SMALL_BLUE,
	BIG_PINK
};


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
    * Turns the turntable from the current nest to to desired nest.

    * @param next_nest Number of the desired nest to deposit into.
    */
    void turn_to_nest_thin(int next_nest);

    /**
    * Turns the turntable from the current nest to to desired nest.

    * @param next_nest Number of the desired nest to deposit into.
    */
    void turn_to_nest_pid(int next_nest);

    /**
     * Same as turn_to_nest_pid but it aligns with the pusher instead

     * @param next_nest Which nest to align with ejector
     * @see turn_to_nest_pid()
     */
    void turn_to_push_pid(int next_nest);

    /**
    * Turns the turntable a given angle using a PID controller and a continuous sensor reading.
    * This function currently will not work as there is no potentiostat on board, but may be useful in the future.

    * @param clockwise Determines direction to turn.
    * @param degrees Number of degrees to turn.
    */
    void turn_angle_pid(int target_angle);

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

    /**
    * Reads the value of the pot resistance and converts it in angle position of the turntable.

    * @return An int between 0 and 360.
    */
    int read_pot();

    /**
    * Determines the type of egg currently in the bucket.

    * @return A value from 0-3 in the form of the Egg enum.
    * @see Egg
    */
    Egg measure_egg_type();

    /**
    * Determines the nest that best fits the current egg. Uses the current contents of the nests
    * and the desired composition of nests as well as the current turntable position to minimize travel.

    * @param egg_type Which egg is currently in the bucket to then move to a nest.
    */
    int determine_nest(Egg egg_type);

    /**
    * Will measure the egg currently in the basket and then place it in the appropriate nest.

    * @see determine_nest()
    * @see measure_egg_type()
    * @see move_arm()
    */
    void place_egg();

    /**
     * This function will push the nest currently under the ejector off the ramp then retract after
     */
    void push_nest();
    
    /**
    * A list of all of the nests on the turntable. Index corresponds to the position of the nest.
    * Each nest is represented by a vector containing arbitrary number of eggs. In reality,
    * there will be a limit to how many small and big eggs fit, determined by experimentation.
    */
    std::vector<Egg> nests[TOTAL_NUMBER_NESTS];

private:
    /**
     * The nest the sorting bucket is currently over, corresponding to turntable rotation.
     */
    int current_nest;
};

#endif /* TURNTABLE_H */
