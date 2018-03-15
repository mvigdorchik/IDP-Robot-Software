#ifndef ROBOT_H
#define ROBOT_H

#include <iostream>
#include <bitset>
#include <robot_instr.h>
#include <robot_link.h>
#include <robot_delay.h>
#include <stopwatch.h>
#include <map>
#include "point.h"
#include "line.h"

#define ROBOT_RAMP_TIME 255 //Value from 0-254 (255 is default value) to increase time to ramp up
#define ROTATION_CALIBRATION 16 //Converts angles into motor times by multiplying, assuming max speed
#define DISTANCE_CALIBRATION 20.0 //Converts distances into motor times, assuming speed is set to 127
#define INERTIA_CALIBRATION 1 //Number of mm that robot travels after stopping from full speed
#define DISTANCE_TO_CENTER 40 //Distance in mm to center of rotation from the line sensors
#define FOLLOWER_KP 40 //determines how much line follower needs to react when one sensor off line
#define FOLLOWER_KP2 80 //Determines how much gain when 2 sensors are off the line
#define FOLLOWER_KI 0.3 //Integral control for the line follower

/**
 * Object used to send commands and receive sensor readings from the robot.
 */
extern robot_link rlink;
extern std::map<std::string, point> junction_points;

/**
 * An enumerator that defines all of the different states available to the robot.
 * They determine the behavior of the robot and will transition between each other
 * as necessary. For Example, the GO_TO_PICKUP state will navigate the robot to the pickup
 * area and then move on to the COLLECT_EGGS state after aligning itself appropriately.
 */
enum State
{
    DEFAULT,
    GO_TO_PICKUP,
    GO_TO_RECYCLING,
    GO_TO_START,
    COLLECT_EGGS,
    REFILL_EGGS,
    DEPOSIT_EGGS,
    FAILED_SENSORS,
    LOST_LINE
};

enum Direction
{
    WEST = 0,
    EAST = 1,
    NORTH = 2,
    SOUTH = 3,
    LOST = -1
};

/**
 * This class stores the fundamental state of the robot as well as its current location.
 * The functions in this class include all of the line following and motion of the robot,
 * as well as any supplementary function to those features such as reading sensors. It also contains functions
 * that carry out the task of its assigned state.
 *  @see State 
 */
class robot
{
public:
    /**
     * Default Constructor for robot will set the default state and start location.
     */
    robot();
    
    /**
     * Uses a discrete PI controller and three line sensors to follow a line.
     * If everything goes well it will stop once it reaches a junction, immediately
     * executing the next action. If there is a delay after this function is called,
     * there will be some inertia and the robot will overshoot the line. This function
     * will also automatically try to adjust for losing the line and recover, and if
     * it does not rapidly recover or it has to move much more or less than expected distance,
     * it will enter the LOST_LINE state and attempt to recover. The parameter recover allows
     * disabling this and considering the function complete as soon as it loses the line.

     * @param expected_distance the approximate distance to next line
     * @param recover If set to false the robot will just stop if its lost, no recovery attempt.
     * @see read_line_sensors() 
     */
    void follow_line_straight(int expected_distance, bool recover);

    /**
     * Specify a c-style array and it size where each element is an int from 0-3
     * The robot will then follow a path corresponding to the array, where 0 is turn left,
     * 1 is turn right, 2 is go straight, and 3 is turn around. The first element is evaluated right away
     * and then following ones are done at each junction.

     * @param path A list of all the turns the robot will take
     * @param size The size of the path array
     * @see follow_line_straight()
     * @see turn_to_line()
     */
    void take_path(int path[], int size);

    /**
     * The robot will turn in place for a given duration calculated by multipling
     * degrees by a calibration constant defined in the header. It will account for intertia
     * based on another calibration constant. If the angle is greater than 180, it turns the other way.
 
     * @param degrees The amount of degrees to turn in the clockwise sense
     */
    void turn_angle(int degrees);

    /**
     * This will turn the robot in a given direction until it intersects a line.
     * If not told to move forward afterwards, inertia will carry it past the line.
     * It can move forward before turning to turn in a junction. Amount is based on calibration constant.

     * @param right Determines which direction the robot turns.
     * @param move_forward Determines if robot should move forward BEFORE turning.
     * @param delay_sensing Delays line detection to prevent accidently stopping on nearby lines.
     * @return Returns true if it found a line before rotating a full 360 degrees.
     */
    bool turn_to_line(bool right, bool move_forward, bool delay_sensing);

    /**
     * Go straight for a given distance at a given speed

     * @param distance Distance in mm to move forward
     * @param speed Speed from 0-127 to go forward and 128-255 to reverse.
     * @param use_inertia Determines if the function should consider inerta or ignore it
     */
    void go_time(int distance, unsigned char speed, bool use_inertia);

    /**
     * Returns the value of the three line sensors as the 3 least significant bits.
     * The LSB is rightmost sensor and the MSB (of the 3) is the leftmost sensor, and remaining is center sensor.
     * The actual connection is on READ_PORT_0, the 3 least significant bits.

     * @return An unsigned char containing the three line sensor values 
     */
    unsigned char read_line_sensors();

    /**
     * Robot goes forward at the specified speed until another command changing speed occurs

     * @param speed Speed from 0-127 to go forward and 128-255 to reverse.
     */
    void go(unsigned char speed);

    /**
     * Moves forward at maximum speed until a line is detected. Does not account for inertia.
     * If a line is not found in time, it will turn an arbitrary amount and try again. It will continue
     * until a line is found.

     * @param timeout Time in ms before the robot tries to search a different direction.
     * @see go()
     */
    void go_to_line(int timeout);

    /**
     * Special function to go around the curve following the line and then transfer to the line across it.
     * This avoids having to go up a hill or in between the two blocks that the robot will not be able to
     * fit between. It should always end up north of the blocks at the next junction.

     * @pre The robot must be at the junction right before the curve facing the curve.
     */
    void traverse_curve();

    /**
     * Will turn into the curved line, and then continue to the next junction as normal.
     * This is assumed to be run from JD1 facing east

     * @pre Robot is facing East at JD1
     */
    void return_to_curve();

    /**
     * Uses a slightly backwards line follow algorithm to be able to reverse.
     * This may not be needed but it can help with leaving space to allow collection point to be refilled.

     * @param distance Distance in mm that the robot should move along the line.
     * @see follow_line_straight()
     */
    void line_follow_reverse(int distance);

    /**
     * Moves the robot away from the palette for a time (at least 400mm) to let eggs refill.

     * @param time The time to wait after moving away.
     * @pre The robot must start aligned to the palette.
     */
    void allow_egg_refill(int time);

    /**
     * Turns the robot at a given speed while still maintaining the forward motion.
     * Works by slowing down the speed of the appropriate motor. It will never turn faster
     * than stopping the second motor.  
     
     * @param right Determines direction of the turn.
     * @param speed Determines amount to slow down other motor.
     */
    void turn(bool right, unsigned char speed, bool reverse);

    /**
     * Attempts to recover the line if it was lost. It will first try turning
     * in the direction the line was lost, determined by latest_reading. If it cannot,
     * it will then try to go forward until it finds a line. If it takes too long to find a line,
     * it will consider itself lost. This function is fairly volatile at the moment and may change specifications

     * @param old_loc Best estimate of the robot's current location.
     * @param latest_reading The most recent line sensor readings before getting lost, to determine direction.
     * @return The newest best guess of current location after attempting to recover.
     * @see go_to_line()
     */
    std::string recover_line(std::string old_loc, unsigned char latest_reading);

    /**
     * Reads the beacon at the very start of the competition to determine the nest
     * Composition

     * @return An integer value corresponding to the beacon code.
     */
    int read_beacon();
    
    /**
     * Current state of the robot for the purpose of overall strategy.
     */
    State state;

    /**
     * The location the robot believes is its most recently visited junction.
     */
    std::string current_loc;

    /**
     * Stores the current orientation of the vehicle as an unsigned integer.
     * 0 corresponds to west, 1 to east, 2 to north, and 3 to south.
     */
    Direction orientation;
};



#endif /* ROBOT_H */
