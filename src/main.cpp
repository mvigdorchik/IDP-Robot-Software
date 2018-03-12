#include <iostream>
#include <bitset>
#include <robot_instr.h>
#include <robot_link.h>
#include <robot_delay.h>
#include <stopwatch.h>
#include "robot.h"
#include "turntable.h"
#include "arm.h"

#define ROBOT_NUM 2
#define DEBUG 1 //If defined all of the print code will run, otherwise it won't

void start_to_pallette();
void demo_follow_line();
void temp_turn_table(int nests);
void push_egg(bool position);
void demo_turntable();

robot_link rlink;
robot r;
turntable t;
/**
 * Datastructure to store the map of the field. Completely static.
 */
std::map<std::string, point> junctions; 

int main()
{
#ifndef __arm__
	if(!rlink.initialise(ROBOT_NUM))
	{
	    std::cout << "Can't find robit" << std::endl;
	    return -1;
	}    
#else
	if(!rlink.initialise())
	{
	    //Pray to your local deity
	    
	    return -1;
	}
#endif
	if(DEBUG) std::cout << "Successful Connection" << std::endl;
/*
	//Define the map of the table, followed by the nodes
	junctions["J1"] = point(300, 793);
	junctions["J2"] = point(1200, 793);
	junctions["J3"] = point(1200, 280);
	junctions["J4"] = point(1810, 280);
	junctions["J5"] = point(2100, 280);
	junctions["J6"] = point(2100, 793);
	junctions["D1"] = point(1790, 2350);
	junctions["D2"] = point(1200, 2350);
	junctions["D3"] = point(300, 2350);
	junctions["Start"] = point(300, 280);
	junctions["JD1"] = point(1790, 0);
	junctions["JD2"] = point(1200, 0);
	junctions["JD3"] = point(300, 0);
	junctions["JC2"] = point(1733, 1543);
	junctions["H1"] = point(300, 0);
	junctions["H2"] = point(300, 0);
	junctions["H3"] = point(300, 0);
	junctions["H4"] = point(300, 0);
	junctions["H5"] = point(300, 0); //TODO Verify measurements for points where coordinates are 0
*/

	rlink.command(RAMP_TIME, ROBOT_RAMP_TIME);
	std::cout << "Initialise turntable" << std::endl;
	turntable t;
	//	while(1)
	//    {
	//	std::cout << "ADC0 " <<rlink.request(ADC0)<< std::endl;
	//	}
	// arm a;
	// temp_turn_table(1);
	// delay(1000);
	// temp_turn_table(5);
	// delay(1000);
	// temp_turn_table(3);
	
	// r.go_time(1000,127);
	// r.turn_angle(180);

	// r.follow_line_straight(100000, true);
	// demo_follow_line();

	// a.move_arm(1);
	// delay(2000);
	// a.move_arm(0);


	demo_turntable();
	std::cout << "Finish Demo" << std::endl;
	return 0;
}

void start_to_pallette()
{
    int path[4] = {2, 2, 1, 2};
    r.take_path(path, 4);
    r.turn_to_line(0, false,true);
    r.follow_line_straight(60, false);
    r.go_time(150, 255, false);
}

void demo_turntable()
{	
    t.turn_to_nest_pid (2);
    std::cout << "At nest 2" << std::endl;
    //delay(5000);
    t.turn_to_nest_pid (4);
    //delay(5000);

    std::cout << "At nest 6" << std::endl;
    t.turn_to_nest_pid (6);
    // delay(5000);

    t.turn_to_nest_pid (9);
    //delay(5000);

    t.turn_to_nest_pid (7);
    // delay(5000);

    t.turn_to_nest_pid (1);
    //delay(5000);

    t.turn_to_nest_pid (3);
    //delay(5000);

    t.turn_to_nest_pid (8);
}

void demo_follow_line()
{
    int path[9] = {2,2,2,2,0,0,2,0,2};
    r.take_path(path,9);
    r.go_time(61, 127, true);
    r.turn_angle(260);
    r.go_time(50, 255, true);
}

void temp_turn_table(int nests)
{
    stopwatch sw;
    sw.start();
    int ROT_CAL = 590;
    rlink.command(MOTOR_4_GO, 64);
    while(sw.read() < ROT_CAL * nests);
    rlink.command(MOTOR_4_GO, 0);
}

void push_egg(bool position)
{
    unsigned char current_reading = rlink.request(READ_PORT_1);
    if(position)
    {
	rlink.command(WRITE_PORT_1, current_reading | 0b00000010);
    }
    else
    {
	rlink.command(WRITE_PORT_1, current_reading & (~0b00000010));
    }

    current_reading = rlink.request(READ_PORT_1);
}
/** \mainpage Passover Elf Software Documentation
 * 
 * \section sectionid Overall Function
 * It was decided that the best approach to structuring the code is via a finite state machine like structure. 
 * This is preferred to a simple flowchart style because it allows for the existence of error states,
 * e.g. entering a new state where the AGV can recover after getting lost. Instead of drawing the entire state machine, 
 * the primary states and their descriptions are shown in the following table. The reason it is not drawn is because 
 * there can be many connections between states and failure states due to all the different ways things 
 * can fail, so a diagram does not really help understand the structure. The main states identified and their description are:
 *    - Determine required egg composition
 *        -# Read the IR beacon 
 *        -# Determine egg composition
 *        -# Determine delivery point
 *    - Navigate to palette 
 *        -# Follows line from start or current location to the location of egg turntable. 
 *        -# Align the AGV.
 *    - Obtaining Eggs
 *        -# Eggs will be removed from the palette one by one , a
 *        -# Analyse egg to determine the type
 *        -# Rotate turntable to grab the correct nest or recycling nest.
 *        -# Place egg in the correct nest or one of the recycling nests.
 *    - Refill Eggs
 *        -# It is expected to require around 30-40 eggs total (the robot can hold all at once)
 *        -# Move the robot away giving time for palette to be refilled
 *        -# Return to obtain eggs 
 *        -# Repeat until all eggs needed are acquired
 *    - Navigate to drop-off
 *        -# AGV drives to the appropriate delivery point
 *        -# Aligns to deliver nests
 *    - Drop off nests
 *        -# Spins the inbuilt turntable to the correct nest 
 *        -# Pushes it into delivery or recycling (procedure is the same, it just pushes a different nest).
 *        -# Go to and align with recycling zone, and then drop off nests again just like with the delivery zone.
 *    - Navigate to start
 *        -# Once the robot is done it returns to start point for the extra points. This is the end state.
 *    - Lost location recovery
 *        -# When AGV does not know its current location it enters this state
 *        -# Locate a junction somewhere on the field
 *        -# Follow it based on a guess of where it is
 *        -# Refining that guess by timing duration between junctions.
 *    - Sensor failure
 *        -# When AGV notices a sensor is not responding in a logical way
 *        -# This state (or potentially set of states, one per sensor type) determines how to handle it
 *        -# For example if the egg colour sensor fails then it will adjust the sorting function to just fill every nest with 4 arbitrary eggs and the rest in recycling since that is the best it can do.
 *    - Any Other Failures
 *        -# During testing its almost certain other potential failures will be discovered or thought of
 *        -# All of those can get their own state to resolve it
 * \subsection
 * The overall function of the AVG can be summarized as : 
 *    - Start
 *    - Register beacon code and determine the requirements
 *    - Collect eggs from C2
 *    - Retreat, wait to refill, collect again until requirements satisfied
 *    - Go to specified delivery
 *    - Leave required nests
 *    - Go to recycling
 *    - Leave recycling nests
 *    - If didn't deliver enough eggs, collect eggs again
 *    - If requirements satisfied and still have time, collect and recycle more eggs
 *    - If running out of time, go to Start
 *    - If raises error or is in unknown state, recover
 * \section wow Interface to other subsystems
 * \subsection hi Pin allocation on chips
 *    - Port​​ ​​0
 *        - Bit 0 - left​​ ​​IR​​ ​​line​​ ​​sensor
 *        - Bit 1 - centre​​​​ ​​IR​​ ​​line​​ ​​sensor
 *        - Bit 2 - right​ ​​IR​​ ​​line​​ ​​sensor
 *        - Bit 3 - turntable rotation ​​IR​​ ​​sensor
 *        - Bit 4 - Egg colour detector
 *        - Bit 5 - Egg size detector
 *        - Bit 6 - Switch for egg size and colour detection LEDs
 *        - Bit 7 - IR transceiver
 *    - Port​​ ​​1
 *        - Bit 0 - indicator​ LED 1 (Small Blue Egg)
 *        - Bit 1 - indicator​ LED 2 (Small Yellow Egg)
 *        - Bit 2 - indicator​ LED 3 (Big Yellow Egg)
 *        - Bit 3 - indicator​ LED 4 (Small Pink Egg)
 *    - Motor
 *        - Motor 1 - left wheel
 *        - Motor 2 - right wheel
 *        - Motor 3 - turntable servomotor
 * \subsection moretest Interaction between software and hardware
 *    - IR line following Sensors
 *        - Gives binary values (0 - black, 1 - white)
 *    - Distance Sensor
 *        - Gives distance away from obstacle
 *    - 2 Large Motors
 *        - Code controls speed, ramp time
 *        - Determine speed and direction of robot
 *    - 1 Small Motor
 *        - Code controls the rotation of the turntable
 *    - LEDs
 *        - Indicate the egg type that was just sorted
 *        - Indicate the contents of what's carried
 *    - IR transceiver
 *        - Determine the mixture of eggs needed to deliver
 * \section test Subsystem​​ ​​Operations​​ for ​​Basic​​ ​​Functionality​​ ​​Demonstration
 *     1. Line ​Follower
 *        -# Robot goes from S to S with the following path:
 *        -# Forward S -> J5
 *        -# Turn Left at J5
 *        -# Forward J5 -> J6 
 *        -# Turn Left at J6
 *        -# Forward J6 -> J1
 *        -# Turn Left at J1
 *        -# Forward J1 -> S 
 *        -# Realign at S
 *     2. Egg Placement
 *        -# Rotate turntable to a given nest number
 *        -# Drop egg into appropriate nest, and repeat
 *     3. Delivery Alignment and Delivery Drop
 *        -# Align the chassis at the delivery pointer
 *        -# Rotate the turntable so that the nest to be delivered is in correct position
 *        -# Action the actuator to push the nest into delivery zone
 */
