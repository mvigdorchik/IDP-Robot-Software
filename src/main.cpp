#include <iostream>
#include <bitset>
#include <robot_instr.h>
#include <robot_link.h>
#include <robot_delay.h>
#include <stopwatch.h>
#include "robot.h"
#include "turntable.h"

#define ROBOT_NUM 2
#define DEBUG 1 //If defined all of the print code will run, otherwise it won't

robot_link rlink;
robot r;

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

	rlink.command(RAMP_TIME, ROBOT_RAMP_TIME);
	
	//r.go_time(30000,127);
	//r.turn(180);
	//int path[5] = {2,0,3,1,3};
	//r.take_path(path, 5);
	//r.follow_line_straight(1);
	turntable t;
	for (int i = 1; i <= 8; ++i)
	{
	    std::cout << "Entered test loop" << std::endl;
	    t.turn_to_nest(1, i);
	    delay(1000);
	}

	return 0;
}

/** \mainpage Passover Elf Software Documentation
 * 
 * \section Overall Function
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
 * \section Interface to other subsystems
 * \subsection Pin allocation on chips
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
 * \subsection Interaction between software and hardware
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
 * \section Subsystem​​ ​​Operations​​ for ​​Basic​​ ​​Functionality​​ ​​Demonstration
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
 *     2. Egg ​Discriminator
 *        -# Test all 4 types of eggs and verify the correct indicator​ LEDs light up:
 *        -# Led 1 : Small Blue 
 *        -# Led 2 : Small Yellow
 *        -# Led 3 : Big Yellow
 *        -# Led 4 : Big Pink
 *     3. Delivery Alignment and Delivery Drop
 *        -# Align the chassis at the delivery pointer
 *        -# Rotate the turntable so that the nest to be delivered is in correct position
 *        -# Action the actuator to push the nest into delivery zone
 */