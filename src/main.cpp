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

