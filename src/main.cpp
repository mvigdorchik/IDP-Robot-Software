#include <iostream>
#include <bitset>
#include <robot_instr.h>
#include <robot_link.h>
#include <robot_delay.h>
#include <stopwatch.h>
#include "robot.h"

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
#ifdef DEBUG    
	std::cout << "Successful Connection" << std::endl;
#endif
	
	//r.go_time(30000,127);
	//r.turn(180);
	stopwatch sw1;
	sw1.start();
	r.follow_line_straight(40000);
	return 0;
}

