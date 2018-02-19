#include <iostream>
#include <robot_instr.h>
#include <robot_link.h>

robot_link rlink;
int main()
{
	std::cout << "test" << std::endl;

#ifndef ARM
	if(!rlink.initialise(1))
	{
	    std::cout << "Can't find robit" << std::endl;
	}
#endif
}
