#include <iostream>
#include <robot_instr.h>
#include <robot_link.h>
#include <stopwatch.h>

#define ROBOT_NUM 2
#define DEBUG //If defined all of the print code will run, otherwise it won't

robot_link rlink;
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
	    //Pray to your local diety
	    
	    return -1;
	}
#ifdef DEBUG    
	std::cout << "Successful Connection" << std::endl;
#endif
#endif

	int val;
	stopwatch sw;
	sw.start();
	for(int i = 0; i < 500; i++)
	{
	    val = rlink.request(TEST_INSTRUCTION);
	}
	float time_taken = sw.read();
	sw.stop();
	std::cout << "Time taken was:" << time_taken << std::endl;

	/*if(val == TEST_INSTRUCTION_RESULT)
	{
	    std::cout << "test passed" << std::endl;
	    return 0;
	}
	else if(val == REQUEST_ERROR)
	{
	    std::cout << "Fatal errors on link:" << std::endl;
	    rlink.print_errs();
	}
	else
	    std::cout << "test failed some weird way" << std::endl;
	    return -1; */
 
}
