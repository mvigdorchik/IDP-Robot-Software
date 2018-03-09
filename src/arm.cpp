#include "arm.h"

arm::arm()
{
    current_position = 0;
}

void arm::move_arm(bool position)
{
    unsigned char current_reading = rlink.request(READ_PORT_1);
    if(position)
    {
	rlink.command(WRITE_PORT_1, current_reading | 0b00000001);
    }
    else
    {
	rlink.command(WRITE_PORT_1, current_reading & (~0b00000001));
    }
}
