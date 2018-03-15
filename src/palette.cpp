#include "palette.h"

//Default constructor
palette::palette()
{
    unsigned char current_state = rlink.request(READ_PORT_0);
    current_state &= 0b01101111; //Default state is low
    rlink.command(WRITE_PORT_0, current_state);
}

void palette::increment(int short_pulses_no)
{
    unsigned char current_state = rlink.request(READ_PORT_0);
    current_state &= 0b10011111; //Default state is low

    for (int i=0; i< short_pulses_no; i++)
    {
	current_state |= 0b10000000; // Make pin high
	rlink.command(WRITE_PORT_0, current_state);
	current_state &= 0b01111111; // Make pin low
	rlink.command(WRITE_PORT_0, current_state);
    }
}

void palette::reset()
{
    unsigned char current_state = rlink.request(READ_PORT_0);
    current_state |= 0b00010000; // Make pin high
    rlink.command(WRITE_PORT_0, current_state);
    delay(100);
    current_state &= 0b11101111; // Make pin low
    rlink.command(WRITE_PORT_0, current_state);
    delay(100);
    this->rotate(2);
}

void palette::rotate(int number)
{
    unsigned char current_state = rlink.request(READ_PORT_0);

    for(int i = 0; i < number; i++)
    {
	current_state |= 0b10000000; // Make pin high
	rlink.command(WRITE_PORT_0, current_state);
	delay(50);
	current_state &= 0b01111111; // Make pin low
	rlink.command(WRITE_PORT_0, current_state);
	delay(50);
    }
}
