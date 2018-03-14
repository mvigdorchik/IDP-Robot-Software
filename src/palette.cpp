#include "palette.h"

//Default constructor
palette::palette()
{
    unsigned char current_state = rlink.request(READ_PORT_0);
    current_state &= 0b00111111; //Default state is low
    rlink.command(WRITE_PORT_0, current_state);
}

void palette::increment(int short_pulses_no)
{
    unsigned char current_state = rlink.request(READ_PORT_0);
    current_state &= 0b00111111; //Default state is low

    for (int i=0; i< short_pulses_no; i++)
    {
	current_state |= 0b01000000; // Make pin high
	rlink.command(WRITE_PORT_0, current_state);
	current_state &= 0b00111111; // Make pin low
	rlink.command(WRITE_PORT_0, current_state);
    }
}

void palette::reset()
{
    unsigned char current_state = rlink.request(READ_PORT_0);
    current_state |= 0b01000000; // Make pin high
    rlink.command(WRITE_PORT_0, current_state);
    delay(1000);
    current_state &= 0b00111111; // Make pin low
    rlink.command(WRITE_PORT_0, current_state);
}

void palette::rotate(int egg_number)
{
    this->increment(egg_number); //TODO change in case a short pulse doesn't change the position for one egg
}
