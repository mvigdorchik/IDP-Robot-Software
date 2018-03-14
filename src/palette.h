#ifndef PALETTE_H
#define PALETTE_H


#include <iostream>
#include <bitset>
#include <robot_instr.h>
#include <robot_link.h>
#include <robot_delay.h>
#include <stopwatch.h>

#define DEBUG 1 //If defined all of the print code will run, otherwise it won't
extern robot_link rlink;

class palette
{
public:
    /**
    * Default constructor, initializing the state of the palette
    */
    palette();

    void rotate(int egg_number);

    void increment(int short_pulses_no);

    void reset();
};



#endif /* PALETTE_H */
