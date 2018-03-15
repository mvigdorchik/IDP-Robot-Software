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


/**
 * This class is used to control the turntable (collection point C2)
 * and it includes functions to read and write requierd ports on baloon board
 */
class palette
{
public:
    /**
    * Default constructor, initializing the state of the palette
    */
    palette();

    /**
    * Rotates the pallette one eggs worth in order
    * to collect them

    */
    void rotate(int number);

    /**
    * Gives a number of short pulses to the required pin in order to be
    * registered by the PIC to give the required length of pulses to
    * rotate the turntable

    * @param short_pulses_no number of short pulses to send to the PIC
    */
    void increment(int short_pulses_no);

    /**
    * Resets the turntable to default position by giving a long 1 s pulse to PIC
    * so that
    */
    void reset();
};



#endif /* PALETTE_H */
