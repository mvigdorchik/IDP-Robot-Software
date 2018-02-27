#ifndef ARM_H
#define ARM_H

#include <iostream>
#include <bitset>
#include <robot_instr.h>
#include <robot_delay.h>
#include <robot_link.h>
#include <cmath>
#include <vector>
#include <stopwatch.h>

extern robot_link rlink;

/**
 * Controls everything to do with the sweeping arm of the AGV. This is generally a simple
 * class as the arm only really moves up and down.
 */
class arm
{
public:
    /**
     * Default constructor, sets the starting arm position.
     */
    arm();
    
    /**
     * Moves the arm up or down. This also corresponds to opening and closing the hatch on the egg
     * sensing bucket. This function will do nothing if it is already in the correct position.

     * @param position True to move the arm up, false to move it down.
     */
    void move_arm(bool position);

    /**
     * Returns the position of the arm, either up or down

     * @return True if the arm is up, false otherwise.
     */ 
    bool get_currrent_position();

private:
    bool current_position;
};

#endif /* ARM_H */
