#ifndef LINE_H
#define LINE_H
#include <iostream>
#include "point.h"

/**
 * Datastructure to contain the actual lines between two points, including their length. 
 * Act as edges in the graph that is a map of the table, and is especially useful for recovering
 * robot position and determining if the robot is lost based on the expected time to traverse lines.
 */
class line
{
public:
    /**
     * Empty constructor to make an empty line. Generally should not be used
     */
    line();

    /**
     * Constructs a line between two points, setting its length and other properties.
     
     * @param P1 First junction making up the line.
     * @param P2 Second junction making up the lines
     * @see point
     */
    line(point P1, point P2);
    //Accessors
    
    /**
     * Returns the first point making up the line.
     
     * @return Point p1.
     */
    point get_start();

    /**
     * Returns the second point making up the line.
     
     * @return Point p2.
     */
    point get_end();

    /**
     * Returns the length of the line, done as Taxicab distance (|x1-x2| + |y1-y2|).
     * Taxicab distance is used because it simplifies calculation and all lines except the curved
     * one on the field are exactly vertical or horizontal. The curved line is treated seperately in software.
     
     * @return Length of the line in mm.
     */
    int get_length();

    /**
     * Returns true if the line actually has a start point and endpoint. Will only be true if the 
     * default constructor was used.

     * @return Whether or not the line was created with the default constructor
     */
    bool is_empty();

private:
    point  p1, p2;
    bool empty;
    int l;
};

#endif // LINE_H 
