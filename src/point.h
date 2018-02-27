#ifndef POINT_H
#define POINT_H
#include <iostream>
#include <vector>

/**
 * Data structure to hold a junction on the board. It keeps track of the coordinates of that junction,
 * and will keep track of the connections to other junctions.
 */
class point
{
public:
    /**
     * Default constructor that should not be used in general case. It will simply create a point 
     * at X,Y = 0.
     */
    point();

    /**
     * Constructs a point with given coordinates. This is the standard constructor that should be used most often.

     * @param new_x X-coordinate as an integer in mm.
     * @param new_y Y-coordinate as an integer in mm.
     */
    point(int new_x, int new_y);

    /**
     * @return X-coordinate in mm.
     */
    int getX();

    /**
     * @return Y-coordinate in mm.
     */
    int getY();

    /**
     * 4 lines represent the adjacent nodes, note that the lines can be empty.
     */
private:
    int xCOORD, yCOORD;
   };

#endif // POINT_H
