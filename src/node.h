#ifndef NODE_H
#define NODE_H
#include <iostream>
#include "point.h"
#include "line.h"

extern std::map<std::string, point> junction_points;
class node
{
public:

    /**
     * Constructor to create a node with all of the adjacent points. This will then populate the 4 adjacent lines
     * and the main point. The reason this uses strings is to make initializing this much easier, by providing
     * a std::map to the points, which is defined in main.cpp. This class is the main unit by which an internal
     * map of the table will be generated, allowing easier navigation.
     
     * @param main The point this node represents.
     * @param north The point north of main
     * @param east The point east of main
     * @param south The point south of main
     * @param west The point west of main
     */
    node(std::string main, std::string north, std::string east, std::string south, std::string west);

    /**
     * Returns the distance from the given direction to the current node. 
     * An input of 0 is left, 1 is right, 2 is north, 3 is south.

     * @param direction The direction to retrieve distance.
     * @return The distance in mm to the junction in that direction.
     */
    int get_adj_distance(unsigned char direction);
	
    point main;
    line north,south,east,west;
}

#endif /* NODE_H */
