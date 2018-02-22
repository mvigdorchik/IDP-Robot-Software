#include "point.h"
#include <iostream>

using namespace std;
point::point():xCOORD(0), yCOORD(0)
{
   //default constructor
   //does nothing
}

point::point(int new_x, int new_y)
{
    xCOORD = new_x;
    yCOORD = new_y;
}

//MUTATOR FUNCTIONS
void point::setX(int x)
{
    xCOORD = x;
}
void point::setY(int y)
{
    yCOORD = y;
}

//ACCESSOR FUNCTIONS
int point::getX()
{
    return xCOORD;
}
int point::getY()
{
    return yCOORD;
}