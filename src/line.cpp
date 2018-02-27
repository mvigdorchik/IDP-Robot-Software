#include "line.h"
#include "point.h"
#include <iostream>
#include <cmath>

using namespace std;

//Default constructor
line::line():l(0)
{
    //left blank intentionally
    empty = true;
}
//Point1_Object & Point2_Object were suppose to get
//Passed into here
line::line(point P1, point  P2)
{
    p1 = P1;
    p2 = P2;
    int x1,x2,y1,y2;
    x1 = P1.getX();
    x2 = P2.getX();
    y1 = P1.getY();
    y2 = P2.getY();
    l = std::abs((float)(x2-x1)) + std::abs((float)(y2-y1));
    empty = false;
}

//ACCESSORS
point  line::get_start()
{
    return p1;
}
point  line::get_end()
{
    return p2;
}
int line::get_length()
{
    return l;
}

bool line::is_empty()
{
    return empty;
}
