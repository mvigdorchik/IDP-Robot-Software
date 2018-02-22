#ifndef LINE_H
#define LINE_H
#include <iostream>
#include "point.h"

using namespace std;

class line
{
public:
    //Default constructor
    line();
    line(point P1, point P2);
    //Accessors
    point get_start();
    point get_end();
    int get_length();

private:
    point p1, p2;
    int l;
};

#endif // LINE_H 
