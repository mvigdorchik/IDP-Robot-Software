#ifndef POINT_H
#define POINT_H
#include <iostream>

using namespace std;

class point
{
public:
    //Default constructor
    point();
    point(int new_x, int new_y);
    //Accessors
    int getX();
    int getY();
    //Mutators
    void setX(int x);
    void setY(int Y);
private:
    int xCOORD, yCOORD;
};

#endif // POINT_H