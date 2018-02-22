#ifndef EGG_H
#define EGG_H
#include <iostream>

using namespace std;

class egg
{
public:
    //Default constructor
    egg();
    egg(int order);
	
    //Accessors
    int getX();
    int getY();
    int get_nest();
    int get_type();
    
    unsigned char get_colour();
	
    //Mutators
    void set_type(int t);
    void set_colour(unsigned char c);
    void set_nest(int nest_number);
    void set_nest(int type, unsigned char colour, int beacon_code, int recycle_basket);
	
private:
    int xCOORD, yCOORD, type, nest_number;
    unsigned char colour;
};

#endif // EGG_H
