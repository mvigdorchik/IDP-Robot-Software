#include "egg.h"
#include <iostream>

using namespace std;
egg::egg():xCOORD(0), yCOORD(0)
{
   //default constructor
   //does nothing
}

egg::egg(int order)
{
    xCOORD = 1125 + order*50;
    yCOORD = 75;
}

//MUTATOR FUNCTIONS
void egg::set_colour(unsigned char c)
{
   colour = c;
}
void egg::set_type(int t)
{
    type = t;
}
void egg::set_nest(int n)
{
	nest_number = n;
}
void egg::set_nest(int type, unsigned char colour, int beacon_code, int recycle_basket)
{
	if (beacon_code == 1 && (colour == 'B' || (colour == 'Y' && type == 2)))
		nest_number = 1;
	else if (beacon_code == 3 && (colour == 'B' || (colour == 'Y' && type == 2)))
		nest_number = 3;
	else if (beacon_code == 4 && (colour == 'P' || (colour == 'Y' && type == 1)))
		nest_number = 4;
	else if (beacon_code == 6 && (colour == 'P' || (colour == 'Y' && type == 1)))
		nest_number = 6;
	else 
		nest_number = recycle_basket;
}

//ACCESSOR FUNCTIONS
int egg::getX()
{
    return xCOORD;
}
int egg::getY()
{
    return yCOORD;
}
int egg::get_order()
{
	return order;
}
int egg::get_nest()
{
	return nest_number;
}
int egg::get_type()
{
	return type;
}
unsigned char egg::get_colour()
{
	return colour;
}