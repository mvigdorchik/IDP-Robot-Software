#ifndef TABLE_H
#define TABLE_H

#include <iostream>


class table
{
public:
    table();
    
    //Accessors
    point get_start();
//	point get_start_east(); Do we need these?
//	point get_start_west();
//	point get_start_south();
//	point get_start_north();
    point get_J1();
    point get_J2();
    point get_J3();
    point get_J4();
    point get_J5();
    point get_J6();
    point get_D1();
    point get_D2();
    point get_D3();
    point get_JD1();
    point get_JD2();
    point get_JD3();
    point get_JC2();
    point get_H1();
    point get_H2();
    point get_H3();
    point get_H4();
    point get_H5();

private:
    point J1,J2,J3,J4,J5,J6,D1,D2,D3,JD1,JD2,JD3,JC2,H1,H2,H3,H4,H5, Start;
	
};


#endif /* TABLE_H */
