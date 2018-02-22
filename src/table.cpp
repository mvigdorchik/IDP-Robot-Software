#include "line.h"
#include "point.h"
#include "table.h"
#include <iostream>

using namespace std;

//Default constructor
line::line():l(0)
{
	point J1, J2, J3, J4, J5, J6, D1, D2, D3, Start, JD1, JD2, JD3, JC2, H1, H2, H3, H4, H5;
	J1 = point(300, 793);
	J2 = point(1200, 793);
	J3 = point(1200, 280);
	J4 = point(1810, 280);
	J5 = point(2100, 280);
	J6 = point(2100, 793);
	D1 = point(1790, 2350);
	D2 = point(1200, 2350);
	D3 = point(300, 2350);
	Start = point(300, 280);
	JD1 = point(1790, 0);
	JD2 = point(1200, 0);
	JD3 = point(300, 0);
	JC2 = point(1733, 1543);
	H1 = point(300, 0);
	H2 = point(300, 0);
	H3 = point(300, 0);
	H4 = point(300, 0);
	H5 = point(300, 0); //TODO Verify measurements for points where coordinates are 0
	
}



//ACCESSORS
point table::get_start()
{
	return Start;
}
//	point table::get_start_east(); Do we need these?
//	point table::get_start_west();
//	point table::get_start_south();
//	point table::get_start_north();
point table::get_J1()
{
	return J1;
}
point table::get_J2()
{
	return J2;
}
point table::get_J3()
{
	return J3;
}
point table::get_J4()
{
	return J4;
}
point table::get_J5()
{
	return J5;
}
point table::get_J6()
{
	return J6;
}
point table::get_D1()
{
	return D1;
}
point table::get_D2()
{
	return D2;
}
point table::get_D3()
{
	return D3;
}
point table::get_JD1()
{
	return JD1;
}
point table::get_JD2()
{
	return JD2;
}
point table::get_JD3()
{
	return JD3;
}
point table::get_JC2()
{
	return JC2;
}
point table::get_H1()
{
	return H1;
}
point table::get_H2()
{
	return H2;
}
point table::get_H3()
{
	return H3;
}
point table::get_H4()
{
	return H4;
}
point table::get_H5()
{
	return H5;
}