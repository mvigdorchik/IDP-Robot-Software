#include "line.h"
#include "point.h"
#include "table.h"
#include <iostream>

using namespace std;

//Default constructor
table::table()
{
	junctions["J1"] = point(300, 793);
	junctions["J2"] = point(1200, 793);
	junctions["J3"] = point(1200, 280);
	junctions["J4"] = point(1810, 280);
	junctions["J5"] = point(2100, 280);
	junctions["J6"] = point(2100, 793);
	junctions["D1"] = point(1790, 2350);
	junctions["D2"] = point(1200, 2350);
	junctions["D3"] = point(300, 2350);
	junctions["Start"] = point(300, 280);
	junctions["JD1"] = point(1790, 0);
	junctions["JD2"] = point(1200, 0);
	junctions["JD3"] = point(300, 0);
	junctions["JC2"] = point(1733, 1543);
	junctions["H1"] = point(300, 0);
	junctions["H2"] = point(300, 0);
	junctions["H3"] = point(300, 0);
	junctions["H4"] = point(300, 0);
	junctions["H5"] = point(300, 0); //TODO Verify measurements for points where coordinates are 0
}




