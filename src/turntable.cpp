#include "turntable.h"
#include "pid.h"

//Default constructor
turntable::turntable():current_nest(0)
{
}

int turntable::read_pot()
{
    int result = rlink.request(ADC0);  //Reads measurement from ADC0
    result = (result - POT_START_OFFSET);
    result = result > POT_MAX_VALUE ? POT_MAX_VALUE : result;
    result = result < -POT_START_OFFSET ? -POT_START_OFFSET : result;

    return result;
}


void turntable::turn_to_nest_pid(int nest)
{
    int target;
    switch (nest) {
        case 0:
	    target = 0;
	    break;
	case 1:
	    target = 29;
	    break;
	case 2:
	    target = 59;
	    break;
	case 3:
	    target = 86;
	    break;
	case 4:
	    target = 114;
	    break;
	case 5:
	    target = 140;
	    break;
	case 6:
	    target = 173;
	    break;
	case 7:
	    target = 205;
	    break;
	case 8:
	    target = 238;
	    break;
	default:
	    target = 0;
    }
    std::cout << "Target is :" << target << std::endl;
    turn_angle_pid(target);
}

void turntable::turn_to_push_pid(int nest)
{
    int target;
    switch (nest) {
        case 0 :
	    target = 65;
	    break;
	case 1 :
	    target = 93;
	    break;
	case 2 :
	    target = 122;
	    break;
	case 3 :
	    target = 150;
	    break;
	case 4 :
	    target = 180;
	    break;
	case 5 :
	    target = 214;
	    break;
	case 6 :
	    target = 246;
	    break;
	case 7 :
	    target = 7;
	    break;
	case 8 :
	    target = 35;
	    break;
	default :
	    target = 65;
    }
    turn_angle_pid(target);
    std::cout << "target is:" << target << std::endl;
}

//TODO !!!!
void turntable::turn_angle_pid(int target)
{
    double min = -127.0;
    double max = 127.0;
    PID pid = PID(TURNTABLE_dt, max, min, TURNTABLE_Kp, TURNTABLE_Kd, TURNTABLE_Ki);
    int val = this->read_pot();
    double inc = POT_MAX_VALUE;
    
    int cycles_in_tol = 0;
    int MAX_TIME = 4500;
    stopwatch timeout;
    // if(DEBUG) std::cout << "Target" << target << std::endl;
    timeout.start();
    // while (std::abs((float)(val - target)) > TURNTABLE_tol || std::abs((float)inc) > 10*TURNTABLE_tol) {
    while (cycles_in_tol < 15 && timeout.read() < MAX_TIME) {
	// std::cout << cycles_in_tol << std::endl;
	// if(DEBUG) std::cout << "val is " << val << std::endl;
	// if(DEBUG) std::cout << "target is " << target << std::endl;
	inc = pid.calculate(target, val);
	// if(DEBUG) std::cout << "inc is " << inc << std::endl;
	val = this->read_pot();
	if (inc>0)
	    this->turn(true, (int) (inc));
	else
	    this->turn(false, (int) (-inc));
	if(std::abs((float)(val - target)) < TURNTABLE_tol || std::abs((float)inc) < 7*TURNTABLE_tol)
	    cycles_in_tol++;
	else
	    cycles_in_tol = 0;
    }
    this->turn(true, 0);
}

void turntable::turn(bool clockwise, int speed)
{
    unsigned char adjusted_speed = speed > 127 ? 127 : speed; //Sets a limit to speed of turn
    adjusted_speed = clockwise ? adjusted_speed : adjusted_speed + 128;
    rlink.command(MOTOR_4_GO, adjusted_speed);
}

unsigned char turntable::read_sensor()
{
    unsigned char result = 0;
    unsigned char current_state = rlink.request(READ_PORT_0);
    current_state |= 0b00101111; //Ensures that the output pins don't change their state
    rlink.command(WRITE_PORT_0, current_state);
    result = rlink.request(READ_PORT_0);
    result &= 0b0001000; //Ignore any other sensor on that bus. 
    result >>= 3;
    if (BLACK_HI)
	return !result;
    else return result;
}

void turntable::initial_align()
{
    stopwatch sw;
    int lines = 0;
    int interval = 0;
    this->turn(true, 127);
    sw.start();
    while (lines < 3)
    {
	if (this->read_sensor()) // When it meets a new line
	{
	    if (!lines || (sw.read() > DOUBLE_LINE_TIMEOUT_CALIBRATION)) // It's the first line in the series, so make lines 1
	    {
		lines = 1;
	    }
	    else if (sw.read() > MIN_TIME_DETECTION_CALIBRATION) // It wasn't the first line and it was close to previous one, so increment lines
	    {
		++lines;
	    }
	    sw.start(); // Restart stopwatch
	}
    }
    this->turn(false, TURNTABLE_SLOW_SPEED);
    bool current_state = this->read_sensor();
	
    if (current_state == 0) // If passed the calibration line
    {
	while (!this->read_sensor()){} // Wait to go back to the calibration line 
    }
    sw.start();
    while (!this->read_sensor() || sw.read() < (MIN_TIME_DETECTION_CALIBRATION * 128 / TURNTABLE_SLOW_SPEED)) {} // Wait until it gets to the central line
    delay(MIN_TIME_DETECTION_BIG/2 * 128 / TURNTABLE_SLOW_SPEED - 10); // Wait until it gets to centre
    this->turn(true, 0); // Stop turntable motor
    sw.stop();
    this->current_nest = 1;
}

void turntable::turn_to_nest(int next_nest)
{
    int diff = (next_nest - this->current_nest) < 0 ? (next_nest - this->current_nest + TOTAL_NUMBER_NESTS) : (next_nest - this->current_nest);
    bool direction = diff < (TOTAL_NUMBER_NESTS/2 + 1) ? true : false;
    int intervals = diff > (TOTAL_NUMBER_NESTS / 2) ? TOTAL_NUMBER_NESTS - diff : diff;
    int degrees = 360 / TOTAL_NUMBER_NESTS * intervals;
    stopwatch sw;
    int lines_passed = 0;

    // turn_angle_time(direction, degrees);   // too rudimentary and imprecise

    this->turn(direction, 127);
    sw.start();
    while (lines_passed < intervals)
    {
	if (sw.read() > DOUBLE_LINE_TIMEOUT_BIG && !this->read_sensor()) // It passed a line without reading it
	{
	    ++lines_passed;
	    if(DEBUG) std::cout << "I MISSED THE " << lines_passed << "th LINE" << std::endl;
	    sw.start(); // Restart stopwatch
	}
	else if (sw.read() > MIN_TIME_DETECTION_BIG && this->read_sensor()) // It meets a new line
	{		
	    ++lines_passed;
	    if(DEBUG) std::cout << "I FOUND THE " << lines_passed << "th LINE" << std::endl;
	    sw.start(); // Restart stopwatch
	}
    }
    bool current_state = this->read_sensor();

    if (current_state) // It didn't passed entirely the line
    {
	this->turn(direction, TURNTABLE_SLOW_SPEED);
	while (this->read_sensor()) {} // Wait to go back to the calibration line 
    }
    this->turn(!direction, TURNTABLE_SLOW_SPEED); // Starts reversing
    while (!this->read_sensor()) {} // Wait until it sees again the line
    delay(MIN_TIME_DETECTION_BIG / 2 * 128 / TURNTABLE_SLOW_SPEED - 10); // Wait until it gets to centre of line
    this->turn(true, 0); // Stop turntable motor
    sw.stop();
    this->current_nest = next_nest;
}


void turntable::turn_to_nest_thin(int next_nest)
{
    int diff = (next_nest - this->current_nest) < 0 ? (next_nest - this->current_nest + TOTAL_NUMBER_NESTS) : (next_nest - this->current_nest);
    bool direction = diff < (TOTAL_NUMBER_NESTS / 2 + 1) ? true : false;
    int intervals = diff >(TOTAL_NUMBER_NESTS / 2) ? TOTAL_NUMBER_NESTS - diff : diff;
    int degrees = 360 / TOTAL_NUMBER_NESTS * intervals;
    stopwatch sw;
    int lines_passed = 0;

    // turn_angle_time(direction, degrees);   // too rudimentary and imprecise

    this->turn(direction, 127);
    sw.start();
    while (lines_passed < intervals * 3 - 1)
    {
	if (sw.read() > DOUBLE_LINE_TIMEOUT_SMALL && !this->read_sensor()) // It passed a line without reading it
	{
	    ++lines_passed;
	    if(DEBUG) std::cout << "I MISSED THE " << lines_passed << "th LINE" << std::endl;
	    sw.start(); // Restart stopwatch
	}
	else if (sw.read() > MIN_TIME_DETECTION_SMALL && this->read_sensor()) // It meets a new line
	{
	    ++lines_passed;
	    if(DEBUG) std::cout << "I FOUND THE " << lines_passed << "th LINE" << std::endl;
	    sw.start(); // Restart stopwatch
	}
    }
    this->turn(direction, TURNTABLE_FIRST_DECREASED_SPEED); // First stage of decreasing speed
    while (this->read_sensor() && sw.read()<MIN_TIME_DETECTION_SMALL * 128 / TURNTABLE_FIRST_DECREASED_SPEED) {} // Wait it's still on line or it passed without seeing it
    sw.start(); // Restart stopwatch
    this->turn(direction, TURNTABLE_SECOND_DECREASED_SPEED);
    while (this->read_sensor() && sw.read()<MIN_TIME_DETECTION_SMALL * 128 / TURNTABLE_SECOND_DECREASED_SPEED) {} // Wait it's still not on a line or it passed without seeing it
    this->turn(direction, 0); // Stops turntable
    bool current_state = this->read_sensor(); //Checks if it's on target line

    if (!current_state) // If it's not on the target line
    {
	this->turn(!direction, TURNTABLE_SLOW_SPEED/2);
	while (!this->read_sensor()) {} // Wait until it sees again the line
	delay(MIN_TIME_DETECTION_BIG * 128 / TURNTABLE_SLOW_SPEED - 10); // Wait until it gets to centre of line
	this->turn(true, 0); // Stop turntable motor
    }
    sw.stop();
    current_nest = next_nest;
}

void turntable::turn_angle_time(bool clockwise, int degrees)
{
    stopwatch sw;
    sw.start();
    this->turn(clockwise, 127);
    while (sw.read() < ((degrees - TURNTABLE_INERTIA_CALIBRATION + 0) / TURNTABLE_ROTATION_SPEED)) {}
    this->turn(clockwise, 0);
    sw.stop();
}

void turntable::calibrate_egg_sensor()
{
    no_egg_reading = rlink.request(ADC1);
}

Egg turntable::measure_egg_type()
{
    bool is_big;
    unsigned char current_values = rlink.request(READ_PORT_0);
    rlink.command(WRITE_PORT_0, current_values | 0b01100000);
    is_big = rlink.request(READ_PORT_0) & 0b00100000;
    is_big = !is_big; //The sensor returns 1 when it is small so fix that
    rlink.command(WRITE_PORT_0, current_values & 0b10111111); //Turn off the LED to save current
    std::cout << is_big << std::endl;
    bool is_yellow;
    if(is_big)
    {
	is_yellow = rlink.request(ADC1) < 1.17*no_egg_reading;
    }
    else
    {
	is_yellow = rlink.request(ADC1) < 1.17*no_egg_reading; //TODO change the calibration of this
    }
    
    Egg result;
    if(is_big)
	result = is_yellow ? BIG_YELLOW : BIG_PINK;
    else
	result = is_yellow ? SMALL_YELLOW : SMALL_BLUE;
	
    return result;
}

int turntable::determine_nest(Egg egg_type)
{
    if(egg_type == big_egg || egg_type == small_egg)
    {
	//Loop through every non recycling nest to check for an open spot
	//Note that even nests 0,2,4,6 are the real nests. This makes everything go generally faster
	for(int i = 0; i < 7; i++)
	{
	    if(i % 2 == 1) //only want even nests
		continue;
	    if(nests[i].size() < 4) //4 eggs is a full nest
	    {
		int egg_count = 0;
		//Count the eggs of egg_type to ensure that there is only 0 or 1. 
		for(std::vector<Egg>::iterator it = nests[i].begin(); it != nests[i].end(); ++it)
		{
		    if(*it == egg_type)
			egg_count++;
		}
		if(egg_count < 2) //Hey we found the right nest!
		{
		    nests[i].push_back(egg_type);
		    return i;
		}
	    }
	}
    }
    //If egg_type made it this far then it must be up for recycling
    for(int i = 1; i < TOTAL_NUMBER_NESTS; i++)
    {
	//Skip even nests, but allow nest 8 since thats also recycling
	if(i % 2 == 0 && i != 8)
	    continue;
	if(nests[i].size() < 4)
	{
	    int size_egg_count = 0;
	    //Count the eggs of egg_types size to ensure that there is only 0 or 1 of that size. 
	    for(std::vector<Egg>::iterator it = nests[i].begin(); it != nests[i].end(); ++it)
	    {
		//Add an egg to count if and only if the size of the egg being tested and that egg is the same
		if((*it >= 2 && egg_type >=2) || (*it <= 1 && egg_type <=1))
		    size_egg_count++;
	    }
	    if(size_egg_count < 2) 
	    {
		nests[i].push_back(egg_type);
		return i;
	    }
	}
    }
    return -1; // No space available means that every nest is full.
}

int turntable::place_egg()
{
    //TODO: REMEMBER TO USE THE LIGHTS`
    Egg type = this->measure_egg_type();
    if(DEBUG) std::cout << "Egg type is:" << type << std::endl;
    int nest_num = this->determine_nest(type);
    if(DEBUG) std::cout << "Nest number is: " << nest_num << std::endl;

    //Code for indicator lights
    int current_state = rlink.request(READ_PORT_1) & 0b00000111;
    int output = 0;
    switch (type) {
    case BIG_YELLOW: {
	output = 1 << 3;
	break;
    }
    case BIG_PINK: {
	output = 1 << 4;
	break;
    }
    case SMALL_YELLOW: {
	output = 1 << 5;
	break;
    }
    case SMALL_BLUE: {
	output = 1 << 6;
	break;
    }
default:
	break;
    }
    output |= 0b1000000;
    rlink.command(WRITE_PORT_1, output | current_state);
    
    if(nest_num == -1)
	return -1; //Time to leave with full nests

    this->turn_to_nest_pid(nest_num);
    a.move_arm(0); //Let the egg fall through by moving arm up
    delay(1000);
    
    if(type >= 2) //This means that the egg is big
	this->jiggle_table(); //Jiggles the turntable to ensure big egg falls in correctly
    a.move_arm(1); //Put the arm back down to knock further eggs

    return 0;
}

void turntable::jiggle_table()
{
    // for(int i = 0; i < 5; i++)
    // {
    // 	rlink.command(MOTOR_4_GO, 127); 
    // 	delay(100);
    // 	rlink.command(MOTOR_4_GO, 255);
    // 	delay(100);
    // }
    
    stopwatch sw;
    for(int i = 0; i < 1; i++)
    {
	sw.start();
	int ROT_CAL = 400;
	rlink.command(MOTOR_4_GO, 127);
	while(sw.read() < ROT_CAL);
	rlink.command(MOTOR_4_GO, 0);
	rlink.command(MOTOR_4_GO, 255);
	sw.start();
	while(sw.read() < ROT_CAL);
	rlink.command(MOTOR_4_GO,0);
    }
}

void turntable::push_nest()
{
    //Retracts the arm (should be unneeded) then pushes the nest and retracts the arm after
    unsigned char current_reading = rlink.request(READ_PORT_1);
    rlink.command(WRITE_PORT_1, current_reading & (~0b00000010));
    delay(200);
    rlink.command(WRITE_PORT_1, current_reading | 0b00000010);
    delay(800);
    rlink.command(WRITE_PORT_1, current_reading & (~0b00000010));
}

void turntable::move_ejector(bool position)
{
    unsigned char current_reading = rlink.request(READ_PORT_1);
    if(position)
    {
	rlink.command(WRITE_PORT_1, current_reading | 0b00000010);
    }
    else
    {
	rlink.command(WRITE_PORT_1, current_reading & (~0b00000010));
    }
}
