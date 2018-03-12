#include "turntable.h"
#include "pid.h"

//Default constructor
turntable::turntable():current_nest(1)
{
  //	rlink.command(RAMP_TIME, TURNTABLE_RAMP_TIME);
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

int turntable::read_pot()
{
	int result = rlink.request(ADC0);  //Reads measurement from ADC0
	result = (result - POT_START_OFFSET)*360/250;  // Scale measurement to degrees
	result = result > 360 ? 360 : result;
	result = result < 1 ? 1 : result;

	return result;
}


void turntable::turn(bool clockwise, int speed)
{
	unsigned char adjusted_speed = speed > 127 ? 127 : speed; //Sets a limit to speed of turn
	adjusted_speed = clockwise ? adjusted_speed : adjusted_speed + 128;
	rlink.command(MOTOR_4_GO, adjusted_speed);
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
			std::cout << "I MISSED THE " << lines_passed << "th LINE" << std::endl;
			sw.start(); // Restart stopwatch
		}
		else if (sw.read() > MIN_TIME_DETECTION_BIG && this->read_sensor()) // It meets a new line
		{		
			++lines_passed;
			std::cout << "I FOUND THE " << lines_passed << "th LINE" << std::endl;
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
			std::cout << "I MISSED THE " << lines_passed << "th LINE" << std::endl;
			sw.start(); // Restart stopwatch
		}
		else if (sw.read() > MIN_TIME_DETECTION_SMALL && this->read_sensor()) // It meets a new line
		{
			++lines_passed;
			std::cout << "I FOUND THE " << lines_passed << "th LINE" << std::endl;
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

void turntable::turn_to_nest_pid(int next_nest)
{
	int target_angle = (next_nest-1) * 360 / TOTAL_NUMBER_NESTS;	
	turn_angle_pid(target_angle);
}

void turntable::turn_angle_pid(int target_angle)
{
  double min = -127.0 * TURNTABLE_ROTATION_CALIBRATION; //-255
  double max = 127.0 * TURNTABLE_ROTATION_CALIBRATION; //+255
	PID pid = PID(TURNTABLE_dt, max, min, TURNTABLE_Kp, TURNTABLE_Kd, TURNTABLE_Ki);
	int val = this->read_pot();
	
	double inc = 360;
	std::cout << "Target angle " << target_angle << std::endl;
	while (std::abs((float)(val - target_angle)) > TURNTABLE_tol && std::abs((float)inc) > 5*TURNTABLE_tol) {
	        std::cout << "val is " << val << std::endl;
		std::cout << "target angle is " << val << std::endl;
		inc = pid.calculate(target_angle, val);
		std::cout << "inc is " << inc << std::endl;
		val = this->read_pot();
		if (inc>0)
			this->turn(true, (int) (inc /  TURNTABLE_ROTATION_CALIBRATION));
		else
			this->turn(false, (int) (-inc / TURNTABLE_ROTATION_CALIBRATION));
	}
	this->turn(true, 0);
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

void turntable::push_nest()
{
    //Retracts the arm (should be unneeded) then pushes the nest and retracts the arm after
    unsigned char current_reading = rlink.request(READ_PORT_1);
    rlink.command(WRITE_PORT_1, current_reading & (~0b00000010));
    delay(100);
    rlink.command(WRITE_PORT_1, current_reading | 0b00000010);
    delay(200);
    rlink.command(WRITE_PORT_1, current_reading & (~0b00000010));
}
