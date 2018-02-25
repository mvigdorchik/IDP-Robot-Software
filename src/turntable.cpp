#include "turntable.h"
#include "pid.h"

//Default constructor
turntable::turntable()
{
	
}

unsigned char turntable::read_sensor()
{
	unsigned char result = 0;
	rlink.command(WRITE_PORT_0, 255);
	result = rlink.request(READ_PORT_0);
	result &= 0b0001000; //Ignore any other sensor on that bus. 
	return result >> 3;
}


void turntable::turn(bool clockwise, int speed)
{
	unsigned char adjusted_speed = speed > 127 ? 127 : speed; //Sets a limit to speed of turn
	adjusted_speed = clockwise ? adjusted_speed : adjusted_speed + 128;
	rlink.command(MOTOR_3_GO, adjusted_speed);
}

void turntable::initial_align()
{
	stopwatch sw;
	int lines = 0;
	int interval = 0;
	this->turn(true, 127);
	sw.start();
	while (1)
	{
		if (this->read_sensor() && sw.read()>MIN_TIME_DETECTION) // When it meets a new line
		{
			if (!lines || sw.read() > DOUBLE_LINE_TIMEOUT) // It's the first line in the series, so make lines 1
			{
				lines = 1;
			}
			else // It wasn't the first line and it was close to previous one, so increment lines
			{
				++lines;
				if (lines == 3) // Found all three lines, exit loop
					break;
			}
			sw.start(); // Restart stopwatch
		}
	}
	sw.start();
	this->turn(false, 40);
	while (sw.read() < TIME_TO_REVERSE) {} // Wait to reverse
	this->turn(true, 0);
	sw.stop();
	current_nest = 1;
}

void turntable::turn_to_nest(int current_nest, int next_nest)
{
	bool direction;
	int diff = (next_nest - current_nest) < 0 ? (next_nest - current_nest + 8) : (next_nest - current_nest);
	direction = diff < 5 ? true : false;
	int degrees = int(360 / 8 * diff) > 180 ? 360 - int(360 / 8 * diff) : int(360 / 8 * diff);
	//std::cout << "direction: " << direction << '\n' << "degrees: " << degrees << '\n'; turn_angle_pid(direction, degrees);
	//turn_angle_pid(direction, degrees);
	 turn_angle_time(direction, degrees);  //If you want to use the timed one, fast but not so smooth as PID
}

void turntable::turn_angle_pid(bool clockwise, int degrees)
{
	double min = -127.0 / TURNTABLE_ROTATION_CALIBRATION;
	double max = 127.0 / TURNTABLE_ROTATION_CALIBRATION;
	PID pid = PID(TURNTABLE_dt, max, min, TURNTABLE_Kp, TURNTABLE_Kd, TURNTABLE_Ki);

	int val = 0;
	double inc = 0;
	while (std::abs((float) (val-degrees)) < TURNTABLE_tol && inc < TURNTABLE_tol) {
		inc = pid.calculate(degrees, val);
		val += inc; //TODO or not: replace <<val += inc>> with actual reading from sensor
		if (inc>0)
			this->turn(clockwise, inc / TURNTABLE_dt * TURNTABLE_ROTATION_CALIBRATION);
		else 
			this->turn(!clockwise, -inc / TURNTABLE_dt * TURNTABLE_ROTATION_CALIBRATION);
	}
}

void turntable::turn_angle_time(bool clockwise, int degrees)
{
	stopwatch sw;
	sw.start();
	this->turn(clockwise, 127);
	while(sw.read() < ((degrees - TURNTABLE_INERTIA_CALIBRATION +0) / TURNTABLE_ROTATION_SPEED)){}
	this->turn(clockwise, 0);	
	sw.stop();
}
