#include "robot.h"

robot::robot()
{
    state = DEFAULT;
}

void robot::go_time(int distance, unsigned char speed)
{
    stopwatch sw;
    rlink.command(BOTH_MOTORS_GO_OPPOSITE, speed);
    sw.start();
    while(sw.read() < distance * DISTANCE_CALIBRATION * speed);
    rlink.command(BOTH_MOTORS_GO_SAME, 0);
    sw.stop();
}

void robot::turn_angle(int degrees)
{
    stopwatch sw;
    unsigned char speed = degrees > 180 ? 127 : 255;
    float turn_amount = degrees > 180 ? 360 - degrees : degrees; 
    rlink.command(BOTH_MOTORS_GO_SAME, speed);
    sw.start();
    while(sw.read() < ROTATION_CALIBRATION * turn_amount);
    rlink.command(BOTH_MOTORS_GO_SAME, 0);
    sw.stop();
}

unsigned char robot::read_line_sensors()
{
    unsigned char result = 0;
    rlink.command(WRITE_PORT_0, 255);
    result = rlink.request(READ_PORT_0);
    result &= 0b00000111; //Ignore any other sensor on that bus.
    return result;
}

void robot::follow_line_straight(int distance)
{
    unsigned char sensor_reading;
    stopwatch sw_overall;
    sw_overall.start();
    this->go(127);
    while(sw_overall.read() < distance * DISTANCE_CALIBRATION * 127)
    {
	sensor_reading = this->read_line_sensors();
	//sensor_reading = 0b100;
	//std::cout << "Sensor Reading" << std::bitset<8>(sensor_reading) << std::endl;
	switch(sensor_reading)
	{
	case 0b010:
	    this->go(127);
	    break;
	case 0b110:
	    this->turn(1, FOLLOWER_GAIN);
	    break;
	case 0b011:
	    this->turn(0, FOLLOWER_GAIN);
	    break;
	case 0b100:
	    this->turn(1, 1.8*FOLLOWER_GAIN);
	    break;
	case 0b001:
	    this->turn(0, 1.8*FOLLOWER_GAIN);
	    break;
	default:
	    //this->go(127);
	    break;
	}
    }
    this->go(0);
}

void robot::go(unsigned char speed)
{
    rlink.command(BOTH_MOTORS_GO_OPPOSITE, speed);
}

//TODO: Improve this function immensely as it assumes thing is going full speed and is rather stupid
void robot::turn(bool right, unsigned char speed)
{
    if(right)
    {
	rlink.command(MOTOR_2_GO, 255 - speed);
    }
    else
    {
	rlink.command(MOTOR_1_GO, 127 - speed);

    }
}
