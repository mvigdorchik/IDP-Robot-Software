#include "robot.h"

robot::robot()
{
    state = DEFAULT;
}

void robot::take_path(int path[], int size)
{
    for(int i = 0; i < size; i++)
    {
	switch(path[i])
	{
	case 0:
	    this->turn_to_line(0,true);
	    break;
	case 1:
	    this->turn_to_line(1,true);
	    break;
	case 2:
	    break;
	case 3:
	    this->turn_to_line(1, false);
	    break;
	default:
	    return; //should never happen unless function is used incorrectly
	}
	this->follow_line_straight();
    }
}

void robot::go_time(int distance, unsigned char speed)
{
    stopwatch sw;
    this->go(speed);
    sw.start();
    while(sw.read() < ((distance-INERTIA_CALIBRATION) * DISTANCE_CALIBRATION)); //TODO: Adjust the calibration for speed differences!!!
    this->go(0);
    delay(800); //Allow Inertia to stop things
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

void robot::follow_line_straight()
{
    unsigned char sensor_reading;
    unsigned char old_sensor_reading = 0;
    int integral_reading = 0;

    this->go(127);
    while(this->read_line_sensors() == 0b111); //Ensures if it starts on a junction it wont detect that one
    while(1) //TODO: Add in some kind of timeout condition
    {
	old_sensor_reading = sensor_reading;
	sensor_reading = this->read_line_sensors();
	//std::cout << "Sensor Reading" << std::bitset<8>(sensor_reading) << std::endl;
	switch(sensor_reading)
	{
	case 0b010:
	    this->go(127);
	    break;
	case 0b110:
	    this->turn(1, FOLLOWER_KP - FOLLOWER_KI * integral_reading);
	    integral_reading -= 1;
	    break;
	case 0b011:
	    this->turn(0, FOLLOWER_KP + FOLLOWER_KI * integral_reading);
	    integral_reading += 1;
	    break;
	case 0b100:
	    this->turn(1, FOLLOWER_KP2 - FOLLOWER_KI * integral_reading);
	    integral_reading -= 2;
	    break;
	case 0b001:
	    this->turn(0, FOLLOWER_KP2 + FOLLOWER_KI * integral_reading);
	    integral_reading += 2;
	    break;
	case 0b111:
	    break;
	default:
	    //this->go(127);
	    break;
	}
	if(sensor_reading == 0b111 && old_sensor_reading == 0b111)
	{
	    this->go(0);
	    break;
	}

	//std::cout << FOLLOWER_KI*integral_reading << std::endl;
    }
    this->go(0);
    //this->go_time(DISTANCE_TO_CENTER, 127);
}

void robot::go(unsigned char speed)
{
    rlink.command(BOTH_MOTORS_GO_OPPOSITE, speed);
}

//TODO: Improve this function immensely as it assumes thing is going full speed and is rather stupid
void robot::turn(bool right, unsigned char speed)
{
    unsigned char adjusted_speed = speed > 127 ? 127 : speed; //Sets a limit to speed of turn
    this->go(127);
    if(right)
    {
	rlink.command(MOTOR_2_GO, 255 - adjusted_speed);
    }
    else
    {
	rlink.command(MOTOR_1_GO, 127 - adjusted_speed);
    }
}

void robot::turn_to_line(bool right, bool move_forward)
{
    if(move_forward) this->go_time(DISTANCE_TO_CENTER, 127); //move forward before turning if requested
    unsigned char sensor_reading = 0;
    this->turn_angle(right ? 30: 330); //Turns a bit initially so it doesnt intersect the line its already on
    rlink.command(BOTH_MOTORS_GO_SAME, right ? 255 : 127);
    while(sensor_reading == 0)
    {
	sensor_reading = this->read_line_sensors();
    }
    this->go(127); //Syncs up motors so they move together
    this->go(0);
}
