#include "robot.h"

robot::robot()
{
    state = DEFAULT;
    current_loc = "Start";
}

void robot::take_path(int path[], int size)
{
    for(int i = 0; i < size; i++)
    {
	switch(path[i])
	{
	case 0:
	    this->turn_to_line(0,true,true);
	    break;
	case 1:
	    this->turn_to_line(1,true,true);
	    break;
	case 2:
	    break;
	case 3:
	    this->turn_to_line(0, false,true);
	    break;
	default:
	    return; //should never happen unless function is used incorrectly
	}
	this->follow_line_straight(1);
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

void robot::follow_line_straight(int expected_distance)
{
    stopwatch junction_timeout;
    unsigned char sensor_reading;
    unsigned char old_sensor_reading = 0;
    unsigned char latest_nonzero_reading = 0; //Used to remember how it got lost
    int integral_reading = 0;
    int lost_line_count = 0; //Keeps track of how many cycles no line was detected
    int MAX_LOST_LINE_COUNT = 100; //Number of times line can be lost before surrender

    this->go(127);
    junction_timeout.start();
    while(this->read_line_sensors() == 0b111); //Ensures if it starts on a junction it wont detect that one
    while(1) //TODO: Add in some kind of timeout condition
    {
	old_sensor_reading = sensor_reading;
	sensor_reading = this->read_line_sensors();
	if(sensor_reading != 0)
	    latest_nonzero_reading = sensor_reading;
	
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
	    lost_line_count++;
	    break;
	}
	if(sensor_reading == 0b111 && old_sensor_reading == 0b111 && junction_timeout.read() > 800)
	{
	    //junction timeout prevents it from stopping if it hits a line at steep angle to start out
	    this->go(0);
	    break;
	}

	if(sensor_reading != 0)
	    lost_line_count = 0;
	if(lost_line_count > MAX_LOST_LINE_COUNT)
	{
	    if(!(this->recover_line(this->current_loc, latest_nonzero_reading) == current_loc))
	    {
		std::cout << "IM TOTALLY LOST" << std::endl;
		this->follow_line_straight(1);
		break;
	    }
	    std::cout << "Im exactly where I was before" << std::endl;
	    this->follow_line_straight(1);

	    return;
	}
    }
    this->go(0);
    //this->go_time(DISTANCE_TO_CENTER, 127);
}

std::string robot::recover_line(std::string old_loc, unsigned char latest_reading)
{
    stopwatch sw;
    sw.start();
    if(this->turn_to_line(!(latest_reading >> 2), false, false) && sw.read() < 60*ROTATION_CALIBRATION) //the first param is the 3rd bit of reading, 0 means it needs to turn right, second is to ensure it doesnt roate too long
	return old_loc; //Return old location because its quite confident its in the same spot as before

    //Begin actually determining where it is
    this->go_to_line(10000);
    return "";
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

bool robot::turn_to_line(bool right, bool move_forward, bool delay_sensing)
{
    stopwatch sw;
    
    if(move_forward) this->go_time(DISTANCE_TO_CENTER, 127); //move forward before turning if requested
    unsigned char sensor_reading = 0;
    if(delay_sensing) this->turn_angle(right ? 45: 315); //Turns a bit initially so it doesnt intersect the line its already on
    rlink.command(BOTH_MOTORS_GO_SAME, right ? 255 : 127);
    sw.start();
    while(sensor_reading == 0)
    {
	sensor_reading = this->read_line_sensors();
	if(sw.read() > (330 + 30*(!delay_sensing)) * ROTATION_CALIBRATION)
	    return false;
    }
    this->go(127); //Syncs up motors so they move together
    this->go(0);
    sw.stop();
    return true;
}

void robot::go_to_line(int timeout)
{
    stopwatch sw; //Have a timeout
    sw.start();
    this->go(127);
    while(this->read_line_sensors() == 0)
    {
	if(sw.read() > timeout)
	{
	    this->turn_angle(70); //70 is kind of arbitrary
	    this->go_to_line(2*timeout);
	    return;
	}
    }
    this->go(0);
}
