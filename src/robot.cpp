#include "robot.h"
#define DEBUG 1
 
robot::robot()
{
    state = DEFAULT;
    current_loc = "Start";
    orientation = NORTH;
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
	this->follow_line_straight(15000, true);
    }
}

void robot::go_time(int distance, unsigned char speed, bool use_inertia)
{
    stopwatch sw;
    int inertia = use_inertia ? INERTIA_CALIBRATION : 0;
    this->go(speed);
    sw.start();
    while(sw.read() < ((distance-inertia) * DISTANCE_CALIBRATION)); //TODO: Adjust the calibration for speed differences!!!
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
    //In the 0th bus expanser, bit 7 and bit 6 and bit 4 are outputs
    unsigned char result = 0;
    //TODO If we are confident that nothing funny happens, can remove this bitmapping stuff and just read, trusting that the inputs are set to 1
    unsigned char current_state = rlink.request(READ_PORT_0);
    current_state |= 0b00101111; //Ensures that the output pins don't change their state
    rlink.command(WRITE_PORT_0, current_state);
    result = rlink.request(READ_PORT_0);
    result &= 0b00000111; //Ignore any other sensor on that bus.
    return result;
}

void robot::follow_line_straight(int expected_distance, bool recover)
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
	    this->turn(1, FOLLOWER_KP - FOLLOWER_KI * integral_reading, false);
	    integral_reading -= 1;
	    break;
	case 0b011:
	    this->turn(0, FOLLOWER_KP + FOLLOWER_KI * integral_reading, false);
	    integral_reading += 1;
	    break;
	case 0b100:
	    this->turn(1, FOLLOWER_KP2 - FOLLOWER_KI * integral_reading, false);
	    integral_reading -= 2;
	    break;
	case 0b001:
	    this->turn(0, FOLLOWER_KP2 + FOLLOWER_KI * integral_reading, false);
	    integral_reading += 2;
	    break;
	case 0b111:
	    break;
	default:
	    //this->go(127);
	    lost_line_count++;
	    break;
	}
	if(sensor_reading == 0b111 && old_sensor_reading == 0b111 && junction_timeout.read() > 500)
	{
	    //junction timeout prevents it from stopping if it hits a line at steep angle to start out
	    this->go(0);
	    break;
	}

	if(sensor_reading != 0)
	    lost_line_count = 0;
	if(lost_line_count > MAX_LOST_LINE_COUNT)
	{
	    int time_when_lost = junction_timeout.read();
	    if(!recover)
	    {
		return; //Its just lost forever, no attempt to recover
	    }
	    if(!(this->recover_line(current_loc, latest_nonzero_reading) == current_loc))
	    {
		if(DEBUG) std::cout << "IM TOTALLY LOST" << std::endl;
		this->follow_line_straight(expected_distance - time_when_lost/DISTANCE_CALIBRATION,  true);
		break;
	    }
	    if(DEBUG) std::cout << "Im exactly where I was before" << std::endl;
	    this->follow_line_straight(expected_distance - time_when_lost/DISTANCE_CALIBRATION, true);

	    return;
	}
	if(junction_timeout.read() > (expected_distance * DISTANCE_CALIBRATION))
	{
	    //This is useful for following the line a short distance
	    return;
	}
    }
    this->go(0);
    //this->go_time(DISTANCE_TO_CENTER, 127);
}

void robot::line_follow_reverse(int distance)
{
    unsigned char sensor_reading;
    stopwatch sw;
    int integral_reading = 0;

    this->go(255);
    sw.start();
    while(sw.read() < distance * DISTANCE_CALIBRATION)
    {
	sensor_reading = this->read_line_sensors();
	switch(sensor_reading)
	{
	case 0b010:
	    this->go(255);
	    break;
	case 0b110:
	    this->turn(1, FOLLOWER_KP - FOLLOWER_KI * integral_reading, true);
	    integral_reading -= 1;
	    break;
	case 0b011:
	    this->turn(0, FOLLOWER_KP + FOLLOWER_KI * integral_reading, true);
	    integral_reading += 1;
	    break;
	case 0b100:
	    this->turn(1, FOLLOWER_KP2 - FOLLOWER_KI * integral_reading, true);
	    integral_reading -= 2;
	    break;
	case 0b001:
	    this->turn(0, FOLLOWER_KP2 + FOLLOWER_KI * integral_reading, true);
	    integral_reading += 2;
	    break;
	case 0b111:
	    this->go(255);
	    break;
	default:
	    //this->go(255);
	    // lost_line_count++;
	    break;
	}
    }
}

std::string robot::recover_line(std::string old_loc, unsigned char latest_reading)
{
    stopwatch sw;
    sw.start();
    if(this->turn_to_line(!(latest_reading >> 2), false, false) && sw.read() < 60*ROTATION_CALIBRATION) //the first param is the 3rd bit of reading, 0 means it needs to turn right, second is to ensure it doesnt roate too long
	return old_loc; //Return old location because its quite confident its in the same spot as before

    //Begin actually determining where it is
    this->go_to_line(10000);
    orientation = LOST;
    return "LOST";
}

void robot::traverse_curve()
{
    this->follow_line_straight(50000, false); //TODO Figure out length of curve
    this->turn_angle(310);
    this->go_to_line(100000);
    this->turn_to_line(1, true, false);
    this->follow_line_straight(10000, true);
    
    orientation = NORTH;
    current_loc = "D2"; //TODO Use correct location name
}

void robot::return_to_curve()
{
    //This function will move the robot forward, turn 90 degrees, then go until it sees the line.
    //Presumably the line it sees should be the curve
    this->go_time(2*DISTANCE_TO_CENTER, 127, true); //Move forward about 100mm, this can be changed.
    this->turn_angle(90);
    this->go_to_line(5000);
    this->follow_line_straight(1000, 1);

    orientation = SOUTH;
    current_loc = "C2";
}

// void robot::go(unsigned char speed)
// {
//     rlink.command(BOTH_MOTORS_GO_OPPOSITE, speed);
// }
void robot::go(unsigned char speed)
{
    //This alternate version of go will adjust one motor slightly to make up for wheel imbalance
    unsigned char reverse_speed = speed > 127 ? speed - 128 : speed + 128;  
    rlink.command(MOTOR_1_GO, speed);
    rlink.command(MOTOR_2_GO, reverse_speed - speed/100);
}

//TODO: Improve this function immensely as it assumes thing is going full speed and is rather stupid
void robot::turn(bool right, unsigned char speed, bool reverse)
{
    unsigned char adjusted_speed;
    adjusted_speed = speed > 127 ? 127 : speed; //Sets a limit to speed of turn
    this->go(reverse ? 255 : 127); //Ensure that it adjusts to the correct direction
    if(right)
    {
	if(!reverse)
	    rlink.command(MOTOR_2_GO, 255 - adjusted_speed);
	else
	    rlink.command(MOTOR_1_GO, 255 - adjusted_speed);
    }
    else
    {
	if(!reverse)
	    rlink.command(MOTOR_1_GO, 127 - adjusted_speed);
	else
	    rlink.command(MOTOR_2_GO, 127 - adjusted_speed);
    }
}

bool robot::turn_to_line(bool right, bool move_forward, bool delay_sensing)
{
    stopwatch sw;
    
    if(move_forward) this->go_time(DISTANCE_TO_CENTER, 127, true); //move forward before turning if requested
    unsigned char sensor_reading = 0;
    if(delay_sensing) this->turn_angle(right ? 45: 315); //Turns a bit initially so it doesnt intersect the line its already on
    rlink.command(BOTH_MOTORS_GO_SAME, right ? 255 : 127);
    sw.start();
    while(sensor_reading == 0)
    {
	sensor_reading = this->read_line_sensors();
	if(sw.read() > (315 + 45*(!delay_sensing)) * ROTATION_CALIBRATION)
	    return false;
    }
    this->go(127); //Syncs up motors so they move together
    this->go(0);
    sw.stop();

    if(orientation != LOST)
	orientation = static_cast<Direction>(right ? (orientation + 1) % 4 : (orientation +3) % 4);
 
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
	    orientation = LOST;
	    this->turn_angle(70); //70 is kind of arbitrary
	    this->go_to_line(2*timeout);
	    return;
	}
    }
    this->go(0);
}

int robot::read_beacon()
{
    rlink.command(WRITE_PORT_1,0b00000100);
    bool last_state = false;
    bool current_state = false;
    int transition_count = 0;
    int transition_count2 = 0; //Used to verify the first reading
    stopwatch sw;

    sw.start();
    while(sw.read() < 1500)
    {
	delay(10);
	current_state = (bool)(rlink.request(READ_PORT_1) & 0b000000100);
	if(last_state && !current_state)
	{
	    if(transition_count == 0)
		sw.start();
	    transition_count  += 1;
	}
	last_state=current_state;

    }
    
    // sw.start();
    // last_state = false;
    // current_state = false;
    // while(sw.read() < 2000)
    // {
    // 	delay(10);
    // 	current_state = (bool)(rlink.request(READ_PORT_1) & 0b000000100);
    // 	if(last_state && !current_state)
    // 	{
	    // if(transition_count2 == 0)
	    // 	sw.start();
    // 	    transition_count2 += 1;
    // 	}
    // 	last_state=current_state;

    // }

    sw.stop();
    if(transition_count != 0 /* && transition_count == transition_count2 */)
	return transition_count;
    else
	return this->read_beacon();
}
