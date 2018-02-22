#include "turntable.h"
#include "pid.h"

using namespace std;

//Default constructor
turntable::turntable()
{
	
}

void turntable::turn_angle(bool clockwise, int degrees)
{
	int min = clockwise ? 0 : 128;
	int max = min + 127;

	PID pid = PID(TURNTABLE_dt, max, min, TURNTABLE_Kp, TURNTABLE_Kd, TURNTABLE_Ki);
	int val = degrees;
	double inc = 0;
	while (val < TURNTABLE_tol && inc < TURNTABLE_tol) {
		double inc = pid.calculate(0, val);
		printf("val:% 7.3f inc:% 7.3f\n", val, inc);
		val += inc; //TODO replace <<val += inc>> with actual reading from sensor
		this->turn(clockwise, inc / TURNTABLE_dt);
	}
}

void turntable::turn(bool clockwise, int speed)
{
	unsigned char adjusted_speed = speed > 127 ? 127 : speed; //Sets a limit to speed of turn
	adjusted_speed = clockwise ? adjusted_speed : adjusted_speed + 128;
	rlink.command(MOTOR_3_GO, adjusted_speed);
}