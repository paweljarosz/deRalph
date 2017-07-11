#ifndef __MAIN_H__
#define __MAIN_H__

// extern IServo* servo;

void setMove(float forward, float turnRight);

void makeStand();
void makeDrive();
bool toggleDriveStand();

typedef enum
{
	STATE_STANDING,
	STATE_STARTING,
	STATE_DRIVE,
	STATE_WAIT_STOP,
	STATE_WAIT_LEG,
} EState;

#endif
