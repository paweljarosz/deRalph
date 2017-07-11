#include "utils.h"

#include <cmath>

#include "main.h"
#include "settings.h"
#include "hCloudClient.h"

static hMotor& hMotL = MOTOR_LEFT;
static hMotor& hMotR = MOTOR_RIGHT;
static IServo* servo = &hServoModule.servo1;

void resetI2C(ISensor_i2c& sens)
{
	sys.log("reset i2c ... ");
	sens.selectGPIO();
	sens.getPin3().setOutOD_pu();
	sens.getPin4().setOutOD_pu();
	sens.getPin4().write(1);
	for (;;)
	{
		while (sens.getPin4().read() == 0)
		{
			sens.getPin3().write(0);
			sys.delay(1);
			sens.getPin3().write(1);
			sys.delay(1);
			sys.log(".");
		}
		if (sens.getPin4().read() == 1)
		{
			sys.delay(10);
			if (sens.getPin4().read() == 1)
				break;
		}
	}

	if (sens.getPin3().read() == 0)
	{
		sys.log("err\r\n");
	} else {
	    sys.log("done\r\n");
	}
}


#define E(str,x) \
	if (!(x)) { \
		sys.log("%s\r\n", str); \
		return false; }

bool initMPU(MPU9250& mpu)
{
	E("init", mpu.init());
	E("gyro", mpu.setGyroScale(MPU9250::GyroScale::DPS_2000));
	E("int", mpu.enableInterrupt());
	return true;
}

void mpuToAccelGyro(MPU9250& mpu, float& accel, float& gyro)
{
	int16_t ax, ay, az, gx, gy, gz;
	mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

	// printf("%d %d\r\n", ax,az);
	// printf("%f\r\n", (float)gy/32768.0*2000.0);
	gyro = -d2r((float)gy / 32768.0f * 2000.0f);
	accel = atan2(ax, az);
}

int32_t getPosition()
{
	return (hMotR.getEncoderCnt() + hMotL.getEncoderCnt()) / 2;
}

void openLeg()
{
    // platform.ui.label("lb_leg_status").setText("1");
	servo->setWidth(LEG_OPENED_WIDTH);
}
void closeLeg()
{
    // platform.ui.label("lb_leg_status").setText("0");
	servo->setWidth(LEG_CLOSED_WIDTH);
}
void releaseLeg()
{
	servo->setWidth(0);
}
