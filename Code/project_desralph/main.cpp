#include "hFramework.h"
#include "MPU9250.h"
#include "hCyclicBuffer.h"
#include "hCloudClient.h"
#include "hPID.h"
#include <cmath>
#include <cstdlib>

#include "ui.h"
#include "utils.h"
#include "settings.h"

#include "main.h"

#define LOG(x,...) MAIN_CONSOLE.printf(x, ##__VA_ARGS__)

#if BOARD(CORE2)
hGPIO pin5V(91);
#endif

hSemaphore sem_calibrate;
bool motorsOn = true;

MPU9250 mpu(MPU_SENS);

static hMotor& hMotL = MOTOR_LEFT;
static hMotor& hMotR = MOTOR_RIGHT;
static IServo* servo = &hServoModule.servo1;

class Settings {
public:
	float Theta0;
	float KTheta, KPos, KSpeed, KdeltaEnc;
	float Kp, Ki, Kd;
	float EncDiffMul;
};

Settings settings = {
	.Theta0 = THETA0,
	.KTheta = 1.35,
	.KPos = 9.4,
	.KSpeed = -5,
	.KdeltaEnc = 0.3,
	.Kp = 5000,
	.Ki = 300,
	.Kd = 29000,
	.EncDiffMul = 1,
};


int32_t lastPos[10];
float speedNow = 0, targetSpeed = 0;
volatile float moveForward = 0, moveTurnRight = 0;
float thetaNow = 0;
float posRef = 0, encRef = 0;

EState state;
uint32_t stateChangeTime = 0;

void setState(EState state)
{
	if (::state != state) {
		::state = state;
		stateChangeTime = sys.getRefTime();
	}
}

uint32_t getStateTime()
{
	return sys.getRefTime() - stateChangeTime;
}

void makeStand()
{
	if (state == STATE_DRIVE) {
		openLeg();
		setState(STATE_WAIT_STOP);
	}
}

void makeDrive()
{
	if (state == STATE_STANDING) {
		setState(STATE_STARTING);
	}
}

bool toggleDriveStand()
{
	if (state == STATE_DRIVE) {
		// platform.ui.button("leg_toggle").setText("click here to drive");
		makeStand();
		return 1;
	} else if (state == STATE_STANDING) {
		// platform.ui.button("leg_toggle").setText("click here to stop");
		makeDrive();
		return 0;
	}
	return 0;
}

void setMove(float forward, float turnRight)
{
	moveForward = forward;
	moveTurnRight = turnRight;
}

void setMotorsPower(bool active, float left, float right)
{
	float voltage = sys.getSupplyVoltage();
	float voltageRatio = 8.0f / voltage;

	if (active == true) {
		hMotL.setPower(left * voltageRatio);
		hMotR.setPower(right * voltageRatio);
	} else {
		hMotL.setPower(0);
		hMotR.setPower(0);
	}
}

void setMotorsPower(bool active, float power)
{
	setMotorsPower(active, power, power);
}

void mainProc()
{
	hCyclicBuffer<int32_t> lastPositions(5, lastPos);

	//set addr pin of MPU9250
	MPU_SENS.pin2.setOut();
	MPU_SENS.pin2.write(1);

	setState(STATE_STANDING);
	openLeg();

	sys.log("reset IMU power  ... ");
	pin5V.write(0);
	sys.delay(100);
	pin5V.write(1);
	sys.delay(50);
	sys.log("done\r\n");

	resetI2C(MPU_SENS);

	sys.log("init IMU  ... ");
	for (;;) {
		bool ok = initMPU(mpu);
		if (!ok)
			sys.log("error\r\n");
		else
			sys.log("done\r\n");
		break;
		sys.delay(300);
		LED3.toggle();
	}

	LED1.on();
	LED2.on();
	LED3.on();

	float stabLastTheta = 0;
	int validCount = 0;
	float gyroOffset = 0;

	hPID pid;


	// MPU stabilization
	sys.log("IMU stabilization ... ");
	for (;;) {
		mpu.waitForData();

		if (!mpu.process()) {
			printf("error\r\n");
			continue;
		}

		float gyrData, accData;
		mpuToAccelGyro(mpu, accData, gyrData);

		thetaNow = 0.99f * thetaNow + 0.01f * accData;
		float diff = abs(thetaNow - stabLastTheta);
		if (diff <= 0.0001f) {
			validCount++;
			gyroOffset += gyrData;
			if (validCount == 100) {
				gyroOffset /= validCount;
				sys.log("done, gyroOffset = %f\r\n", gyroOffset);
				break;
			}
		} else {
			validCount = 0;
			gyroOffset = 0;
		}
		stabLastTheta = thetaNow;
	}

	for (int i = 0; i < 3; i++) {
		LED1.toggle();
		LED2.toggle();
		LED3.toggle();
		sys.delay(50);
	}

	LED1.off();
	LED2.off();
	LED3.off();
	releaseLeg();

	sys.log("ready to control!\r\n");

	int o = 0;
	for (;;) {
		float dt = 0.005f;
		mpu.waitForData();
		o++;
		// printf("%d %d\r\n", o, sys.getRefTime()/1000);

		if (!mpu.process()) {
			printf("sth went wrong\r\n");
			continue;
		}

		float gyrData, accData;
		mpuToAccelGyro(mpu, accData, gyrData);

		if(motorsOn == false) {
		    posRef = 0;
		    MOTOR_RIGHT.resetEncoderCnt();
		    MOTOR_LEFT.resetEncoderCnt();
		    
		    speedNow = 0;
		    targetSpeed = 0;
		    
		    pid.reset();
		}

		thetaNow = 0.999f * (thetaNow + gyrData * dt) + 0.001f * accData;
		// printf("  %f theta %f\r\n", gyrData, thetaNow);
		float errTheta = thetaNow - settings.Theta0;

		if (abs(moveForward) < 0.1 && abs(targetSpeed) < 0.15)
			targetSpeed = 0;
		else
			targetSpeed += (moveForward - targetSpeed) * dt;
		posRef += targetSpeed * dt;
		encRef -= moveTurnRight * dt;

		float posNow = getPosition();
		speedNow = (posNow - lastPositions[0]) / (dt * lastPositions.size());
		lastPositions.push_back(posNow);
		
		float errPos = posRef - posNow;
		errPos = saturate(errPos, -2000.0f, 2000.0f);

		float errSpeed = speedNow - targetSpeed;
		errSpeed = saturate(errSpeed, -1000.0f, 1000.0f);

		float errNowTheta = settings.KTheta * errTheta;
		float errNowPos = -(settings.KPos / 100000.0f) * errPos;
		float errNowSpeed = -(settings.KSpeed / 500000.0f) * errSpeed;

		float errNow = errNowTheta + errNowPos + errNowSpeed;

		pid.setCoeffs(settings.Kp, settings.Ki, settings.Kd);
		float motorPower = pid.update(errNow, 1);

		int32_t encDiff = hMotR.getEncoderCnt() - hMotL.getEncoderCnt();
		float turnAdd = settings.KdeltaEnc * (encDiff - encRef);
		int16_t motorPowerLeft = motorPower + turnAdd;
		int16_t motorPowerRight = motorPower - turnAdd;

		if (abs(errTheta) > 0.4f || errPos >= 10000) {
			state = STATE_STARTING;
		}

		switch (state) {
		case STATE_STANDING:
			setMotorsPower(motorsOn, 0);
			break;
		case STATE_STARTING:
			if (abs(errTheta) < 0.5f)
				setMotorsPower(motorsOn, errTheta > 0 ? 900 : -900);
			else
				setMotorsPower(motorsOn, 0);

			if (abs(errTheta) < 0.04f) {
				setState(STATE_DRIVE);
				errPos = posRef = encRef = posNow = targetSpeed = 0;
				pid.reset();
				hMotL.resetEncoderCnt();
				hMotR.resetEncoderCnt();
				for (int i = 0; i < lastPositions.size(); i++)
					lastPositions.push_back(0);
				closeLeg();
				sys.log("stand\r\n");
			}
			break;
		case STATE_DRIVE:
			setMotorsPower(motorsOn, motorPowerLeft, motorPowerRight);
			if (getStateTime() >= 600) {
				releaseLeg();
			}
			break;
		case STATE_WAIT_STOP:
			setMotorsPower(motorsOn, motorPowerLeft, motorPowerRight);

			if (abs(speedNow) < 10.0f && getStateTime() >= 200) {
				setMotorsPower(motorsOn, -300);
				setState(STATE_WAIT_LEG);
			}
			break;
		case STATE_WAIT_LEG:
			if (getStateTime() >= 200) {
				setMotorsPower(motorsOn, 0);
			}
			if (getStateTime() >= 600) {
				releaseLeg();
				setState(STATE_STANDING);
			}
			break;
		}

		if (sem_calibrate.take(0)) {
			settings.Theta0 = thetaNow;
			sys.getStorage().store(0, settings.Theta0);
			sys.log("calibration done, new THETA0 = %f\r\n", settings.Theta0);
			platform.ui.console("cl1").printf("calibration done, new THETA0 = %f\r\n", settings.Theta0);
		}

		if (MAIN_CONSOLE.available() > 0) {
			char letter = MAIN_CONSOLE.getch();
			switch (letter) {
			case 'q':
				settings.Kp     += 50;
				LOG("kp=%d\r\n", (int)settings.Kp);
				break;
			case 'a':
				settings.Kp     -= 50;
				LOG("kp=%d\r\n", (int)settings.Kp);
				break;
			case 'w':
				settings.Ki     += 4;
				LOG("ki=%d\r\n", (int)settings.Ki);
				break;
			case 's':
				settings.Ki     -= 4;
				LOG("ki=%d\r\n", (int)settings.Ki);
				break;
			case 'e':
				settings.Kd     += 100;
				LOG("kd=%d\r\n", (int)settings.Kd);
				break;
			case 'd':
				settings.Kd     -= 100;
				LOG("kd=%d\r\n", (int)settings.Kd);
				break;
			case 'r':
				settings.Theta0 += 0.005;
				LOG("th0=%f\r\n", settings.Theta0);
				break;
			case 'f':
				settings.Theta0 -= 0.005;
				LOG("th0=%f\r\n", settings.Theta0);
				break;
			case 'u':
				settings.KTheta += 0.1;
				LOG("K_th0=%f\r\n", settings.KTheta);
				break;
			case 'j':
				settings.KTheta -= 0.1;
				LOG("K_th0=%f\r\n", settings.KTheta);
				break;
			case 'i':
				settings.KPos   += 0.2;
				LOG("K_Y=%f\r\n", settings.KPos);
				break;
			case 'k':
				settings.KPos   -= 0.2;
				LOG("K_Y=%f\r\n", settings.KPos);
				break;
			case 'o':
				settings.KSpeed += 1;
				LOG("K_dYdt=%f\r\n", settings.KSpeed);
				break;
			case 'l':
				settings.KSpeed -= 1;
				LOG("K_dYdt=%f\r\n", settings.KSpeed);
				break;
			case 'v':
				sys.log("voltage %f ", sys.getSupplyVoltage());
				sys.log("pos: %5d ", getPosition());
				sys.log("ref %f ", posRef);
				sys.log("theta %f\r\n", thetaNow);
				sys.log("e1 %d e2 %d\r\n", hMotL.getEncoderCnt(), hMotR.getEncoderCnt());
				break;
			case 'c':
				toggleDriveStand();
				break;
			case 'b':
				moveForward = 1800;
				break;
			case 'n':
				moveForward = 0;
				break;
			case 'm':
				moveForward = -1800;
				break;
			case ',':
				moveTurnRight = 600;
				break;
			case '.':
				moveTurnRight = 0;
				break;
			case '/':
				moveTurnRight = -600;
				break;
			case '!':
				settings.Theta0 = thetaNow;
				sys.getStorage().store(0, settings.Theta0);
				sys.log("calibration done, new THETA0 = %f\r\n", settings.Theta0);
				break;
			}
		}
	}
}

int btnMode = 0;
uint32_t pressTime1 = 0;
uint32_t pressTime2 = 0;



void hMain()
{
	uiInit();

	sem_calibrate.take(0);

	Serial.init(115200);
	Serial.printf("\r\n==================\r\n");

	sys.setSysLogDev(&devNull);
	sys.setLogDev(&MAIN_CONSOLE);

	sys.log("Start!!!\r\nSuppy voltage = %f [V]\r\n", sys.getSupplyVoltage());
	if (sys.getSupplyVoltage() < 11) {
		sys.disableUsbCharging();
		printf("disabling USB charging\r\n");
	}

	float storageVal = 0;

#if THETA0_LOAD_FROM_STORAGE == 1
	sys.getStorage().load(0, storageVal);
	printf("settings.Theta0 in storage %f\r\n", storageVal);
	if (storageVal == 0x00) {

	} else {
		settings.Theta0 = storageVal;
	}
#endif
	printf("settings.Theta0 in program %f\r\n", settings.Theta0);


	// sys.disableSyslog();
	// sys.setLogDev(&devNull);

	hServoModule.enablePower();

	hMotor::setPWMFreq(PWM_freq_hMotor_ID_3, PWM_freq_21_kHz);
	hMotor::setPWMFreq(PWM_freq_hMotor_ID_4, PWM_freq_21_kHz);

	// hMotL.setEncoderPolarity(Polarity::Reversed);
	hMotR.setEncoderPolarity(Polarity::Reversed);
	hMotR.setMotorPolarity(Polarity::Reversed);
//	hMotL.setMotorPolarity(Polarity::Reversed);

	sys.taskCreate(mainProc, 4, 800);

	hBtn1.setOnPressHandler([]() {
		pressTime2 = sys.getRefTime();
	});

	hBtn2.setOnPressHandler([]() {
		pressTime1 = sys.getRefTime();
	});

	hBtn1.setOnReleaseHandler([]() {
		if (btnMode == 1) {
			settings.Theta0 -= 0.01f;
			platform.ui.console("cl1").printf("theta %f\r\n", settings.Theta0);
		} else if (btnMode == 2) {
			if (pressTime2 != 0 && sys.getRefTime() - pressTime2 <= 700) {
				openLeg();
			}
		} else {
			toggleDriveStand();
		}
		pressTime2 = 0;
	});
	hBtn2.setOnReleaseHandler([]() {
		if (btnMode == 1) {
			if (pressTime1 != 0 && sys.getRefTime() - pressTime1 <= 700) {
				settings.Theta0 += 0.01f;
				platform.ui.console("cl1").printf("theta %f\r\n", settings.Theta0);
			}
		} else if (btnMode == 2) {
			closeLeg();
		}
		pressTime1 = 0;
	});

	while (1) {
		uiProcess();
		sys.delay(100);

		if (btnMode == 0) {
			if (pressTime1 != 0 && sys.getRefTime() - pressTime1 >= 1000) {
				ledCaptured = true;
				LED1.off();
				LED2.on();
				LED3.off();
				btnMode = 1;
				pressTime1 = 0;
			}
			if (pressTime2 != 0 && sys.getRefTime() - pressTime2 >= 1000) {
				ledCaptured = true;
				LED1.on();
				LED2.on();
				LED3.off();
				btnMode = 2;
				pressTime2 = 0;
			}
		}
		if (btnMode == 1) {
			if (pressTime1 != 0 && sys.getRefTime() - pressTime1 >= 1000) {
				btnMode = 0;
				pressTime1 = 0;
				ledCaptured = false;
				sys.getStorage().store(0, settings.Theta0);
			}
		}
		if (btnMode == 2) {
			if (pressTime2 != 0 && sys.getRefTime() - pressTime2 >= 1000) {
				btnMode = 0;
				pressTime2 = 0;
				ledCaptured = false;
			}
		}
	}
}


