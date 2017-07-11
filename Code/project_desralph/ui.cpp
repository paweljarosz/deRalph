#include "ui.h"
#include <stdlib.h>

#include <hFramework.h>
#include <hCloudClient.h>
#include <debug.h>

#include "main.h"
#include "settings.h"

#define MYKEY_W 0
#define MYKEY_S 1
#define MYKEY_A 2
#define MYKEY_D 3


volatile pressedKeys_t pressedKeys = { false, false, false, false };

bool ledCaptured = false;

int targer_speed = SPEED1;
int targer_turn_speed = TURN_SPEED1;

extern hSemaphore sem_calibrate;
extern bool motorsOn;

void onValueChangeEvent(hId id, const char* data)
{
	;
}
void onButtonEvent(hId id, ButtonEventType type)
{
	platform.ui.console("cl1").printf("%d: button event - id=%s, type=%d\r\n", sys.getRefTime(), id.c_str(), type);

	bool isPressed = (type == ButtonEventType::Pressed);
	if (id == "btn_up") {
		pressedKeys.W = isPressed;
	}
	if (id == "btn_down") {
		pressedKeys.S = isPressed;
	}
	if (id == "btn_left") {
		pressedKeys.A = isPressed;
	}
	if (id == "btn_right") {
		pressedKeys.D = isPressed;
	}
	if (id == "btn_left_up") {
		pressedKeys.W = isPressed;
		pressedKeys.A = isPressed;
	}
	if (id == "btn_right_up") {
		pressedKeys.W = isPressed;
		pressedKeys.D = isPressed;
	}
	if (id == "btn_right_down") {
		pressedKeys.S = isPressed;
		pressedKeys.D = isPressed;
	}
	if (id == "btn_left_down") {
		pressedKeys.S = isPressed;
		pressedKeys.A = isPressed;
	}
	if (id == "stop") {
		makeStand();
	}
	if (id == "g1") {
		makeDrive();
		targer_speed = SPEED1;
		targer_turn_speed = TURN_SPEED1;
	}
	if (id == "g2") {
		makeDrive();
		targer_speed = SPEED2;
		targer_turn_speed = TURN_SPEED2;
	}
	if (id == "g3") {
		makeDrive();
		targer_speed = SPEED3;
		targer_turn_speed = TURN_SPEED3;
	}


	if (isPressed) {
		if (id == "leg_toggle") {
			toggleDriveStand();
		}
		if (id == "btn_calibrate") {
			if (motorsOn == false) {
				sem_calibrate.give();
				LED1.toggle();
			} else {
			    platform.ui.console("cl1").printf("at first turn off motors to calibrate\r\n");
			}
		}
		if (id == "btn_motor_onoff") {
			if (motorsOn == true) {
				motorsOn = false;
				platform.ui.button("btn_motor_onoff").setText("turn on motors");
			} else {
				motorsOn = true;
				platform.ui.button("btn_motor_onoff").setText("turn off motors");
			}
		}
	}
}
void onKeyEvent(KeyEventType type, KeyCode code)
{
	platform.ui.console("cl1").printf("%d: button event - type=%d, code=%d\r\n", sys.getRefTime(), type, code);

	bool isPressed = (type == KeyEventType::Pressed);
	if (code == KeyCode::Key_W || code == KeyCode::Up)
		pressedKeys.W = isPressed;
	if (code == KeyCode::Key_S || code == KeyCode::Down)
		pressedKeys.S = isPressed;
	if (code == KeyCode::Key_A || code == KeyCode::Left)
		pressedKeys.A = isPressed;
	if (code == KeyCode::Key_D || code == KeyCode::Right)
		pressedKeys.D = isPressed;

	if (isPressed) {
		if (code == KeyCode::Key_G) {
			toggleDriveStand();
		}
	}
}
void cfgHandler()
{
	platform.ui.loadHtml({Resource::URL, "/web_ui/ui.html"});
	platform.ui.video.enable();
}

void processKeys()
{
	int turnStep = targer_turn_speed;
	float moveForward = 0, moveTurnRight = 0;

	if (pressedKeys.A && !pressedKeys.S) {
		if (!ledCaptured)
			LED1.on();
		moveTurnRight += turnStep;
	} else if (pressedKeys.A && pressedKeys.S) {
		if (!ledCaptured)
			LED1.on();
		moveTurnRight -= turnStep;
	} else {
		if (!ledCaptured)
			LED1.off();
	}

	if (pressedKeys.D && !pressedKeys.S) {
		if (!ledCaptured)
			LED3.on();
		moveTurnRight -= turnStep;
	}  else if (pressedKeys.D && pressedKeys.S) {
		if (!ledCaptured)
			LED3.on();
		moveTurnRight += turnStep;
	} else {
		if (!ledCaptured)
			LED3.off();
	}
	if (pressedKeys.W) {
		if (!ledCaptured)
			LED2.on();
		moveForward -= targer_speed;
	} else if (pressedKeys.S) {
		moveForward += targer_speed;
		static int time = 0;
		if (sys.getRefTime() - time >= 100) {
			time = sys.getRefTime();
			if (!ledCaptured)
				LED2.toggle();
		}
	} else {
		if (!ledCaptured)
			LED2.off();
	}
	setMove(moveForward, moveTurnRight);
}

void statusChanged(PlatformStatus status)
{
	if (status == PlatformStatus::Disconnected) {
		pressedKeys.W = false;
		pressedKeys.S = false;
		pressedKeys.A = false;
		pressedKeys.D = false;
	}
}

void uiInit()
{
	platform.begin(&RPi);

	platform.ui.setProjectId("@@@PROJECT_ID@@@");
	platform.ui.configHandler = cfgHandler;
	platform.ui.onKeyEvent = onKeyEvent;
	platform.ui.onButtonEvent = onButtonEvent;
	platform.ui.onValueChangeEvent = onValueChangeEvent;
}

hElapsedTimer uiUpdateTimer(3000);

#define CASE(x) case x: return #x;
#define ABRA_CADABRA switch(state)

char* getStateDesc(EState state)
{
	ABRA_CADABRA {
		CASE(STATE_STANDING)
		CASE(STATE_STARTING)
		CASE(STATE_DRIVE)
		CASE(STATE_WAIT_STOP)
		CASE(STATE_WAIT_LEG)
	}
	return "";
}

void uiProcess()
{
	processKeys();
	extern EState state;
	static EState stateOld = state;

	if (uiUpdateTimer.hasElapsed()) {
		platform.ui.label("lb_bat").setText("%2f V", sys.getSupplyVoltage());
		platform.ui.progressBar("pb_bat").setValue(sys.getSupplyVoltageMV() / 15); //supply voltage milivolts
	}
	if (stateOld != state) {
		stateOld = state;
		platform.ui.console("cl1").printf("%d: new state = %s\r\n", sys.getRefTime(), getStateDesc(stateOld));
	}
}
