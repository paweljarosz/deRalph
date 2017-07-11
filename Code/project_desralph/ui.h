#ifndef __UI_H__
#define __UI_H__

struct pressedKeys_t
{
	bool W, S, A, D;
};

extern bool ledCaptured;

void processKeys();

void uiInit();
void uiProcess();

#endif
