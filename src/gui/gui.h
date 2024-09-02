#pragma once
#include <Arduino.h>
#include "common/mavlink.h"
#include <Adafruit_GFX.h>
#include <Adafruit_SharpMem.h>
#include <mat.h>
#include <SPI.h>
#include "gfx.h"

class DisplayElements {	
	public:
		String gcsCompass;
		String mavStatusMsg;
		float attitudeRoll = 0;
		bool isConnected = false;
};

class Menu {
	public:
		const char *name;
		class MenuElement *elements;
		int elementCount;
		int selectedElement;
		class Menu *parent;
};

class MenuElement {
	public:
		const char *name;
		void (*function)(void * parameters);
		void * parameters;
		class Menu *subMenu;
};

void menu();
void artificialHorizon();
void guiTask(void * parameters);
void degTask(void * parameters);
void guiInitialize();
void sendAttitude(mavlink_attitude_t attitude);