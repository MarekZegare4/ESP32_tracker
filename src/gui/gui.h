#pragma once
#include <Arduino.h>
#include "common/mavlink.h"
#include <Adafruit_GFX.h>
#include <Adafruit_SharpMem.h>
#include <mat.h>
#include <SPI.h>
#include "gfx.h"

class displayElements {	
	public:
		String gcsCompass;
		String mavStatusMsg;
		float attitudeRoll = 0;
		bool isConnected = false;
};

void menu();
void artificialHorizon();
void displayTask(void * parameters);
void degTask(void * parameters);
void displayInitialize();
void sendAttitude(mavlink_attitude_t attitude);