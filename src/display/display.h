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
		float attitudeRoll;
};

void Menu();
void ArtificialHorizon();
void DisplayTask(void * parameters);
void DegTask(void * parameters);
void DisplayInitialize();
void SendAttitude(mavlink_attitude_t attitude);