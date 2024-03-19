#pragma once
#include <Arduino.h>

class displayElements {	
	public:
		String gcsCompass;
		String mavStatusMsg;
};

void menu();
void artificialHorizon();
void ScreenTask(void * parameters);
void DegTask(void * parameters);
void DisplayInitialize();