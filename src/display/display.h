#pragma once
#include <Arduino.h>

class displayElements {	
	public:
		String gcsCompass;
};

void menu();
void artificialHorizon();
void ScreenTask(void * parameters);
void DegTask(void * parameters);
void DisplayInitialize();