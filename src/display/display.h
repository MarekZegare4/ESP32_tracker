#pragma once
#include <Arduino.h>

class displayElements {	
	public:
		String gcsCompass;
		String mavStatusMsg;
};

void Menu();
void ArtificialHorizon();
void DisplayTask(void * parameters);
void DegTask(void * parameters);
void DisplayInitialize();