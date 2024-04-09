#pragma once
#include <Arduino.h>
#include "mavlink/mav.h"

void wifiBridgeInitialize();
void wifiBridgeTask(void * parameters);
void btBridgeTask(void * parameters);
