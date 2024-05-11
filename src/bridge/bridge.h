#pragma once
#include <Arduino.h>
#include "mavlink/mav.h"

void wifiBridgeInitialize();
void bluetoothBridgeInitialize();
void wifiBridgeTask(void * parameters);
void bluetoothBridgeTask(void * parameters);