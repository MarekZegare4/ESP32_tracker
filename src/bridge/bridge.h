#pragma once
#include <Arduino.h>
#include "mavlink/mav.h"

void WiFiBridgeInitialize();
void WiFiBridgeTask(void * parameters);
void BtBridgeTask(void * parameters);
