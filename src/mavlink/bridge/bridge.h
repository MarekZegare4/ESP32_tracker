#pragma once
#include <Arduino.h>
#include "mavlink/mav.h"

void wifiBridgeInitialize();
void bluetoothBridgeInitialize();
void sendBTMsg(uint8_t *buf, uint8_t len);
void sendUDPPacket(uint8_t *buf, uint8_t len);
void wifiBridgeTask(void * parameters);
void bluetoothBridgeTask(void * parameters);