#pragma once
#include <Arduino.h>

class packet {
  public:
    int len;
    uint8_t buf[256];
};

void CreateQueue();
packet AccessQueue();
void WiFiBridgeInitialize();
void BtBridgeInitialize();
void serialFlushRx();
void WiFiBridgeTask(void * parameters);
void BtBridgeTask(void * parameters);
