#pragma once
#include <Arduino.h>

class packet {
  public:
    int len;
    uint8_t buf[256];
};

void CreateQueue();
packet AccessQueue();
void BridgeInitialize();
void serialFlushRx();
void BridgeTask(void * parameters);