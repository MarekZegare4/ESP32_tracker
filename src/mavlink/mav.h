#pragma once

class packet {
  public:
    int len;
    uint8_t buf[256];
};

void serialFlushRx();
void CreateQueue();
packet AccessQueue();
bool PacketAvailable();
void MavlinkInitialize();
void SendHeartbeatTask(void * parameters);
void DecodeTelemetryTask(void * parameters);