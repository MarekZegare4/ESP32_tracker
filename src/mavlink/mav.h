#pragma once

class Packet {
  public:
    int len;
    uint8_t buf[256];
};

class UavDataGPS {
  public:
    uint32_t global_lat;
    uint32_t global_lon;
    uint32_t global_alt;
};

class UavDataAttitude {
  public:
    float pitch;
    float roll;
    float yaw;
};

bool GetConnectionStatus();
UavDataAttitude GetUavAttitude();
UavDataGPS GetUavGPS();
void serialFlushRx();
void CreateQueue();
Packet AccessQueue();
bool PacketAvailable();
void MavlinkInitialize();
void SendHeartbeatTask(void * parameters);
void DecodeTelemetryTask(void * parameters);