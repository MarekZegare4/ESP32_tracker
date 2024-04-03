#pragma once

class packet {
  public:
    int len;
    uint8_t buf[256];
};

class uav_data {
  public:
    uint32_t global_lat;
    uint32_t global_lon;
    uint32_t global_alt;
    float pitch;
    float roll;
    float yaw;
};

bool GetConnectionStatus();
std::tuple<float, float, float> GetUavAttitude();
std::tuple<uint32_t, uint32_t, uint32_t> GetUavPosition();
void serialFlushRx();
void CreateQueue();
packet AccessQueue();
bool PacketAvailable();
void MavlinkInitialize();
void SendHeartbeatTask(void * parameters);
void DecodeTelemetryTask(void * parameters);