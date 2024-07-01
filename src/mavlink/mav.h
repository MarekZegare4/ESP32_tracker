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

class UavSysText {
  public:
    int severity = 0;
    char text[50] = "";
};

bool getConnectionStatus();
UavDataAttitude getUavAttitude();
UavDataGPS getUavGPS();
UavSysText getUavSysText();
void serialFlushRx();
void createQueue();
Packet accessQueue();
bool packetAvailable();
void mavlinkInitialize();
void sendMavlinkMsgTask(void * parameters);
void decodeTelemetryTask(void * parameters);