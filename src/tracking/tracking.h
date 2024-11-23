#include <Arduino.h>
#include "geoTransform/geoTransform.h"

void trackingTask(void * parameters);
void trackingInitialize();
int readCurrent();
void servoDemo();
float readHeading();

class TrackerDataGPS {
    public:
        int32_t latitude;
        int32_t longitude;
        int32_t altitude;
        byte fixType;
        int8_t satCount;
        uint8_t day;
        uint8_t month;
        uint16_t year;
        AngleValues angles;
};

TrackerDataGPS getTrackerGPS();