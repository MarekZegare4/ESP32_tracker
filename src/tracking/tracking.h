#include <Arduino.h>
#include "geoTransform/geoTransform.h"

void trackingTask(void * parameters);
void trackingInitialize();
float getCompassDegree();
int readCurrent();
void servoDemo();

class TrackerDataGPS {
    public:
        int32_t latitude;
        int32_t longitude;
        int32_t altitude;
        byte fixType;
        int8_t satCount;
        AngleValues angles;

};

TrackerDataGPS getTrackerGPS();