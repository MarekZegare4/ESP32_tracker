#include <Arduino.h>
#include "servo.h"
#include "geoTransform/geoTransform.h"

extern int32_t uavLat;
extern int32_t uavLon;
extern int32_t uavAlt;

std::tuple<float, float, float> distAziElev;

void ServoTask(void * parameters) {
    for(;;) {
        wgs84_coord uavPos;
        uavPos.lat = uavLat;
        uavPos.lon = uavLon;
        uavPos.alt = uavAlt;
        distAziElev = dist_azi_elev(uavPos, uavPos);

    }
}