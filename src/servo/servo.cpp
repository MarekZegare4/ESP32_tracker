#include <Arduino.h>
#include "servo.h"
#include "geoTransform/geoTransform.h"
#include <SCServo.h>

#define SERVO_RX 18
#define SERVO_TX 19

SMS_STS st;

extern int32_t uavLat;
extern int32_t uavLon;
extern int32_t uavAlt;



std::tuple<float, float, float> distAziElev;

void ServoInitialize(){
    Serial1.begin(1000000, SERIAL_8N1, SERVO_RX, SERVO_TX);
    st.pSerial = &Serial1;
}

void ServoTask(void * parameters) {
    for(;;) {
        wgs84_coord uavPos;
        uavPos.lat = uavLat;
        uavPos.lon = uavLon;
        uavPos.alt = uavAlt;
        distAziElev = dist_azi_elev(uavPos, uavPos);

    }
}