#include <Arduino.h>
#include "tracking.h"
#include "geoTransform/geoTransform.h"
#include <SCServo.h>
#include <DFRobot_BMM150.h>
#include <SPI.h>
#include "gui/gui.h"
#include "mavlink/mav.h"

// https://www.waveshare.com/wiki/ST3020_Servo#Overview

#define SERVO_RX 18
#define SERVO_TX 19

SMS_STS st;
DFRobot_BMM150_I2C bmm150(&Wire, I2C_ADDRESS_4);

static AngleValues sDistAziElev;
static float sCompassDegree;

float GetCompassDegree() {
    return sCompassDegree;
}

void TrackingInitialize(){
    Serial1.begin(1000000, SERIAL_8N1, SERVO_RX, SERVO_TX);
    st.pSerial = &Serial1;
    bmm150.begin();
    bmm150.setOperationMode(BMM150_POWERMODE_NORMAL);
    bmm150.setPresetMode(BMM150_PRESETMODE_HIGHACCURACY);
    bmm150.setRate(BMM150_DATA_RATE_10HZ);
    bmm150.setMeasurementXYZ();
}

void TrackingTask(void * parameters) {
    for(;;) {
        Wgs84Coord uavPos;
        sDistAziElev = DistAziElev(uavPos, uavPos);
        sCompassDegree = bmm150.getCompassDegree();
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}