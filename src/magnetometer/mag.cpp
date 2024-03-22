#include <Arduino.h>
#include <DFRobot_BMM150.h>
#include <SPI.h>
#include "mag.h"
#include "display/display.h"

extern displayElements dispElem;

DFRobot_BMM150_I2C bmm150(&Wire, I2C_ADDRESS_4);

void MagInitialize() {
    bmm150.setOperationMode(BMM150_POWERMODE_NORMAL);
    bmm150.setPresetMode(BMM150_PRESETMODE_HIGHACCURACY);
    bmm150.setRate(BMM150_DATA_RATE_10HZ);
    bmm150.setMeasurementXYZ();
}

void MagTask(void * parameters) {
    for(;;){
        sBmm150MagData_t magData = bmm150.getGeomagneticData();
        float compassDegree = bmm150.getCompassDegree();
        dispElem.gcsCompass = String(compassDegree);
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}