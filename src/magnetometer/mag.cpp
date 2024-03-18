#include <Arduino.h>
#include <DFRobot_BMM150.h>
#include <SPI.h>
#include "mag.h"
#include "display/display.h"

extern displayElements dispElem;

DFRobot_BMM150_I2C bmm150(&Wire, I2C_ADDRESS_4);

void MagInitialize() {
    while(!Serial);
    while(bmm150.begin()){
    Serial.println("bmm150 init failed, Please try again!");
    delay(1000);
    } 
    Serial.println("bmm150 init success!");
    bmm150.setOperationMode(BMM150_POWERMODE_NORMAL);
    bmm150.setPresetMode(BMM150_PRESETMODE_HIGHACCURACY);
    bmm150.setRate(BMM150_DATA_RATE_10HZ);
    bmm150.setMeasurementXYZ();
}

void MagTask(void * parameters) {
    for(;;){
        sBmm150MagData_t magData = bmm150.getGeomagneticData();
        // Serial.print("mag x = "); Serial.print(magData.x); Serial.println(" uT");
        // Serial.print("mag y = "); Serial.print(magData.y); Serial.println(" uT");
        // Serial.print("mag z = "); Serial.print(magData.z); Serial.println(" uT");
        float compassDegree = bmm150.getCompassDegree();
        dispElem.gcsCompass = String(compassDegree);
        // Serial.print("the angle between the pointing direction and north (counterclockwise) is:");
        // Serial.println(compassDegree);
        // Serial.println("--------------------------------");
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}