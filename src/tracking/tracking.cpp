#include <Arduino.h>
#include <SCServo.h>
#include <Wire.h>
#include <Adafruit_LIS3MDL.h>
#include <Adafruit_LSM6DS3TRC.h>
#include <Adafruit_Sensor.h>
#include <ESP32Servo.h>
#include "gui/gui.h"
#include "mavlink/mav.h"
#include "tracking.h"
#include "tracking/geoTransform/geoTransform.h"
#include <SPI.h> //Needed for SPI to GNSS
#include <SparkFun_u-blox_GNSS_v3.h> //http://librarymanager/All#SparkFun_u-blox_GNSS_v3

// https://www.waveshare.com/wiki/ST3020_Servo#Overview

#define SERVO_RX 16
#define SERVO_TX 17
#define SERVO_TILT_PIN 4
#define myCS 15 // Define the GPIO pin number for the SPI Chip Select. Change this if required
#define myClockSpeed 1000000 // Define what SPI clock speed to use. Change this if required


Servo servo;
SMS_STS st;
Adafruit_LIS3MDL lis3mdl;
Adafruit_LSM6DS3TRC lsm6ds3trc;
SFE_UBLOX_GNSS_SPI myGNSS; // SFE_UBLOX_GNSS_SPI uses SPI. For I2C or Serial, see Example1 and Example2
SPIClass spiPort(HSPI);
TrackerDataGPS gpsData;

// DFRobot_BMM150_I2C bmm150(&Wire, I2C_ADDRESS_4);

static AngleValues sDistAziElev;
static float sCompassDegree;

float getCompassDegree()
{
    return sCompassDegree;
}

void trackingInitialize()
{
    Serial1.begin(1000000, SERIAL_8N1, SERVO_RX, SERVO_TX);
    st.pSerial = &Serial1;
    ESP32PWM::allocateTimer(0);
    ESP32PWM::allocateTimer(1);
    ESP32PWM::allocateTimer(2);
    ESP32PWM::allocateTimer(3);
    servo.setPeriodHertz(50);                 // Standard 50hz servo
    servo.attach(SERVO_TILT_PIN, 1000, 2000); // attaches the servo on pin 4 to the servo object
    servo.write(0);
    lis3mdl.begin_I2C(0x1C, &Wire);
    lsm6ds3trc.begin_I2C(0x6A, &Wire);
    while (myGNSS.begin(spiPort, myCS, myClockSpeed) == false) // Connect to the u-blox module using the port, chip select pin and speed defined above
  {
    delay(1000);
  }
  Serial.println("Connected to u-blox module!");
  myGNSS.setSPIOutput(COM_TYPE_UBX); // Set the SPI port to output UBX only (turn off NMEA noise)

    
    // lsm6ds3.begin_I2C();
    // bmm150.begin();
    // bmm150.setOperationMode(BMM150_POWERMODE_NORMAL);
    // bmm150.setPresetMode(BMM150_PRESETMODE_HIGHACCURACY);
    // bmm150.setRate(BMM150_DATA_RATE_10HZ);
    // bmm150.setMeasurementXYZ();
}

int angleToServo(int angle)
{
    return 4096 / 360 * angle;
}

void servoDemo()
{   
    st.WritePosEx(1, 2048, 1500);
    servo.write(45);
    delay(1000);
    st.WritePosEx(1, 0, 1500);
    servo.write(0);
    delay(1000);
}

int readCurrent(){
    return st.ReadCurrent(1);
}

TrackerDataGPS getTrackerGPS()
{
  return gpsData;
}

sensor_t sensor;
float x, y, z;
void trackingTask(void *parameters)
{
    for (;;)
    {
        if (myGNSS.getPVT() == true)
    {
      gpsData.latitude = myGNSS.getLatitude();
      gpsData.longitude = myGNSS.getLongitude();
      gpsData.altitude = myGNSS.getAltitudeMSL(); // Altitude above Mean Sea Level
      gpsData.fixType = myGNSS.getFixType();
      gpsData.satCount = myGNSS.getSIV();
    }
        // lis3mdl.getSensor(&sensor);
        // Serial.print("Sensor: "); Serial.println(sensor.name);
        // lsm6ds3trc.setAccelRange(LSM6DS_ACCEL_RANGE_2_G);
        // Serial.print("Accelerometer range set to: ");
        // switch (lsm6ds3trc.getAccelRange()) {
        // case LSM6DS_ACCEL_RANGE_2_G:
        //     Serial.println("+-2G");
        //     break;
        // case LSM6DS_ACCEL_RANGE_4_G:
        //     Serial.println("+-4G");
        //     break;
        // case LSM6DS_ACCEL_RANGE_8_G:
        //     Serial.println("+-8G");
        //     break;
        // case LSM6DS_ACCEL_RANGE_16_G:
        //     Serial.println("+-16G");
        //     break;
        // }

        // sensors_event_t accel;
        // sensors_event_t gyro;
        // sensors_event_t temp;
        // lsm6ds3trc.getEvent(&accel, &gyro, &temp);

        // Serial.print("\t\tTemperature ");
        // Serial.print(temp.temperature);
        // Serial.println(" deg C");

        // /* Display the results (acceleration is measured in m/s^2) */
        // Serial.print("\t\tAccel X: ");
        // Serial.print(accel.acceleration.x);
        // Serial.print(" \tY: ");
        // Serial.print(accel.acceleration.y);
        // Serial.print(" \tZ: ");
        // Serial.print(accel.acceleration.z);
        // Serial.println(" m/s^2 ");

        // /* Display the results (rotation is measured in rad/s) */
        // Serial.print("\t\tGyro X: ");
        // Serial.print(gyro.gyro.x);
        // Serial.print(" \tY: ");
        // Serial.print(gyro.gyro.y);
        // Serial.print(" \tZ: ");
        // Serial.print(gyro.gyro.z);
        // Serial.println(" radians/s ");
        // Serial.println();

        // lis3mdl.read();      // get X Y and Z data at once
        // // Then print out the raw data
        // Serial.print("\nX:  "); Serial.print(lis3mdl.x); 
        // Serial.print("  \tY:  "); Serial.print(lis3mdl.y); 
        // Serial.print("  \tZ:  "); Serial.println(lis3mdl.z); 

        // /* Or....get a new sensor event, normalized to uTesla */
        // sensors_event_t event; 
        // lis3mdl.getEvent(&event);
        // /* Display the results (magnetic field is measured in uTesla) */
        // Serial.print("\tX: "); Serial.print(event.magnetic.x);
        // Serial.print(" \tY: "); Serial.print(event.magnetic.y); 
        // Serial.print(" \tZ: "); Serial.print(event.magnetic.z); 
        // Serial.println(" uTesla ");
        // lsm6ds3.readAcceleration(x, y, z);
        // Serial.print("Acceleration: "); Serial.print(x); Serial.print(", "); Serial.print(y); Serial.print(", "); Serial.println(z);
        //Serial.println(st.Ping(1));
        // servo.write(90);
        // st.WritePosEx(1, 2048, 1500, 50);
        // delay(1000);
        // servo.write(45);
        // delay(1000);
        // servo.write(0);
        // st.WritePosEx(1, 4096, 1500, 50);
        // delay(1000);
        // st.WritePosEx(1, 0, 4000, 50);
        // delay(5000);
        // st.WritePosEx(1, 2048, 1500, 50);
        // delay(5000);
        // st.WritePosEx(1, 4096, 1500, 50);
        // delay(5000);
        // Wgs84Coord uavPos;
        // sDistAziElev = DistAziElev(uavPos, uavPos);
        // sCompassDegree = bmm150.getCompassDegree();
        // st.WritePosEx(1, angleToServo(sCompassDegree), 1500, 50); // To control the servo with ID 1, rotate it to position 1000 at a speed of 1500, with a start-stop acceleration of 50.
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}