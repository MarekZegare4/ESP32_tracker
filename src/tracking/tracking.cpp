#include <Arduino.h>
#include <SCServo.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_LIS3MDL.h>
#include <Adafruit_LSM6DS3TRC.h>
#include <Adafruit_Sensor.h>
#include <ESP32Servo.h>
#include <SparkFun_u-blox_GNSS_v3.h>
#include <WMM_Tinier.h>
#include "gui/gui.h"
#include "mavlink/mav.h"
#include "tracking.h"
#include "tracking/geoTransform/geoTransform.h"
#include "mag_calibration/mag_calibration.h"

// https://www.waveshare.com/wiki/ST3020_Servo#Overview

#define SERVO_RX 16
#define SERVO_TX 17
#define SERVO_TILT_PIN 4
#define myCS 15              // Define the GPIO pin number for the SPI Chip Select. Change this if required
#define myClockSpeed 1000000 // Define what SPI clock speed to use. Change this if required

Servo servo;
SMS_STS st;
Adafruit_LIS3MDL lis3mdl;
Adafruit_LSM6DS3TRC lsm6ds3trc;
SFE_UBLOX_GNSS_SPI myGNSS; // SFE_UBLOX_GNSS_SPI uses SPI. For I2C or Serial, see Example1 and Example2
WMM_Tinier declination;
SPIClass spiPort(HSPI);
TrackerDataGPS gpsData;

AngleValues sDistAziElev;
float sCompassDegree;

float getCompassDegree()
{
  return sCompassDegree;
}

void trackingInitialize()
{
  Serial1.begin(1000000, SERIAL_8N1, SERVO_RX, SERVO_TX);
  declination.begin();
  st.pSerial = &Serial1;
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  servo.setPeriodHertz(50);                 // Standard 50hz servo
  servo.attach(SERVO_TILT_PIN, 1000, 2000); // attaches the servo on pin 4 to the servo object
  servo.write(0);
  lis3mdl.begin_I2C();
  lis3mdl.setRange(LIS3MDL_RANGE_4_GAUSS);
  lis3mdl.setDataRate(LIS3MDL_DATARATE_5_HZ);
  lsm6ds3trc.begin_I2C();
  while (myGNSS.begin(spiPort, myCS, myClockSpeed) == false) // Connect to the u-blox module using the port, chip select pin and speed defined above
  {
    delay(1000);
  }
  myGNSS.setSPIOutput(COM_TYPE_UBX); // Set the SPI port to output UBX only (turn off NMEA noise)
  myGNSS.setNavigationFrequency(5);  // Set the update rate to 10Hz
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
  st.WritePosEx(1, -2048, 1500);
  servo.write(0);
  delay(1000);
}

int readCurrent()
{
  return st.ReadCurrent(1);
}

TrackerDataGPS getTrackerGPS()
{
  return gpsData;
}

sensor_t sensor;
float x, y, z;
int i = 0;
void trackingTask(void *parameters)
{
  for (;;)
  {
    // Read data from GNSS module
    if (myGNSS.getPVT() == true)
    {
      gpsData.latitude = myGNSS.getLatitude();
      gpsData.longitude = myGNSS.getLongitude();
      gpsData.altitude = myGNSS.getAltitudeMSL(); // Altitude above Mean Sea Level
      gpsData.fixType = myGNSS.getFixType();
      gpsData.satCount = myGNSS.getSIV();
    }

    Wgs84Coord trackerPos;
    Wgs84Coord uavPos;
    AngleValues angles;
    // Update the tracker and UAV positions
    trackerPos.lat = gpsData.latitude;
    trackerPos.lon = gpsData.longitude;
    trackerPos.alt = gpsData.altitude;

    uavPos.alt = getUavGPS()->global_lat;
    uavPos.lon = getUavGPS()->global_lon;
    uavPos.alt = getUavGPS()->global_alt;

    // Calculate the magnetic declination
    float dec = declination.magneticDeclination(trackerPos.lat, trackerPos.lon, myGNSS.getYear(), myGNSS.getMonth(), myGNSS.getDay());

    angles = DistAziElev(trackerPos, uavPos);
    gpsData.angles = angles;

    sensors_event_t event;
    lis3mdl.getEvent(&event);
    CalibratedMagData data = calibratedData(event.magnetic.x, event.magnetic.y, event.magnetic.z);
    Serial.print(String(data.x) + " " + String(data.y) + " " + String(data.z) + ";" + "\n");

    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}