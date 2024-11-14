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
#include <SensorFusion.h>
#include "gui/gui.h"
#include "mavlink/mav.h"
#include "tracking.h"
#include "tracking/geoTransform/geoTransform.h"
#include "imu_calibration/imu_calibration.h"
#include "kalman_filter/kalman.h"

// https://www.waveshare.com/wiki/ST3020_Servo#Overview

#define SERVO_RX 16
#define SERVO_TX 17
#define SERVO_TILT_PIN 4
#define myCS 15              // Define the GPIO pin number for the SPI Chip Select. Change this if required
#define myClockSpeed 1000000 // Define what SPI clock speed to use. Change this if required

Servo servo;
// Serial Servo ST3020
SMS_STS st;
// Magnetometer
Adafruit_LIS3MDL lis3mdl;
// Accelerometer and Gyroscope
Adafruit_LSM6DS3TRC lsm6ds3trc;
// GNSS
SFE_UBLOX_GNSS_SPI myGNSS; // SFE_UBLOX_GNSS_SPI uses SPI. For I2C or Serial, see Example1 and Example2
SPIClass spiPort(HSPI);

WMM_Tinier declination;
TrackerDataGPS gpsData;
AngleValues sDistAziElev;
float sCompassDegree;

// Sensor fusion
SF fusion;
float mx, my, mz, ax, ay, az, gx, gy, gz;
float pitch, roll, yaw;
float deltat;
float heading;

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
  lis3mdl.setDataRate(LIS3MDL_DATARATE_155_HZ);
  lis3mdl.setPerformanceMode(LIS3MDL_HIGHMODE);
  lsm6ds3trc.begin_I2C();
  lsm6ds3trc.setGyroDataRate(LSM6DS_RATE_104_HZ);
  lsm6ds3trc.setAccelDataRate(LSM6DS_RATE_104_HZ);
  while (myGNSS.begin(spiPort, myCS, myClockSpeed) == false) // Connect to the u-blox module using the port, chip select pin and speed defined above
  {
    delay(1000);
  }
  myGNSS.setSPIOutput(COM_TYPE_UBX); // Set the SPI port to output UBX only (turn off NMEA noise)
  myGNSS.setNavigationFrequency(5);  // Set the update rate to 5Hz
  // myGNSS.newCfgValset(VAL_LAYER_RAM); // Disable NMEA GxGSV messages
  // myGNSS.addCfgValset(UBLOX_CFG_GNSS_, 1);
  // myGNSS.sendCfgValset();
}

int angleToServo(int angle)
{
  return  360 / 4096 * angle;
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
    // Orientation calculation
    // Update IMU readings
    sensors_event_t mag;
    sensors_event_t accel;
    sensors_event_t gyro;
    sensors_event_t temp;
    lis3mdl.getEvent(&mag);
    lsm6ds3trc.getEvent(&accel, &gyro, &temp);

    CalibratedIMUData mag_data = calibratedMagData(mag.magnetic.x, mag.magnetic.y, mag.magnetic.z);
    CalibratedIMUData accel_data = calibratedAccData(accel.acceleration.x, accel.acceleration.y, accel.acceleration.z);
    mx = mag_data.x;
    my = mag_data.y;
    mz = mag_data.z;
    ax = accel_data.x;
    ay = accel_data.y;
    az = accel_data.z;
    gx = gyro.gyro.x;
    gy = gyro.gyro.y;
    gz = gyro.gyro.z;
    deltat = fusion.deltatUpdate();
    fusion.MadgwickUpdate(gx, gy, gz, ax, ay, az, mx, my, mz, deltat);

    pitch = pitchKalman(fusion.getPitch());
    roll = rollKalman(fusion.getRoll());
    heading = headingKalman(tiltCompensatedHeading(mag_data.x, mag_data.y, mag_data.z, pitch, roll));

    // Read data from GNSS module
    gpsData.latitude = myGNSS.getLatitude();
    gpsData.longitude = myGNSS.getLongitude();
    gpsData.altitude = myGNSS.getAltitudeMSL(); // Altitude above Mean Sea Level
    gpsData.fixType = myGNSS.getFixType();
    gpsData.satCount = myGNSS.getSIV();

    // Wgs84Coord trackerPos;
    // Wgs84Coord uavPos;
    // AngleValues angles;
    // // Update the tracker and UAV positions
    // trackerPos.lat = gpsData.latitude;
    // trackerPos.lon = gpsData.longitude;
    // trackerPos.alt = gpsData.altitude;

    // uavPos.alt = getUavGPS()->global_lat;
    // uavPos.lon = getUavGPS()->global_lon;
    // uavPos.alt = getUavGPS()->global_alt;

    // // Calculate the magnetic declination
    // float dec = declination.magneticDeclination(trackerPos.lat, trackerPos.lon, myGNSS.getYear(), myGNSS.getMonth(), myGNSS.getDay());

    // angles = DistAziElev(trackerPos, uavPos);
    // gpsData.angles = angles;

    // CalibratedMagData data = calibratedData(mag.magnetic.x, mag.magnetic.y, mag.magnetic.z);
    // Serial.print(String(data.x) + " " + String(data.y) + " " + String(data.z) + ";" + "\n");
    // Serial.print(String(accel.acceleration.x) + " " + String(accel.acceleration.y) + " " + String(accel.acceleration.z) + ";" + "\n");
    // CalibratedIMUData data = calibratedAccData(accel.acceleration.x, accel.acceleration.y, accel.acceleration.z);
    // Serial.print(String(data.x) + " " + String(data.y) + " " + String(data.z) + ";" + "\n");

    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}