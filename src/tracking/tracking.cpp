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
Wgs84Coord uavCoord;
Wgs84Coord trackerCoord;

// Sensor fusion
SF fusion;
float mx, my, mz, ax, ay, az, gx, gy, gz;
float pitch, roll, yaw;
float deltat;
float heading;

CalibratedIMUData mag_data;
CalibratedIMUData accel_data;

bool ready_to_track = false;

/*
@brief Corrects the angle to work in the current configuration
@param angle: wanted angle
*/
int servoAngle(int angle)
{
  return map(angle, 0, 90, 14, 170);
}

void trackingInitialize()
{
  Serial1.begin(1000000, SERIAL_8N1, SERVO_RX, SERVO_TX);
  declination.begin();
  st.pSerial = &Serial1;
  // st.CalibrationOfs(1);
  st.WritePosEx(1, 0, 1000, 100);
  ESP32PWM::allocateTimer(0);
  servo.setPeriodHertz(50);                 // Standard 50hz servo
  servo.attach(SERVO_TILT_PIN, 1000, 2000); // attaches the servo on pin 4 to the servo object
  servo.write(servoAngle(0));
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
}

void sendRawMagData()
{
  sensors_event_t mag;
  lis3mdl.getEvent(&mag);
  Serial.print(String(mag.magnetic.x) + " " + String(mag.magnetic.y) + " " + String(mag.magnetic.z) + "\n");
}

int angleToServo(int angle)
{
  return map(angle, 0, 360, 0, 4096);
}

int servoToAngle(int servo)
{
  return map(servo, 0, 4096, 0, 360);
}

void moveServoByAngle(int angle, int speed, int acc){
  int current_angle = servoToAngle(st.ReadPos(1));
  st.WritePosEx(1, angleToServo(current_angle + angle) % 4096, speed, acc);
}

void servoDemo()
{
  st.WritePosEx(1, 0, 1000, 100);
  delay(10000);
  st.WritePosEx(1, 2048, 1000, 100);
  delay(10000);
}

int readCurrent()
{
  return st.ReadCurrent(1);
}

TrackerDataGPS getTrackerGPS()
{
  return gpsData;
}

float readHeading()
{
  return heading;
}

sensor_t sensor;
float x, y, z;
int i = 0;
bool date_flag = false;
float mag_declination = 0;

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

    mag_data = calibratedMagData(mag.magnetic.x, mag.magnetic.y, mag.magnetic.z);
    accel_data = calibratedAccData(accel.acceleration.x, accel.acceleration.y, accel.acceleration.z);
    // mx = mag_data.x;
    // my = mag_data.y;
    // mz = mag_data.z;
    // ax = accel_data.x;
    // ay = accel_data.y;
    // az = accel_data.z;
    // gx = gyro.gyro.x;
    // gy = gyro.gyro.y;
    // gz = gyro.gyro.z;
    // deltat = fusion.deltatUpdate();
    // fusion.MadgwickUpdate(gx, gy, gz, ax, ay, az, mx, my, mz, deltat);

    // pitch = pitchKalman(fusion.getPitch());
    // roll = rollKalman(fusion.getRoll());

    // Read data from GNSS module
    gpsData.latitude = myGNSS.getLatitude();
    gpsData.longitude = myGNSS.getLongitude();
    gpsData.altitude = myGNSS.getAltitudeMSL();
    gpsData.fixType = myGNSS.getFixType();
    gpsData.satCount = myGNSS.getSIV();

    gpsData.day = myGNSS.getDay();
    gpsData.month = myGNSS.getMonth();
    gpsData.year = myGNSS.getYear();

    trackerCoord.lat = gpsData.latitude;
    trackerCoord.lon = gpsData.longitude;
    trackerCoord.alt = gpsData.altitude;

    uavCoord.lat = getUavGPS()->global_lat;
    uavCoord.lon = getUavGPS()->global_lon;
    uavCoord.alt = getUavGPS()->global_alt;

    if (gpsData.fixType >= 3 && !date_flag)
    {
      mag_declination = declination.magneticDeclination(gpsData.latitude, gpsData.longitude, myGNSS.getYear() % 100, myGNSS.getMonth(), myGNSS.getDay());
      date_flag = true;
    }
  
    // heading = convertHeading(headingKalman(tiltCompensatedHeading(mag_data.x, mag_data.y, mag_data.z, pitch, roll)), -90 + declination.magneticDeclination(gpsData.latitude, gpsData.longitude, myGNSS.getYear() % 100, myGNSS.getMonth(), myGNSS.getDay()));
    if (gpsData.fixType >= 3)
    {
      heading = convertHeading(headingKalman(atan2(mag_data.y, mag_data.x) * 180 / PI), -90 + declination.magneticDeclination(gpsData.latitude, gpsData.longitude, myGNSS.getYear() % 100, myGNSS.getMonth(), myGNSS.getDay())); 
      if(i <= 200){
        i++;
      }
      if (i >= 200 && !ready_to_track)
      {
        moveServoByAngle((int)(abs(heading - 180)) % 360, 1000, 100);
        st.CalibrationOfs(1);
        delay(2000);
        ready_to_track = true;
      }
      if (ready_to_track && getConnectionStatus())
      {
        sDistAziElev = distAziElev(trackerCoord, uavCoord);
        st.WritePosEx(1, angleToServo(sDistAziElev.azimuth), 1000, 100); 
      }
    }
    else
    {
      heading = convertHeading(headingKalman(atan2(mag_data.y, mag_data.x) * 180 / PI), -90 );
      //Serial.println("______________________________");
      // Serial.println(servoToAngle(st.ReadPos(1)));
      // //Serial.println(servoToAngle(servoToAngle(st.ReadPos(1)) + heading));
      // Serial.println((int)(abs(heading - 180)) % 360);
      //Serial.println(heading);
      // Serial.println(i);
      // if(i <= 100){
      //   i++;
      // }
      // if (i >= 100 && !ready_to_track)
      // {
      //   //st.WritePosEx(1, angleToServo(servoToAngle(st.ReadPos(1)) + heading), 1000, 100);
      //   moveServoByAngle((int)(abs(heading - 180)) % 360, 1000, 100);
      //   delay(5000);
      //   st.CalibrationOfs(1);
      //   delay(1000);
      //   ready_to_track = true;
      // }
      // heading = convertHeading(headingKalman(tiltCompensatedHeading(mag_data.x, mag_data.y, mag_data.z, pitch, roll)), -90);
    }
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}