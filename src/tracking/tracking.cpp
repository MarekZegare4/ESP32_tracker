#include <Arduino.h>
#include "tracking.h"
#include "geoTransform/geoTransform.h"
#include <SCServo.h>
#include <DFRobot_BMM150.h>
#include <SPI.h>
#include <ESP32Servo.h>
#include "gui/gui.h"
#include "mavlink/mav.h"

// https://www.waveshare.com/wiki/ST3020_Servo#Overview

#define SERVO_RX 16
#define SERVO_TX 17
#define SERVO_TILT_PIN 4

Servo servo;
SMS_STS st;
DFRobot_BMM150_I2C bmm150(&Wire, I2C_ADDRESS_4);

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
    servo.setPeriodHertz(50); // Standard 50hz servo
    servo.attach(SERVO_TILT_PIN, 1000, 2000); // attaches the servo on pin 4 to the servo object
    servo.write(0);
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

void trackingTask(void *parameters)
{
for (;;)
    {
        //Serial.println(st.Ping(1));
        // servo.write(90);
        // st.WritePosEx(1, 2048, 1500, 50);
        // delay(1000);
        // servo.write(45);
        // delay(1000);
        // servo.write(0);
        // st.WritePosEx(1, 4096, 1500, 50);
        // delay(1000);
        //st.WritePosEx(1, 0, 4000, 50);
        //delay(5000);
        //st.WritePosEx(1, 2048, 1500, 50);
        //delay(5000);
        //st.WritePosEx(1, 4096, 1500, 50);
        //delay(5000);
        //Wgs84Coord uavPos;
        //sDistAziElev = DistAziElev(uavPos, uavPos);
        //sCompassDegree = bmm150.getCompassDegree();
        //st.WritePosEx(1, angleToServo(sCompassDegree), 1500, 50); // To control the servo with ID 1, rotate it to position 1000 at a speed of 1500, with a start-stop acceleration of 50.
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}