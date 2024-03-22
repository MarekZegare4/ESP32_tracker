#include <Arduino.h>
#include "magnetometer/mag.h"
#include "servo/servo.h"
#include "display/display.h"
#include "bridge/bridge.h"
#include "mavlink/mav.h"
#include "gps/gps.h"

void setup() {
  Serial.begin(57600);

  ServoInitialize();
  CreateQueue();
  MavlinkInitialize();
  WiFiBridgeInitialize();
  DisplayInitialize();
  MagInitialize();

  xTaskCreate(WiFiBridgeTask, "Bridge", 5000, NULL, 5, NULL);
  xTaskCreate(MagTask, "Magnetometer", 2000, NULL, 2, NULL);
  xTaskCreate(DisplayTask, "Display", 5000, NULL, 4, NULL);
  xTaskCreate(SendHeartbeatTask, "Heartbeat", 2000, NULL, 3, NULL);
  xTaskCreate(DecodeTelemetryTask, "Telemetry decoding", 5000, NULL, 4, NULL);
}

void loop() {
  // int ID = sms_sts.Ping(TEST_ID);
  // if(ID!=-1){
  //   Serial.print("Servo ID:");
  //   Serial.println(ID, DEC);
  //   delay(100);
  // }else{
  //   Serial.println("Ping servo ID error!");
  delay(2000);
  // }
}