#include <Arduino.h>
#include "magnetometer/mag.h"
#include "servo/servo.h"
#include "display/display.h"
#include "bridge/bridge.h"
#include "mavlink/mav.h"
#include "gps/gps.h"

displayElements dispElem;

void setup() {
  Serial.begin(57600);

  ServoInitialize();
  CreateQueue();
  MavlinkInitialize();
  BridgeInitialize();
  DisplayInitialize();
  MagInitialize();

  xTaskCreatePinnedToCore(BridgeTask, "Bridge", 5000, NULL, 1, NULL, 0);
  xTaskCreate(MagTask, "Magnetometer", 2000, NULL, 1, NULL);
  xTaskCreatePinnedToCore(DisplayTask, "Display", 2000, NULL, 1, NULL, 1);
  //xTaskCreate(DegTask, "Deg", 1000, NULL, 1, NULL);
  xTaskCreate(SendHeartbeatTask, "Heartbeat", 2000, NULL, 1, NULL);
  xTaskCreate(DecodeTelemetryTask, "Telemetry decoding", 5000, NULL, 1, NULL);
  
}


void loop() {
  // int ID = sms_sts.Ping(TEST_ID);
  // if(ID!=-1){
  //   Serial.print("Servo ID:");
  //   Serial.println(ID, DEC);
  //   delay(100);
  // }else{
  //   Serial.println("Ping servo ID error!");
  //   delay(2000);
  // }
}