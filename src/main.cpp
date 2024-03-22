#include <Arduino.h>
#include "magnetometer/mag.h"
#include "servo/servo.h"
#include "display/display.h"
#include "bridge/bridge.h"
#include "mavlink/mav.h"
#include "gps/gps.h"

void setup() {
  Serial.begin(57600);
  
  CreateQueue();
  ServoInitialize();
  MavlinkInitialize();
  WiFiBridgeInitialize();
  DisplayInitialize();
  MagInitialize();

  xTaskCreatePinnedToCore(WiFiBridgeTask, "Bridge", 5000, NULL, 1, NULL, 0);
  xTaskCreate(MagTask, "Magnetometer", 2000, NULL, 5, NULL);
  xTaskCreatePinnedToCore(DisplayTask, "Display", 2000, NULL, 2, NULL, 1);
  xTaskCreate(SendHeartbeatTask, "Heartbeat", 2000, NULL, 3, NULL);
  xTaskCreate(DecodeTelemetryTask, "Telemetry decoding", 5000, NULL, 2, NULL);
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
  vTaskPrioritySet(NULL, 10);
  vTaskSuspend(NULL);
}