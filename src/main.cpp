#include <Arduino.h>
#include "tracking/tracking.h"
#include "display/display.h"
#include "bridge/bridge.h"
#include "mavlink/mav.h"
#include "gps/gps.h"

void setup() {
  Serial.begin(115200);
  CreateQueue();
  TrackingInitialize();
  MavlinkInitialize();
  WiFiBridgeInitialize();
  DisplayInitialize();
 // xTaskCreatePinnedToCore(WiFiBridgeTask, "Bridge", 5000, NULL, 1, NULL, 0);
  xTaskCreate(TrackingTask, "Servo + Magnetometer", 2000, NULL, 1, NULL);
  xTaskCreate(DisplayTask, "Display", 5000, NULL, 1, NULL);
  xTaskCreate(SendHeartbeatTask, "Heartbeat", 2000, NULL, 1, NULL);
  xTaskCreate(DecodeTelemetryTask, "Telemetry decoding", 5000, NULL, 1, NULL);
}

void loop() {
  vTaskDelete(NULL);
}