#include <Arduino.h>
#include "tracking/tracking.h"
#include "gui/gui.h"
#include "bridge/bridge.h"
#include "mavlink/mav.h"
#include "gps/gps.h"

void setup() {
  Serial.begin(115200);
  CreateQueue();          // Tworzenie kolejki dla pakietów
  TrackingInitialize();   // Inicjalizacja serwomechanizmu i magnetometru
  MavlinkInitialize();    // Inicjalizacja komunikacji MAVLink
  WiFiBridgeInitialize(); // Inicjalizacja mostu WiFi
  DisplayInitialize();    // Inicjalizacja wyświetlacza
  xTaskCreatePinnedToCore(TrackingTask, "Servo + Magnetometer", 2000, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(DisplayTask, "Display", 5000, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(SendHeartbeatTask, "Heartbeat", 2000, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(DecodeTelemetryTask, "Telemetry decoding", 5000, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(WiFiBridgeTask, "Bridge", 5000, NULL, 1, NULL, 0);
}

void loop() {
  vTaskDelete(NULL);
}