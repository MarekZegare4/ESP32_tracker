#include <Arduino.h>
#include "tracking/tracking.h"
#include "gui/gui.h"
#include "mavlink/mav.h"

void setup()
{
  Serial.begin(115200);
  trackingInitialize();   // Servo and IMU initialization
  mavlinkInit();          // MAVLink comm initialization
  guiInitialize();        // GUI initialization
  xTaskCreate(trackingTask, "Movement and stabilization", 4096, NULL, 1, NULL);
  xTaskCreate(guiTask, "GUI", 8192, NULL, 1, NULL);
  xTaskCreate(decodeTelemetryTask, "Telemetry decoding", 4096, NULL, 1, NULL);
}

void loop()
{
  vTaskDelete(NULL);
}
