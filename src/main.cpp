#include <Arduino.h>
#include "tracking/tracking.h"
#include "gui/gui.h"
#include "mavlink/mav.h"

void setup()
{
  Serial.begin(115200);
  trackingInitialize();   // Inicjalizacja serwomechanizmu i magnetometru
  mavlinkInit();    // Inicjalizacja komunikacji MAVLink
  guiInitialize();    // Inicjalizacja wyświetlacza
  xTaskCreate(trackingTask, "Movement and stabilization", 2048, NULL, 1, NULL);
  xTaskCreate(guiTask, "GUI", 4096, NULL, 1, NULL);
  xTaskCreate(decodeTelemetryTask, "Telemetry decoding", 4096, NULL, 1, NULL);
}

void loop()
{
  vTaskDelete(NULL);
}
