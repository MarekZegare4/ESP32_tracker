#include <Arduino.h>
#include "tracking/tracking.h"
#include "gui/gui.h"
#include "bridge/bridge.h"
#include "mavlink/mav.h"
#include "gps/gps.h"

void setup()
{
  Serial.begin(115200);
  createQueue();          // Tworzenie kolejki dla pakietów
  trackingInitialize();   // Inicjalizacja serwomechanizmu i magnetometru
  mavlinkInitialize();    // Inicjalizacja komunikacji MAVLink
  //wifiBridgeInitialize(); // Inicjalizacja mostu WiFi
  bluetoothBridgeInitialize(); // Inicjalizacja mostu Bluetooth
  guiInitialize();    // Inicjalizacja wyświetlacza
  gpsInitialize();        // Inicjalizacja GNSS
  xTaskCreatePinnedToCore(trackingTask, "Movement and stabilization", 2048, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(guiTask, "GUI", 4096, NULL, 1, NULL, 1);
  //xTaskCreatePinnedToCore(sendMavlinkMsgTask, "Heartbeat", 2000, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(decodeTelemetryTask, "Telemetry decoding", 4096, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(bluetoothBridgeTask, "Bridge", 4096, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(gpsTask, "GPS", 2048, NULL, 1, NULL, 1);
}

void loop()
{
  vTaskDelete(NULL);
}

// Do poprawki
// usunąć/zmienić kondesator BOOT https://www.esp32.com/viewtopic.php?t=22895