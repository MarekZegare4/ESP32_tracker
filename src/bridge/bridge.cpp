#include <WiFi.h>
#include <Arduino.h>
#include "bridge.h"

#define WIFI_POWER  WIFI_POWER_2dBm
static QueueHandle_t queue;

// WiFi settings
String ssid = "mLRS UDP"; // Wifi name
String password = ""; // "thisisgreat"; // WiFi password, "" makes it an open AP

IPAddress ip(192, 168, 4, 55); // connect to this IP // MissionPlanner default is 127.0.0.1, so enter
IPAddress ip_udp(ip[0], ip[1], ip[2], ip[3]+1); // usually the client/MissionPlanner gets assigned +1
IPAddress ip_gateway(0, 0, 0, 0);
IPAddress netmask(255, 255, 255, 0);

WiFiUDP udp;

int port_tcp = 5760; // connect to this port per TCP // MissionPlanner default is 5760
int port_udp = 14550; // connect to this port per UDP // MissionPlanner default is 14550
int wifi_channel = 6;

// Connection timing
unsigned long led_tlast_ms;
bool is_connected;
unsigned long is_connected_tlast_ms;
unsigned long serial_data_received_tfirst_ms;

void CreateQueue() {
    queue = xQueueCreate(2, sizeof(packet));
}

packet AccessQueue() {
  packet packet;
  if (xQueueReceive(queue, &packet, portMAX_DELAY)) {
    return packet;
  }
}

void serialFlushRx(void)
{
  while (Serial2.available() > 0) { Serial2.read(); }
}

void WiFiBridgeInitialize() { // Do wyczyszczenia, niepotrzebnie mierzony czas
  serialFlushRx();
  WiFi.mode(WIFI_AP); // seems not to be needed, done by WiFi.softAP()?
  WiFi.softAPConfig(ip, ip_gateway, netmask);
  WiFi.softAP(ssid.c_str(), (password.length()) ? password.c_str() : NULL, wifi_channel); // channel = 1 is default
  WiFi.setTxPower(WIFI_POWER);
  udp.begin(port_udp);

  is_connected = false;
  is_connected_tlast_ms = 0;
  serial_data_received_tfirst_ms = 0;
}

void WiFiBridgeTask(void * parameters) {
  packet packet;
  for(;;){
    
    unsigned long tnow_ms = millis();
    if (is_connected && (tnow_ms - is_connected_tlast_ms > 2000)) { // nothing from GCS for 2 secs
    is_connected = false;
    }

    if (tnow_ms - led_tlast_ms > (is_connected ? 500 : 200)) {
      led_tlast_ms = tnow_ms;
    }
    uint8_t buf[256]; // working buffer
    int packetSize = udp.parsePacket();
    if (packetSize) {
      int len = udp.read(buf, sizeof(buf));
      Serial2.write(buf, len);
      is_connected = true;
      is_connected_tlast_ms = millis();
    }

    tnow_ms = millis(); // may not be relevant, but just update it
    int avail = Serial2.available();
    if (avail <= 0) {
      serial_data_received_tfirst_ms = tnow_ms;
    } else
    if ((tnow_ms - serial_data_received_tfirst_ms) > 10 || avail > 128) { // 10 ms at 57600 bps corresponds to 57 bytes, no chance for 128 bytes
      serial_data_received_tfirst_ms = tnow_ms;
      
      int len = Serial2.read(buf, sizeof(buf));
      packet.len = len;
      for (uint8_t i = 0; i < len; i++){
        packet.buf[i] = buf[i];
      }
      udp.beginPacket(ip_udp, port_udp);
      udp.write(buf, len);
      udp.endPacket();
      xQueueSend(queue, &packet, 10);
    }
    vTaskDelay(10);
  }
}