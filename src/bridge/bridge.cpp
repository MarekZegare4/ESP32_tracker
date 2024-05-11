#include <WiFi.h>
#include <Arduino.h>
#include "bridge.h"
#include "mavlink/mav.h"
#include "BluetoothSerial.h"

#define WIFI_POWER  WIFI_POWER_2dBm

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
int wifi_channel = 6; // WiFi channel, 1-13

BluetoothSerial btSerial;
const char *pin = "1234";

void wifiBridgeInitialize() {
  serialFlushRx();
  WiFi.mode(WIFI_AP); // seems not to be needed, done by WiFi.softAP()?
  WiFi.softAPConfig(ip, ip_gateway, netmask);
  WiFi.softAP(ssid.c_str(), (password.length()) ? password.c_str() : NULL, wifi_channel); // channel = 1 is default
  WiFi.setTxPower(WIFI_POWER);
  udp.begin(port_udp);
}

void bluetoothBridgeInitialize() {
  btSerial.begin("Tracker");
  btSerial.setPin(pin);
}

void wifiBridgeTask(void * parameters) {
  Packet packet;
  for(;;){
    uint8_t buf[256]; // working buffer
    int packetSize = udp.parsePacket();
    if (packetSize) {
      int len = udp.read(buf, sizeof(buf));
      if (Serial2.availableForWrite()) {
        Serial2.write(buf, len);
      }
    }
    packet = accessQueue();
    udp.beginPacket(ip_udp, port_udp);
    udp.write(packet.buf, packet.len);
    udp.endPacket();
    vTaskDelay(50 / portTICK_PERIOD_MS);
  }
}

void bluetoothBridgeTask(void * parameters) {
  Packet packet;
  for(;;){
    uint8_t buf[256];
    if (Serial2.availableForWrite()) {
      int len = btSerial.available();
      if(len > sizeof(buf)) {
        len = sizeof(buf);
      }
      for (int i = 0; i < len; i++) {
        buf[i] = btSerial.read();
      }
      Serial2.write(buf, len);
    }
    packet = accessQueue();
    btSerial.write(packet.buf, packet.len);
    vTaskDelay(50 / portTICK_PERIOD_MS);
  }
}