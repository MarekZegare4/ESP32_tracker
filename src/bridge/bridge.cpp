#include <WiFi.h>
#include <BluetoothSerial.h>
#include <Arduino.h>
#include "bridge.h"

#define WIFI_POWER  WIFI_POWER_2dBm
static QueueHandle_t queue;

// WiFi settings
String ssid = "mLRS AP"; // Wifi name
String password = ""; // "thisisgreat"; // WiFi password, "" makes it an open AP

IPAddress ip(192, 168, 4, 55); // connect to this IP // MissionPlanner default is 127.0.0.1, so enter
IPAddress ip_udp(ip[0], ip[1], ip[2], ip[3]+1); // usually the client/MissionPlanner gets assigned +1
IPAddress ip_gateway(0, 0, 0, 0);
IPAddress netmask(255, 255, 255, 0);

WiFiUDP udp;

int port_tcp = 5760; // connect to this port per TCP // MissionPlanner default is 5760
int port_udp = 14550; // connect to this port per UDP // MissionPlanner default is 14550
int wifi_channel = 6;

// Bluetooth settings
BluetoothSerial SerialBT;
String device_name = "ESP32-BT"; // Bluetooth device name

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
  if (xQueueReceive(queue, &packet, portMAX_DELAY) == pdTRUE) {
    return packet;
  }
  else {
    packet.len = 0;
    return packet;
  }
}

void serialFlushRx(void)
{
  while (Serial2.available() > 0) { Serial2.read(); }
}

void WiFiBridgeInitialize() {
  WiFi.mode(WIFI_AP); // seems not to be needed, done by WiFi.softAP()?
  WiFi.softAPConfig(ip, ip_gateway, netmask);
  String ssid_full = ssid + " UDP";
  WiFi.softAP(ssid_full.c_str(), (password.length()) ? password.c_str() : NULL, wifi_channel); // channel = 1 is default
  WiFi.setTxPower(WIFI_POWER);
  udp.begin(port_udp);

  is_connected = false;
  is_connected_tlast_ms = 0;
  serial_data_received_tfirst_ms = 0;
}

void BtBridgeInitialize(){
  BluetoothSerial SerialBT;
  Serial.begin(115200);
  SerialBT.begin(device_name); //Bluetooth device name
  Serial.printf("The device with name \"%s\" is started.\nNow you can pair it with Bluetooth!\n", device_name.c_str());
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
      xQueueSend(queue, &packet, 0);
      udp.beginPacket(ip_udp, port_udp);
      udp.write(buf, len);
      udp.endPacket();
    }
  }
}

void BtBridgeTask(void * parameters) {
  for(;;){
   // uint8_t buf[256]; // working buffer
    if (Serial.available()) {
      SerialBT.write(Serial.read());
    }
    if (SerialBT.available()) {
      Serial.write(SerialBT.read());
    }
    vTaskDelay(20 / portTICK_PERIOD_MS);
  }
}