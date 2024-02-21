
#define MODULE_ESP32_DEVKITC_V4

#include <Arduino.h>
#include "mlrs-wireless-bridge-boards.h"
#include <WiFi.h>
#include <TMC2209.h>

#define WIFI_POWER  WIFI_POWER_2dBm // WIFI_POWER_MINUS_1dBm is the lowest possible, WIFI_POWER_19_5dBm is the max

String ssid = "mLRS AP"; // Wifi name
String password = ""; // "thisisgreat"; // WiFi password, "" makes it an open AP
IPAddress ip(192, 168, 4, 55); // connect to this IP // MissionPlanner default is 127.0.0.1, so enter

int port_tcp = 5760; // connect to this port per TCP // MissionPlanner default is 5760
int port_udp = 14550; // connect to this port per UDP // MissionPlanner default is 14550

IPAddress ip_udpcl(192, 168, 0, 164); // connect to this IP // MissionPlanner default is 192.168.0.164
int port_udpcl = 14550; // connect to this port per UDPCL // MissionPlanner default is 14550
int wifi_channel = 6;

int baudrate = 115200;

IPAddress ip_udp(ip[0], ip[1], ip[2], ip[3]+1); // usually the client/MissionPlanner gets assigned +1
IPAddress ip_gateway(0, 0, 0, 0);
IPAddress netmask(255, 255, 255, 0);
WiFiUDP udp;

bool led_state;
unsigned long led_tlast_ms;
bool is_connected;
unsigned long is_connected_tlast_ms;
unsigned long serial_data_received_tfirst_ms;

TaskHandle_t SerialBridge;
TaskHandle_t GeoCalculations;
QueueHandle_t shareMavlinkMsg;

void serialFlushRx(void) {
    while (SERIAL.available() > 0) { uint8_t c = SERIAL.read(); }
}

void bridgeTask(void * parameters) {
  for(;;) {
    unsigned long tnow_ms = millis();

    if (is_connected && (tnow_ms - is_connected_tlast_ms > 2000)) { // nothing from GCS for 2 secs
        is_connected = false;
    }

    if (tnow_ms - led_tlast_ms > (is_connected ? 500 : 200)) {
        led_tlast_ms = tnow_ms;
        led_state = !led_state;
        if (led_state) led_on(is_connected); else led_off();
    }

    //-- here comes the core code, handle WiFi or Bluetooth connection and do the bridge

    uint8_t buf[256]; // working buffer

    int packetSize = udp.parsePacket();
    if (packetSize) {
        int len = udp.read(buf, sizeof(buf));
        SERIAL.write(buf, len);
        is_connected = true;
        is_connected_tlast_ms = millis();
    }

    tnow_ms = millis(); // may not be relevant, but just update it
    int avail = SERIAL.available();
    if (avail <= 0) {
        serial_data_received_tfirst_ms = tnow_ms;
    } 
    else
      if ((tnow_ms - serial_data_received_tfirst_ms) > 10 || avail > 128) { // 10 ms at 57600 bps corresponds to 57 bytes, no chance for 128 bytes
          serial_data_received_tfirst_ms = tnow_ms;

          int len = SERIAL.read(buf, sizeof(buf));
          xQueueSend(shareMavlinkMsg, &buf, 1);
          udp.beginPacket(ip_udp, port_udp);
          udp.write(buf, len);
          udp.endPacket();
      }
  }
}

void GeoClacTask(void * parameters) {

  uint8_t buf[256];
  xQueueReceive(shareMavlinkMsg, &buf, portMAX_DELAY);
}
void setup() {

    led_init();
    dbg_init();
    delay(500);

    shareMavlinkMsg = xQueueCreate(1, sizeof(uint8_t)*256);

    size_t rxbufsize = SERIAL.setRxBufferSize(2*1024); // must come before uart started, retuns 0 if it fails
    size_t txbufsize = SERIAL.setTxBufferSize(512); // must come before uart started, retuns 0 if it fails

    SERIAL.begin(baudrate);

//????used to work    pinMode(U1_RXD, INPUT_PULLUP); // important, at least in older versions Arduino serial lib did not do it

    DBG_PRINTLN(rxbufsize);
    DBG_PRINTLN(txbufsize);


    WiFi.mode(WIFI_AP); // seems not to be needed, done by WiFi.softAP()?
    WiFi.softAPConfig(ip, ip_gateway, netmask);
  
    String ssid_full = ssid + " UDP";

    WiFi.softAP(ssid_full.c_str(), (password.length()) ? password.c_str() : NULL, wifi_channel); // channel = 1 is default
    DBG_PRINT("ap ip address: ");
    DBG_PRINTLN(WiFi.softAPIP()); // comes out as 192.168.4.1
    DBG_PRINT("channel: ");
    DBG_PRINTLN(WiFi.channel());
  
    WiFi.setTxPower(WIFI_POWER); // set WiFi power, AP or STA must have been started, returns false if it fails
    udp.begin(port_udp);

    led_tlast_ms = 0;
    led_state = false;
    is_connected = false;
    is_connected_tlast_ms = 0;
    serial_data_received_tfirst_ms = 0;

    serialFlushRx();

    xTaskCreatePinnedToCore(bridgeTask, "Bridge", 5000, NULL, 2, &SerialBridge, ARDUINO_RUNNING_CORE);
    xTaskCreatePinnedToCore(GeoClacTask, "Calculations", 5000, NULL, 2, &GeoCalculations, ARDUINO_RUNNING_CORE);
}
void loop() {

}
