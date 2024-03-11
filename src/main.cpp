#include <Arduino.h>
#include "common/mavlink.h"
#include <Adafruit_GFX.h>
#include <Adafruit_SharpMem.h>
#include <WiFi.h>
#include <SCServo.h>
#include <DFRobot_BMM150.h>
#include <SPI.h>
#include "ekran.h"

#define mlrsRX  33
#define mlrsTX  32

// #define SHARP_SCK  14
// #define SHARP_MOSI 13
// #define SHARP_SS   12

// #define BLACK 0
// #define WHITE 1

#define WIFI_POWER  WIFI_POWER_2dBm

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

unsigned long led_tlast_ms;
bool is_connected;
unsigned long is_connected_tlast_ms;
unsigned long serial_data_received_tfirst_ms;

//Adafruit_SharpMem display(SHARP_SCK, SHARP_MOSI, SHARP_SS, 400, 240);

SMS_STS sms_sts;
#define S_RXD 18
#define S_TXD 19

int TEST_ID = 3;

//uint8_t buf[256]; // working buffer
//int len;
static QueueHandle_t kolejka;

class packet {
  public:
    int len;
    uint8_t buf[256];
};

void serialFlushRx(void)
{
    while (Serial2.available() > 0) { Serial2.read(); }
}

void JedenTask (void * parameters) {
  for(;;)
  {
    //printf("dziala\n");
    Serial1.write("test\n");
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

void ScreenTask (void * parameters) {
  int FPS = 60;
  szcz();
  for(;;) {
    
    //vTaskDelay(1/FPS*portTICK_PERIOD_MS);
  }
}

void HeartbeatTask(void * parameters) {
  for(;;) {
    uint8_t system_id = 1;
    uint8_t component_id = MAV_COMP_ID_PERIPHERAL;
    mavlink_message_t msg;
    mavlink_heartbeat_t heartbeat;
    if (Serial2.availableForWrite()){
      Serial2.write(mavlink_msg_heartbeat_encode(system_id, component_id, &msg, &heartbeat));
      Serial.print("Wyslano HB\n");
      vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
  }
}

void BridgeTask(void * parameters) {
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
        struct{
          int len;
          uint8_t buf[256];
        } packet;

        packet.len = len;
        for (uint8_t i = 0; i < len; i++){
          packet.buf[i] = buf[i];
        }
        xQueueSend(kolejka, &packet, 0);
        udp.beginPacket(ip_udp, port_udp);
        udp.write(buf, len);
        udp.endPacket();
    }
  }
}

void MavTask(void * parameters){
  mavlink_status_t status;
  mavlink_message_t msg;
  int chan = MAVLINK_COMM_0;
  packet packet;
for(;;){
  
  if(xQueuePeek(kolejka, &packet, 0)){
    xQueueReceive(kolejka, &packet, 0);
  for (uint16_t i = 0; i < packet.len; i++){
    uint8_t byte = packet.buf[i];
    if (mavlink_parse_char(chan, byte, &msg, &status))
    {
    printf("Received message with ID %d, sequence: %d from component %d of system %d\n", msg.msgid, msg.seq, msg.compid, msg.sysid);
    switch(msg.msgid) {
      case MAVLINK_MSG_ID_GLOBAL_POSITION_INT: // ID for GLOBAL_POSITION_INT
        {
          // Get all fields in payload (into global_position)
          mavlink_global_position_int_t global_position;
          mavlink_msg_global_position_int_decode(&msg, &global_position);
          Serial.print("Alt ");
          Serial.print(global_position.alt);
          Serial.print('\n');
        }
        break;
      case MAVLINK_MSG_ID_GPS_INPUT:
        {
          // Get just one field from payload
          mavlink_gps_input_t gps_input;
          Serial.print("Visible sats ");
          Serial.print(gps_input.satellites_visible);
          Serial.print('\n');
        }
        break;
      case MAVLINK_MSG_ID_ATTITUDE:
      {
        Serial.print("Jest ATTITUDE\n");
        mavlink_attitude_t attitude;
        mavlink_msg_attitude_decode(&msg, &attitude);
      }
      break;
     default:
        break;
    }
    }
  }
  }
}
}

void setup() {
  size_t rxbufsize = Serial2.setRxBufferSize(2*1024); // must come before uart started, retuns 0 if it fails
  size_t txbufsize = Serial2.setTxBufferSize(512); // must come before uart started, retuns 0 if it fails
  Serial1.setRxBufferSize(1024);
  Serial2.begin(57600, SERIAL_8N1, mlrsRX, mlrsTX);
  //Serial1.begin(57600, SERIAL_8N1, 27, 26);
  Serial1.begin(1000000, SERIAL_8N1, S_RXD, S_TXD);
  sms_sts.pSerial = &Serial1;
  Serial.begin(57600);

  WiFi.mode(WIFI_AP); // seems not to be needed, done by WiFi.softAP()?
  WiFi.softAPConfig(ip, ip_gateway, netmask);
  String ssid_full = ssid + " UDP";
  WiFi.softAP(ssid_full.c_str(), (password.length()) ? password.c_str() : NULL, wifi_channel); // channel = 1 is default
  WiFi.setTxPower(WIFI_POWER);
  udp.begin(port_udp);

  is_connected = false;
  is_connected_tlast_ms = 0;
  serial_data_received_tfirst_ms = 0;

  display.begin();
  display.clearDisplay();

  // draw a circle, 10 pixel radius
  kolejka = xQueueCreate(2, sizeof(packet));
  //xTaskCreate(JedenTask, "task", 5000, NULL, 1, NULL);
  xTaskCreate(ScreenTask, "Ekran", 1000, NULL, 1, NULL);
  //xTaskCreate(HeartbeatTask, "Heartbeat", 5000, NULL, 1, NULL);
  xTaskCreate(MavTask, "Mav", 5000, NULL, 1, NULL);
  xTaskCreatePinnedToCore(BridgeTask, "Bridge", 5000, NULL, 1, NULL, ARDUINO_RUNNING_CORE);
}



void loop()
{
  // int ID = sms_sts.Ping(TEST_ID);
  // if(ID!=-1){
  //   Serial.print("Servo ID:");
  //   Serial.println(ID, DEC);
  //   delay(100);
  // }else{
  //   Serial.println("Ping servo ID error!");
  //   delay(2000);
  // }
}