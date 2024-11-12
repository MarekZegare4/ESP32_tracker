#include <Arduino.h>
#include <BluetoothSerial.h>
#include <WiFi.h>
#include "common/mavlink.h"
#include "gui/gui.h"
#include "mavlink/mav.h"
#include "tracking/tracking.h"

// Some info: https://discuss.ardupilot.org/t/mavlink-and-arduino-step-by-step/25566
// Bridge part courtesy of @olliw42 https://github.com/olliw42/mLRS/tree/main/esp

#define LRS_RX 33
#define LRS_TX 32
#define MLRS_BAUD 57600
#define WIFI_POWER WIFI_POWER_2dBm

// WiFi settings
String ssid = "mLRS UDP"; // Wifi name
String password = "";     

IPAddress ip(192, 168, 4, 55);                    
IPAddress ip_udp(ip[0], ip[1], ip[2], ip[3] + 1); 
IPAddress ip_gateway(0, 0, 0, 0);
IPAddress netmask(255, 255, 255, 0);

int port_tcp = 5760;  // connect to this port per TCP // MissionPlanner default is 5760
int port_udp = 14550; // connect to this port per UDP // MissionPlanner default is 14550
int wifi_channel = 6; // WiFi channel, 1-13

BluetoothSerial SerialBT;
WiFiUDP udp;

QueueHandle_t sQueue;
UavDataGPS sUavDataGPS;
UavDataAttitude sUavDataAttitude;
UavSysText sUavSysText;

eBridgeType bridgeType = BLUETOOTH;

bool bt_bridge_flag = false;
bool udp_bridge_flag = false;

bool isConnected = false;
bool bridgeActive = true;

bool getConnectionStatus()
{
    return isConnected;
}

UavDataGPS *getUavGPS()
{
    return &sUavDataGPS;
}

UavDataAttitude *getUavAttitude()
{
    return &sUavDataAttitude;
}

UavSysText *getUavSysText()
{
    return &sUavSysText;
}
void bluetoothInit()
{
    while (!SerialBT.begin("BT Tracker bridge"))
    {
        delay(1000);
    }
}

void bluetoothDeinit()
{
    SerialBT.end();
}

void udpDeinit()
{
    udp.stop();
}

void udpInit()
{
    WiFi.softAPConfig(ip, ip_gateway, netmask);
    WiFi.softAP(ssid.c_str(), (password.length()) ? password.c_str() : NULL, wifi_channel); // channel = 1 is default
    WiFi.setTxPower(WIFI_POWER);
    udp.begin(port_udp);
}

void setBridgeType(eBridgeType type)
{
    bridgeType = type;
}

/**
 * @brief Initialize communication with MAVLink receiver
 */
void mavlinkInit()
{
    Serial2.begin(MLRS_BAUD, SERIAL_8N1, LRS_RX, LRS_TX);

    switch (bridgeType)
    {
    case BLUETOOTH:
        bluetoothInit();
        bt_bridge_flag = true;
        break;

    case UDP:
        udpInit();
        udp_bridge_flag = true;
        break;
    }
}

/**
 * @brief Send MAVLink Heartbeat message
 */
void sendHeartbeat()
{
    // Heartbeat
    uint8_t system_id = 200;
    uint8_t component_id = MAV_COMP_ID_MISSIONPLANNER;
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    mavlink_msg_heartbeat_pack(system_id, component_id, &msg, MAV_TYPE_GCS, MAV_AUTOPILOT_INVALID, MAV_MODE_PREFLIGHT, 0, MAV_STATE_ACTIVE);
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    SerialBT.write(buf, len);
}

/**
 * @brief Send MAVLink Global Position message
 */
void sendGlobalPosition()
{
    // Global position
    uint8_t system_id = 10;
    uint8_t component_id = MAV_COMP_ID_PERIPHERAL;
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

    uint32_t time_boot_ms = 100;
    TrackerDataGPS tracker_gps = getTrackerGPS();
    int32_t lat = tracker_gps.latitude;
    int32_t lon = tracker_gps.longitude;
    int32_t alt = tracker_gps.altitude;
    int32_t realtive_alt = tracker_gps.altitude;
    int16_t vx = 0;
    int16_t vy = 0;
    int16_t vz = 0;
    uint16_t hdg = UINT16_MAX;
    mavlink_msg_global_position_int_pack(system_id, component_id, &msg, time_boot_ms, lat, lon, alt, realtive_alt, vx, vy, vz, hdg);
    len = mavlink_msg_to_send_buffer(buf, &msg);
    SerialBT.write(buf, len);
}

/**
 * @brief Task for decoding telemetry and sending Mavlink messages to UAV
 * @param parameters
 */
void decodeTelemetryTask(void *parameters)
{
    mavlink_status_t status;
    mavlink_message_t msg;
    int chan = MAVLINK_COMM_0;
    uint64_t HBPeriod = 1e6; // in microseconds
    uint64_t currentTime = 0;
    uint64_t lastHBTime = 0;
    uint64_t connectionTimeout = 3e6; // in microseconds
    uint64_t currentConnectionTime = 0;
    uint64_t lastConnectionTime = 0;

    for (;;)
    {
        // Check connection and send heartbeat 
        currentTime = esp_timer_get_time();

        if (currentTime - lastConnectionTime > connectionTimeout)
        {
            isConnected = false;
        }
        else
        {
            isConnected = true;
        }
        if (currentTime - lastHBTime >= HBPeriod)
        {
            if (Serial2.availableForWrite())
            {
                sendHeartbeat();
                lastHBTime = currentTime;
            }
        }

        uint8_t buf[256]; // working buffer
        if (Serial2.available())
        {
            int len = Serial2.read(buf, sizeof(buf));

            // Bridge part
            // --------------------------------
            switch (bridgeType)
            {
            case BLUETOOTH:
            {
                if (!bt_bridge_flag)
                {
                    udpDeinit();
                    udp_bridge_flag = false;
                    bluetoothInit();
                    bt_bridge_flag = true;
                }
                uint8_t buf2[256];
                if (Serial2.availableForWrite())
                {
                    int len2 = SerialBT.available();
                    if (len2 > sizeof(buf2))
                    {
                        len2 = sizeof(buf2);
                    }
                    for (int i = 0; i < len2; i++)
                    {
                        buf2[i] = SerialBT.read();
                    }
                    Serial2.write(buf2, len2);
                }
                SerialBT.write(buf, len);
                break;
            }

            case UDP:
            {
                if (!udp_bridge_flag)
                {
                    bluetoothDeinit();
                    bt_bridge_flag = false;
                    udpInit();
                    udp_bridge_flag = true;
                }
                int packetSize = udp.parsePacket();
                uint8_t buf2[256];
                if (packetSize)
                {
                    int len2 = udp.read(buf2, sizeof(buf2));
                    if (Serial2.availableForWrite())
                    {
                        Serial2.write(buf2, len2);
                    }
                }
                udp.beginPacket(ip_udp, port_udp);
                udp.write(buf, len);
                udp.endPacket();
                break;
            }

            case USB:
            {
                if (bt_bridge_flag)
                {
                    bluetoothDeinit();
                    bt_bridge_flag = false;
                }
                if (udp_bridge_flag)
                {
                    udpDeinit();
                    udp_bridge_flag = false;
                }
                uint8_t buf2[256];
                if (Serial.available())
                {
                    int len2 = Serial.read(buf2, sizeof(buf2));
                    Serial2.write(buf2, len2);
                }
                Serial.write(buf, len);
                break;
            }
            default:
                break;
            }
            // Message decoding
            // --------------------------------
            for (uint16_t i = 0; i < len; i++)
            {
                uint8_t byte = buf[i];
                if (mavlink_parse_char(chan, byte, &msg, &status))
                {
                    // Serial.printf("Received message with ID %d, sequence: %d from component %d of system %d\n", msg.msgid, msg.seq, msg.compid, msg.sysid);

                    switch (msg.msgid)
                    {
                    case MAVLINK_MSG_ID_HEARTBEAT: // ID for HEARTBEAT
                    {
                        // Get all fields in payload (into heartbeat)
                        lastConnectionTime = esp_timer_get_time();
                        mavlink_heartbeat_t heartbeat;
                        mavlink_msg_heartbeat_decode(&msg, &heartbeat);
                        break;
                    }
                    case MAVLINK_MSG_ID_STATUSTEXT: // ID for SYS_STATUS
                    {
                        // Get all fields in payload (into sys_status)
                        mavlink_statustext_t sys_status;
                        mavlink_msg_statustext_decode(&msg, &sys_status);
                        // Serial.printf("Severity: %d, text: %s\n", sys_status.severity, sys_status.text);
                        sUavSysText.severity = sys_status.severity;
                        strcpy(sUavSysText.text, sys_status.text);
                        //Serial.println(sUavSysText.text);
                        break;
                    }
                    case MAVLINK_MSG_ID_GLOBAL_POSITION_INT: // ID for GLOBAL_POSITION_INT
                    {
                        // Get all fields in payload (into global_position)
                        mavlink_global_position_int_t global_position;
                        mavlink_msg_global_position_int_decode(&msg, &global_position);
                        sUavDataGPS.global_lat = global_position.lat;
                        sUavDataGPS.global_lon = global_position.lon;
                        sUavDataGPS.global_alt = global_position.alt;
                    }
                    break;
                    case MAVLINK_MSG_ID_GPS_INPUT: // ID for GPS_INPUT
                    {
                        // Get just one field from payload
                        mavlink_gps_input_t gps_input;
                    }
                    break;
                    case MAVLINK_MSG_ID_ATTITUDE: // ID for ATTITUDE
                    {
                        // Get all fields in payload (into attitude)
                        mavlink_attitude_t attitude;
                        mavlink_msg_attitude_decode(&msg, &attitude);
                        sUavDataAttitude.pitch = attitude.pitch;
                        sUavDataAttitude.roll = attitude.roll;
                        sUavDataAttitude.yaw = attitude.yaw;
                    }
                    break;
                    default:
                        break;
                    }
                }
            }
        }
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}