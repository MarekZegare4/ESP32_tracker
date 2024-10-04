#include "common/mavlink.h"
#include <Arduino.h>
#include "gui/gui.h"
#include "mavlink/mav.h"
#include "bridge/bridge.h"
#include "gps/gps.h"
#include "bridge/bridge.h"

// Pomoce: https://discuss.ardupilot.org/t/mavlink-and-arduino-step-by-step/25566

#define LRS_RX 33
#define LRS_TX 32
#define MLRS_BAUD 57600

static QueueHandle_t sQueue;
static UavDataGPS sUavDataGPS;
static UavDataAttitude sUavDataAttitude;
static UavSysText sUavSysText;
static bool isConnected = false;

bool bridgeActive = true;

bool getConnectionStatus()
{
    return isConnected;
}

UavDataGPS getUavGPS()
{
    return sUavDataGPS;
}

UavDataAttitude getUavAttitude()
{
    return sUavDataAttitude;
}

UavSysText getUavSysText()
{
    return sUavSysText;
}

/**
 * @brief Initialize communication with MAVLink receiver
 */
void mavlinkInitialize()
{
    Serial2.begin(MLRS_BAUD, SERIAL_8N1, LRS_RX, LRS_TX);
}

// /**
//  * @brief Task for sending MAVLink messages
//  * @param parameters
//  */
// void sendMavlinkMsgTask(void *parameters)
// {
//     for (;;)
//     {
        
//         vTaskDelay(1000 / portTICK_PERIOD_MS);
//     }
// }

/**
 * @brief Send MAVLink Heartbeat message
 */
void sendHeartbeat()
{
    // Heartbeat
    uint8_t system_id = 10;
    uint8_t component_id = MAV_COMP_ID_PERIPHERAL;
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    mavlink_msg_heartbeat_pack(system_id, component_id, &msg, MAV_TYPE_GCS, MAV_AUTOPILOT_INVALID, MAV_MODE_PREFLIGHT, 0, MAV_STATE_ACTIVE);
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    sendBTMsg(buf, len);
}

/**
 * @brief Send MAVLink Global Position message
 */
void sendGlobalPosition() {
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
    sendBTMsg(buf, len);
}

/**
 * @brief Create queue for packets
 */
void createQueue()
{
    sQueue = xQueueCreate(1, sizeof(Packet));
}

/**
 * @brief Access the queue
 * @return data packet from queue
 */
Packet accessQueue()
{
    Packet packet;
    if (xQueueReceive(sQueue, &packet, portMAX_DELAY))
    {
        return packet;
    }
}

/**
 * @brief Check if packet is available
 * @return true if packet is available
 */
bool packetAvailable()
{
    return xQueuePeek(sQueue, NULL, 0);
}

/**
 * @brief Flush serial buffer
 */
void serialFlushRx(void)
{
    while (Serial2.available() > 0)
    {
        Serial2.read();
    }
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
    uint16_t HBPeriod = 1000;
    uint16_t currentHBTime = 0;
    uint16_t lastHBTime = 0;
    for (;;)
    {
        // Send heartbeat
        currentHBTime = millis();
        if (currentHBTime - lastHBTime >= HBPeriod)
        {
            sendHeartbeat();
            lastHBTime = currentHBTime;
        }

        uint8_t buf[256]; // working buffer
        if (Serial2.available())
        {
            int len = Serial2.read(buf, sizeof(buf));
            if (bridgeActive)
            {
                Packet packet;
                packet.len = len;
                std::copy(buf, buf + len, packet.buf);
                xQueueSend(sQueue, &packet, portMAX_DELAY);
            }
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
                        Serial.println(sUavSysText.text);
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
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
}