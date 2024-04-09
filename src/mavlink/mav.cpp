#include "common/mavlink.h"
#include <Arduino.h>
#include "gui/gui.h"
#include "mavlink/mav.h"
#include "bridge/bridge.h"

// Pomoce: https://discuss.ardupilot.org/t/mavlink-and-arduino-step-by-step/25566

#define LRS_RX  33
#define LRS_TX  32
#define MLRS_BAUD 57600

static QueueHandle_t sQueue;
static UavDataGPS sUavDataGPS;
static UavDataAttitude sUavDataAttitude;
static bool isConnected = false;

bool bridgeActive = true;

bool getConnectionStatus() {
    return isConnected;
}

UavDataGPS getsUavGPS() {
    return sUavDataGPS;
}

UavDataAttitude getUavAttitude() {
    return sUavDataAttitude;
}

void mavlinkInitialize() {
    Serial2.begin(MLRS_BAUD, SERIAL_8N1, LRS_RX, LRS_TX);
}

void sendHeartbeatTask(void * parameters) {
    for(;;) {
        uint8_t system_id = 1;
        uint8_t component_id = MAV_COMP_ID_UDP_BRIDGE;
        mavlink_message_t msg;
        uint8_t buf[MAVLINK_MAX_PACKET_LEN];
        mavlink_msg_heartbeat_pack(system_id, component_id, &msg, MAV_TYPE_GCS, MAV_AUTOPILOT_INVALID, MAV_MODE_PREFLIGHT, 0, MAV_STATE_ACTIVE);
        uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
        if (Serial2.availableForWrite()) {
            Serial2.write(buf, len);
            //Serial.print("Wyslano HB\n");
            vTaskDelay(1000 / portTICK_PERIOD_MS);
        }
    }
}

void createQueue() {
    sQueue = xQueueCreate(1, sizeof(Packet));
}

Packet accessQueue() {
  Packet packet;
  if (xQueueReceive(sQueue, &packet, portMAX_DELAY)) {
    return packet;
  }
}

bool packetAvailable() {
    return xQueuePeek(sQueue, NULL, 0);
}

void serialFlushRx(void)
{
  while (Serial2.available() > 0) { Serial2.read(); }
}

void decodeTelemetryTask(void * parameters){
    mavlink_status_t status;
    mavlink_message_t msg;
    int chan = MAVLINK_COMM_0;
    for(;;){
        uint8_t buf[256]; // working buffer
        if (Serial2.available()) {
            int len = Serial2.read(buf, sizeof(buf));
            if (bridgeActive){
                Packet packet;
                packet.len = len;
                std::copy(buf, buf+len, packet.buf);
                xQueueSend(sQueue, &packet, portMAX_DELAY);
            }
            for (uint16_t i = 0; i < len; i++) {
                uint8_t byte = buf[i];
                if (mavlink_parse_char(chan, byte, &msg, &status)) {
                    //Serial.printf("Received message with ID %d, sequence: %d from component %d of system %d\n", msg.msgid, msg.seq, msg.compid, msg.sysid);
                    switch(msg.msgid) {
                        case MAVLINK_MSG_ID_HEARTBEAT: // ID for HEARTBEAT
                            {
                                // Get all fields in payload (into heartbeat)
                                mavlink_heartbeat_t heartbeat;
                                mavlink_msg_heartbeat_decode(&msg, &heartbeat);
                                break;
                            }
                        case MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN: // ID for SYS_STATUS
                            {
                                // Get all fields in payload (into sys_status)
                                mavlink_statustext_t sys_status;
                                mavlink_msg_statustext_decode(&msg, &sys_status);
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