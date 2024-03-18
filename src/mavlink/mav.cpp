#include "common/mavlink.h"
#include <Arduino.h>
#include "bridge/bridge.h" // packet struct

extern QueueHandle_t kolejka;
// Pomoce: https://discuss.ardupilot.org/t/mavlink-and-arduino-step-by-step/25566

void SendHeartbeatTask(void * parameters) {
    for(;;) {
        uint8_t system_id = 1;
        uint8_t component_id = MAV_COMP_ID_PERIPHERAL;
        mavlink_message_t msg;
        uint8_t buf[MAVLINK_MAX_PACKET_LEN];
        mavlink_msg_heartbeat_pack(system_id, component_id, &msg, MAV_TYPE_GCS, MAV_AUTOPILOT_INVALID, MAV_MODE_PREFLIGHT, 0, MAV_STATE_ACTIVE);
        uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
        if (Serial2.availableForWrite()) {
            Serial2.write(buf, len);
            Serial.print("Wyslano HB\n");
            vTaskDelay(1000 / portTICK_PERIOD_MS);
        }
    }
}

void DecodeTelemetryTask(void * parameters){
    mavlink_status_t status;
    mavlink_message_t msg;
    int chan = MAVLINK_COMM_0;
    packet packet;
   
    for(;;){
        if(xQueuePeek(kolejka, &packet, 10/portTICK_PERIOD_MS)) {
            xQueueReceive(kolejka, &packet, 0);
            for (uint16_t i = 0; i < packet.len; i++) {
                uint8_t byte = packet.buf[i];
                if (mavlink_parse_char(chan, byte, &msg, &status)) {
                    printf("Received message with ID %d, sequence: %d from component %d of system %d\n", msg.msgid, msg.seq, msg.compid, msg.sysid);
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
                            Serial.print("Alt ");
                            Serial.print(global_position.alt);
                            Serial.print('\n');
                            }
                            break;
                        case MAVLINK_MSG_ID_GPS_INPUT: // ID for GPS_INPUT
                            {
                            // Get just one field from payload
                            mavlink_gps_input_t gps_input;
                            Serial.print("Visible sats ");
                            Serial.print(gps_input.satellites_visible);
                            Serial.print('\n');
                            }
                            break;
                        case MAVLINK_MSG_ID_ATTITUDE: // ID for ATTITUDE
                            {
                                // Get all fields in payload (into attitude)
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