#pragma once
#include "common/mavlink.h"
#include <Arduino.h>
#include "bridge.h"

void HeartbeatTask(void * parameters) {
    for(;;) {
        uint8_t system_id = 1;
        uint8_t component_id = MAV_COMP_ID_PERIPHERAL;
        mavlink_message_t msg;
        mavlink_heartbeat_t heartbeat;
        if (Serial2.availableForWrite()) {
            Serial2.write(mavlink_msg_heartbeat_encode(system_id, component_id, &msg, &heartbeat));
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
         vTaskDelay(10/portTICK_PERIOD_MS); // 10ms delay
        if(xQueuePeek(kolejka, &packet, 0)) {
            xQueueReceive(kolejka, &packet, 0);
            for (uint16_t i = 0; i < packet.len; i++) {
                uint8_t byte = packet.buf[i];
                if (mavlink_parse_char(chan, byte, &msg, &status)) {
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