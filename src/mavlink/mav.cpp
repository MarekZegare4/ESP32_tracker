#include "common/mavlink.h"
#include <Arduino.h>
#include "display/display.h"
#include "mavlink/mav.h"
#include "bridge/bridge.h"

// Pomoce: https://discuss.ardupilot.org/t/mavlink-and-arduino-step-by-step/25566

#define LRS_RX  33
#define LRS_TX  32

extern displayElements dispElem;
static QueueHandle_t queue;

int32_t uavLon;
int32_t uavLat;
int32_t uavAlt;

bool bridge_active = false;

void MavlinkInitialize() {
    //size_t rxbufsize = Serial2.setRxBufferSize(2*1024); // must come before uart started, retuns 0 if it fails
    //size_t txbufsize = Serial2.setTxBufferSize(512); // must come before uart started, retuns 0 if it fails
    Serial2.begin(57600, SERIAL_8N1, LRS_RX, LRS_TX);
}

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
            //Serial.print("Wyslano HB\n");
            vTaskDelay(1000 / portTICK_PERIOD_MS);
        }
    }
}

void CreateQueue() {
    queue = xQueueCreate(1, sizeof(packet));
}

packet AccessQueue() {
  packet packet;
  if (xQueueReceive(queue, &packet, portMAX_DELAY)) {
    return packet;
  }
}

bool PacketAvailable() {
    return xQueuePeek(queue, NULL, 0);
}

void serialFlushRx(void)
{
  while (Serial2.available() > 0) { Serial2.read(); }
}

void DecodeTelemetryTask(void * parameters){
    mavlink_status_t status;
    mavlink_message_t msg;
    int chan = MAVLINK_COMM_0;
    dispElem.isConnected = false;
    for(;;){
        uint8_t buf[256]; // working buffer
        if (Serial2.available()) {
            int len = Serial2.read(buf, sizeof(buf));
            if (bridge_active){
                xTaskCreatePinnedToCore(WiFiBridgeTask, "Bridge", 5000, NULL, 1, NULL, 0);
                packet packet;
                packet.len = len;
                for (uint8_t i = 0; i < len; i++){
                    packet.buf[i] = buf[i];
                }
                xQueueSend(queue, &packet, portMAX_DELAY);
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
                            uavLat = global_position.lat;
                            uavLon = global_position.lon;
                            uavAlt = global_position.alt;
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
                                dispElem.attitudeRoll = attitude.roll;
                                //SendAttitude(attitude);
                            }
                            break;
                        default:
                            break;
                    }
                }
            }
        }
        vTaskDelay(10);
    }  
}