#include <Arduino.h>
#include "common/mavlink.h"
#include <Adafruit_GFX.h>
#include <Adafruit_SharpMem.h>

#define mlrsRX  33
#define mlrsTX  32

#define SHARP_SCK  14
#define SHARP_MOSI 13
#define SHARP_SS   12

#define BLACK 0
#define WHITE 1

Adafruit_SharpMem display(SHARP_SCK, SHARP_MOSI, SHARP_SS, 400, 240);

void JedenTask (void * parameters)
{
  for(;;)
  {
    //printf("dziala\n");
    Serial1.write("test\n");
    display.fillCircle(display.width()/2, display.height()/2, 10, BLACK);
    display.refresh();
    delay(500);
    display.clearDisplay();
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

void setup() {
  Serial1.setRxBufferSize(1024);
  Serial2.begin(57600, SERIAL_8N1, mlrsRX, mlrsTX);
  Serial1.begin(57600, SERIAL_8N1, 27, 26);
  Serial.begin(57600);

  display.begin();
  display.clearDisplay();

  // draw a circle, 10 pixel radius

  xTaskCreate(JedenTask, "task", 5000, NULL, 1, NULL);
}

// void loop() {
//     uint8_t msg;
//     while (mlrsSerial.available() > 0){
//        msg = mlrsSerial.read();
//     }
//     Serial.write(msg);
    

// }

// void loop(){
//   while (Serial2.available()) {
//     Serial.print(char(Serial2.read()));
//   }
// }


mavlink_status_t status;
uint8_t stat;
mavlink_message_t msg;
int chan = MAVLINK_COMM_0;



int visible_sats;

void loop(){

  uint8_t sysid = 1;
  uint8_t compid = MAV_COMP_ID_PERIPHERAL;
  //Serial.print(mavlink_msg_heartbeat_encode(sysid, compid, &msg, MAV_TYPE_GENERIC, MAV_AUTOPILOT_INVALID, MAV_MODE_FLAG_SAFETY_ARMED, 0, stat));
  

  uint16_t available = Serial2.available();
  for (uint16_t i = 0; i < available; i++){
    uint8_t byte = Serial2.read();
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
          
          visible_sats = mavlink_msg_gps_input_get_fix_type(&msg);
          Serial.print("Visible sats ");
          Serial.print(visible_sats);
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