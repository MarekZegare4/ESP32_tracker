#include <Arduino.h>
#include "mag.h"
#include "servo.h"
#include "display.h"
#include "bridge.h"
#include "mav.h"

#define mlrsRX  33
#define mlrsTX  32

#define S_RXD 18
#define S_TXD 19

void JedenTask (void * parameters) {
  for(;;) {
    printf("Działa\n");
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

void setup() {
  size_t rxbufsize = Serial2.setRxBufferSize(2*1024); // must come before uart started, retuns 0 if it fails
  size_t txbufsize = Serial2.setTxBufferSize(512); // must come before uart started, retuns 0 if it fails
  Serial1.setRxBufferSize(1024);
  Serial2.begin(57600, SERIAL_8N1, mlrsRX, mlrsTX);
  //Serial1.begin(57600, SERIAL_8N1, 27, 26);
  Serial1.begin(1000000, SERIAL_8N1, S_RXD, S_TXD);
  Serial.begin(57600);

  WiFiInitialize();
  DisplayInitialize();

  kolejka = xQueueCreate(2, sizeof(packet));
  xTaskCreate(JedenTask, "Test", 1000, NULL, 1, NULL);
  xTaskCreatePinnedToCore(ScreenTask, "Ekran", 1000, NULL, 1, NULL, 1);
  //xTaskCreate(HeartbeatTask, "Heartbeat", 5000, NULL, 1, NULL);
  xTaskCreatePinnedToCore(MavTask, "Mav", 5000, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(BridgeTask, "Bridge", 5000, NULL, 1, NULL, ARDUINO_RUNNING_CORE);
}


void loop() {
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