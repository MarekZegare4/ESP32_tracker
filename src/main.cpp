
#include <WiFi.h>                        // include libraries
#include "AsyncUDP.h"
#include "esp_wifi.h"

const char* ssid     = "ESP32_AP";
const char* password = "123456789";          // set password, NULL; sets no password

IPAddress local_ip(192,168,0,1);
IPAddress gateway(192,168,0,1);
IPAddress subnet(255,255,255,0);


void JedenTask (void * parameters)
{
  for(;;)
  {
    printf("dziala\n");
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

void setup()
{
  WiFi.mode(WIFI_AP);
  WiFi.setTxPower(WIFI_POWER_8_5dBm);
  WiFi.softAPConfig(local_ip, gateway, subnet);
  WiFi.softAP(ssid, password);
  delay(2000);
  Serial.print("[+] AP Created with IP Gateway ");
  Serial.println(WiFi.softAPIP());
  Serial.begin(115200);

  xTaskCreate(JedenTask, "task", 5000, NULL, 1, NULL);
}

void loop()
{
  wifi_sta_list_t wifi_sta_list;
  tcpip_adapter_sta_list_t adapter_sta_list;
 
  memset(&wifi_sta_list, 0, sizeof(wifi_sta_list));
  memset(&adapter_sta_list, 0, sizeof(adapter_sta_list));
 
  esp_wifi_ap_get_sta_list(&wifi_sta_list);
  tcpip_adapter_get_sta_list(&wifi_sta_list, &adapter_sta_list);
 
  for (int i = 0; i < adapter_sta_list.num; i++) {
 
    tcpip_adapter_sta_info_t station = adapter_sta_list.sta[i];

    Serial.print("station nr ");
    Serial.println(i);
 
    Serial.print("MAC: ");
 
    for(int i = 0; i< 6; i++){
      
      Serial.printf("%02X", station.mac[i]);  
      if(i<5)Serial.print(":");
    }
    char ip[IP4ADDR_STRLEN_MAX];
    esp_ip4addr_ntoa(&station.ip, ip, IP4ADDR_STRLEN_MAX);
    Serial.print("\nIP: ");  
    Serial.print(ip);
  }
 
  Serial.println("-----------");
  delay(5000);
}