#include <WiFiClientSecure.h>
#include <MQTT.h>
#include "LDR.h"
#include "arduino_secrets.h"

#define NumberOfLDRs 3

const char* ssid     = SECRET_SSID;
const char* password = SECRET_PASS;
const uint16_t port = 1883;
const char * clientName = "ESPInclineC1Station";

const char * server = "192.168.1.29";


WiFiClient wifiClient;
MQTTClient client;
bool inSetup = true;

LDR LDRs[NumberOfLDRs];

void connect() {
  Serial.print("checking wifi...");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(1000);
  }

  Serial.print("connecting...");
  while (!client.connect(clientName)) {
    Serial.print(".");
    delay(1000);
  }

  Serial.println("\nconnected!");
}

void setup()
{
  Serial.begin(115200);
  delay(10);

  WiFi.begin(ssid, password);

  int tryDelay = 500;
  int numberOfTries = 20;

    InitialiseConfig();

  // Wait for the WiFi event
  bool isConnected = false;
  while (!isConnected) {

    switch (WiFi.status()) {
      case WL_NO_SSID_AVAIL:
        Serial.println("[WiFi] SSID not found");
        break;
      case WL_CONNECT_FAILED:
        Serial.print("[WiFi] Failed - WiFi not connected! Reason: ");
        return;
        break;
      case WL_CONNECTION_LOST:
        Serial.println("[WiFi] Connection was lost");
        break;
      case WL_SCAN_COMPLETED:
        Serial.println("[WiFi] Scan is completed");
        break;
      case WL_DISCONNECTED:
        Serial.println("[WiFi] WiFi is disconnected");
        break;
      case WL_CONNECTED:
        Serial.println("[WiFi] WiFi is connected!");
        Serial.print("[WiFi] IP address: ");
        Serial.println(WiFi.localIP());
        isConnected = true;
        break;
      default:
        Serial.print("[WiFi] WiFi Status: ");
        Serial.println(WiFi.status());
        break;
    }
    delay(tryDelay);

    if (numberOfTries <= 0) {
      Serial.print("[WiFi] Failed to connect to WiFi!");
      // Use disconnect function to force stop trying to connect
      WiFi.disconnect();
      return;
    } else {
      numberOfTries--;
    }
  }

  client.begin(server, wifiClient);
  client.onMessage(messageReceived);

  connect();
  inSetup = false;
}

void messageReceived(String &topic, String &payload) {
  Serial.println("incoming: " + topic + " - " + payload);
}

void loop()
{
  client.loop();
  if (!client.connected()) {
    connect();
  }
  for (int i = 0; i < NumberOfLDRs; i++) {
    UpdateLDR(i);    
  }
}

void UpdateLDR(int i) {
  bool hasChanged = LDRs[i].UpdateSensor();
//  int debugVal = analogRead(LDRs[i].pin);
//  if (i==0) Serial.println(String(debugVal)+ " - "+String(i));
  if (hasChanged) {
    Serial.println("Has changed " + String(LDRs[i].pin) + " " + LDRs[i].State);
    String publishMessage = LDRs[i].State;
    String topic = LDRs[i].GetSensorPublishTopic();
    
    client.publish(topic, publishMessage, true, 0);
    String debugMessage = String(LDRs[i].analogVal)+" - "+LDRs[i].State;
    client.publish("debug/"+LDRs[i].JMRIId,debugMessage,false,0);
  }
}

void InitialiseConfig() {
  LDRs[0] = LDR("LDR Station Approach XO", 34, "6010", 2800, 100, 2);
  LDRs[1] = LDR("LDR Station AC Entry", 35, "6011", 1800, 100, 2);
  LDRs[2] = LDR("LDR Station XO", 39, "6012", 1800, 100, 2);  
}
