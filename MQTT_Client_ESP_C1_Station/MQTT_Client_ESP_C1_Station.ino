#include <WiFiClientSecure.h>
#include <MQTT.h>
#include "LDR.h"
#include "arduino_secrets.h"
#include "Output.h"

#define NumberOfLDRs 4

const char* ssid     = SECRET_SSID;
const char* password = SECRET_PASS;
const uint16_t port = 1883;
const char * clientName = "ESPInclineC1Station";

const char * server = "192.168.1.29";


WiFiClient wifiClient;
MQTTClient client;
Output relays[8];
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
  client.subscribe("track/turnout/6001");
  client.subscribe("track/turnout/6002");
  client.subscribe("track/turnout/6003");
  client.subscribe("track/turnout/6004");
  client.subscribe("track/turnout/6005");
  client.subscribe("track/turnout/6006");
  client.subscribe("track/turnout/6007");
  client.subscribe("track/turnout/6008");
  client.subscribe("track/turnout/6009");
  client.subscribe("track/turnout/6010");
  client.subscribe("track/turnout/6011"); // frog only
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
  String strTopic = (String)topic;
  int pos = strTopic.lastIndexOf("/");
  bool usingDegrees = false;
  if (pos >= 0 && strTopic.indexOf("turnout") >= 0) {
    String justTheID = strTopic.substring(pos + 1);
    int iID = justTheID.toInt();

    Serial.println("Looking in relays for " + justTheID);
    int foundRelayIndex = -1;
    for (int r = 0; r < 6; r++) {
      if (relays[r].JMRIId == justTheID) {
        foundRelayIndex = r;
        break;
      }
    }

    if (foundRelayIndex >= 0) {
      Serial.println("Found matching local relay on pin " + String(relays[foundRelayIndex].pin));
      int rVal = 0;
      if (payload == "THROWN") rVal = 1;
      if (relays[foundRelayIndex].isInverted) rVal = !rVal;
      Serial.println("Set to write " + String(rVal) + " to pin " + String(relays[foundRelayIndex].pin));
      digitalWrite(relays[foundRelayIndex].pin, rVal);
    }

  }
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
    String debugMessage = String(LDRs[i].analogVal) + " - " + LDRs[i].State;
    client.publish("debug/" + LDRs[i].JMRIId, debugMessage, false, 0);
  }
}

void InitialiseConfig() {
  LDRs[0] = LDR("LDR Station Approach XO", 34, "6010", 2800, 100, 2000);
  LDRs[1] = LDR("LDR Station AC Entry", 35, "6011", 1800, 100, 2000);
  LDRs[2] = LDR("LDR Station XO", 39, "6012", 1800, 100, 2000);
  LDRs[3] = LDR("LDR Station Bay", 32, "6013", 3000, 100, 500);


  relays[0] = Output("6001", 23, true);
  relays[1] = Output("6002", 22, true);
  relays[2] = Output("6003", 21, true);
  relays[3] = Output("6004", 19, false);
  relays[4] = Output("6005", 18, true);
  relays[5] = Output("6006", 5, false);
  relays[6] = Output("6007", 17, false);
  relays[7] = Output("6008", 16, false);
  relays[8] = Output("6009", 4, false);
  relays[9] = Output("6010", 0, true);
  relays[10] = Output("6011", 2, true);
  //relays[11] = Output("xx6012", 15, true);

  for (int i = 0; i < 8; i++) {
    pinMode(relays[i].pin, OUTPUT);
    Serial.println("Setup complete " + String(i));
  }
}
