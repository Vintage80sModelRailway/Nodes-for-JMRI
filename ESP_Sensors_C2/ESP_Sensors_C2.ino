#include <WiFiClientSecure.h>
#include <MQTT.h>
#include "Output.h"
#include "arduino_secrets.h"

const char* ssid     = SECRET_SSID;
const char* password = SECRET_PASS;
const uint16_t port = 1883;
const char * clientName = "ESPSensorsC2";

const char * server = "192.168.1.29";


WiFiClient wifiClient;
MQTTClient client;
Output relays[6];

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
  client.subscribe("debug/PWMVal/#");

  client.subscribe("track/turnout/5003");
  client.subscribe("track/turnout/5004");
  client.subscribe("track/turnout/5005");
  client.subscribe("track/turnout/5006");
  client.subscribe("track/turnout/5007");
  client.subscribe("track/turnout/5008");
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



  // Note: Local domain names (e.g. "Computer.local" on OSX) are not supported
  // by Arduino. You need to set the IP address directly.
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

    if (foundRelayIndex >=0) {
      Serial.println("Found matching local relay on pin "+String(relays[foundRelayIndex].pin));
      int rVal = 0;
      if (payload == "THROWN") rVal = 1;
      if (relays[foundRelayIndex].isInverted) rVal = !rVal;
      Serial.println("Set to write "+String(rVal)+" to pin "+String(relays[foundRelayIndex].pin));
      digitalWrite(relays[foundRelayIndex].pin,rVal);
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

void PublishToMQTT(String topic, String message)  {
  char topicBuffer[topic.length() + 1];
  char messageBuffer[message.length() + 1];

  topic.toCharArray(topicBuffer, topic.length() + 1);
  message.toCharArray(messageBuffer, message.length() + 1);

  Serial.println("Publish: " + topic + " - " + message);
  //mqttClient.publish(topicBuffer, (uint8_t*)messageBuffer, message.length(), true);
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
  LDRs[0] = LDR("Incline Top DS PC end", 36, "5020", 1800, 300, 2);
  LDRs[1] = LDR("Incline Top DS Pi end", 32, "5021", 1800, 300, 2);
  
  relays[0] = Output("5006",23,false);
  relays[1] = Output("5004",22,false);
  relays[2] = Output("5005",21,true);
  relays[3] = Output("5003",19,true);
  relays[4] = Output("5007",18,true);
  relays[5] = Output("5008",5,false);
  //relays[6] = Output("xx",16,false);
  //relays[7] = Output("xx",17,true);

  for (int i = 0; i < 6; i++) {
    pinMode(relays[i].pin, OUTPUT);
    Serial.println("Setup complete "+String(i));
  }

  
}
