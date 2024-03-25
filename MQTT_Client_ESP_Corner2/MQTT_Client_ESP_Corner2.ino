#include <WiFiClientSecure.h>
#include <MQTT.h>
#include "Sensor.h"
#include "Turnout.h"
#include "ESP32Servo.h"
#include "arduino_secrets.h"

#define NumberOfSensors 9
#define NumberOfTurnouts 3

const char* ssid     = SECRET_SSID;
const char* password = SECRET_PASS;
const uint16_t port = 1883;
const char * clientName = "ESPCorner2Client";

const char * server = "192.168.1.29";


WiFiClient wifiClient;
MQTTClient client;

bool inSetup = true;

Sensor Sensors[NumberOfSensors];
Turnout Turnouts[NumberOfTurnouts];

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
  client.subscribe("track/turnout/1005");
  client.subscribe("track/turnout/1017");
  client.subscribe("track/turnout/1018");

  //client.subscribe("track/turnout/1009");
//  client.subscribe("track/turnout/1010");
//  client.subscribe("track/turnout/1011");
//  client.subscribe("track/turnout/1012");
//  client.subscribe("track/turnout/1013");
//  client.subscribe("track/turnout/1014");
//  client.subscribe("track/turnout/1015");
//  client.subscribe("track/turnout/1016");
//  client.subscribe("track/turnout/1017");

  client.subscribe("debug/PWMVal/#");
}

void setup()
{
  Serial.begin(115200);
  delay(10);

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  WiFi.begin(ssid, password);

  int tryDelay = 500;
  int numberOfTries = 20;

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

  InitialiseConfig();

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

    //Find turnout with this ID
    for (int i = 0; i < NumberOfTurnouts; i++) {
      if (Turnouts[i].jMRIId == justTheID) {
        Serial.println(justTheID + " - needs to go " + payload);
        Turnouts[i].requiredState = payload;
        if (payload == "THROWN") {
          if (inSetup) {
            Turnouts[i].currentPWMVal = Turnouts[i].thrownVal - 1;
            //Serial.println("Setup - init PWM value of " + String(Turnouts[i].currentPWMVal) + " for " + Turnouts[i].jMRIId);
          }
          Turnouts[i].requiredPWMVal = Turnouts[i].thrownVal;
        }
        else {
          if (inSetup) {
            Turnouts[i].currentPWMVal = Turnouts[i].closedVal + 1;
            //Serial.println("Setup - init PWM value of " + String(Turnouts[i].currentPWMVal) + " for " + Turnouts[i].jMRIId);
          }
          Turnouts[i].requiredPWMVal = Turnouts[i].closedVal;
        }
        Serial.println("Required PWM " + String(Turnouts[i].requiredPWMVal));
        Turnouts[i].Start();
      }
    }
  }
  else if (pos >= 0 && strTopic.indexOf("PWMVal") >= 0) {
    String justTheID = strTopic.substring(pos + 1);
    int iID = justTheID.toInt();
    if (payload.indexOf('D') > 0) {
      usingDegrees = true;
    }

    Serial.println("Using degrees " + String(usingDegrees));

    //Find turnout with this ID
    for (int i = 0; i < NumberOfTurnouts; i++) {
      if (Turnouts[i].jMRIId == justTheID) {
        Serial.println(justTheID + " - PWM debug value " + payload);
        int PWM = payload.toInt();
        Serial.println("Debug PWM val = " + String(PWM));

        Turnouts[i].requiredPWMVal = PWM;
        if (payload.indexOf('C') > 0) {
          Turnouts[i].requiredState = "CLOSED";
          Serial.println("Debug - assuming closed");
        } else if (payload.indexOf('T') > 0) {
          Turnouts[i].requiredState = "THROWN";
          Serial.println("Debug - assuming thrown");
        }
        else {
          Serial.println("Debug - danger - unable to assert turnout state, inconsistent performance possible");
        }
        Serial.println("Required PWM " + String(Turnouts[i].requiredPWMVal));
        Turnouts[i].Start();
      }
    }
  }
}

void loop()
{
  client.loop();
  if (!client.connected()) {
    connect();
  }

  for (int i = 0; i < NumberOfTurnouts; i++) {
    Turnouts[i].CheckState();
  }

//  for (int i = 0; i < NumberOfSensors; i++) {
//    UpdateSensor(i);
//  }
}

void PublishToMQTT(String topic, String message)  {
  char topicBuffer[topic.length() + 1];
  char messageBuffer[message.length() + 1];

  topic.toCharArray(topicBuffer, topic.length() + 1);
  message.toCharArray(messageBuffer, message.length() + 1);

  Serial.println("Publish: " + topic + " - " + message);
  //mqttClient.publish(topicBuffer, (uint8_t*)messageBuffer, message.length(), true);
}

void UpdateSensor(int i) {
  bool hasChanged = Sensors[i].UpdateSensor();
  if (hasChanged) {
    Serial.println("Has changed " + String(Sensors[i].pin) + " " + Sensors[i].State);
    String publishMessage = Sensors[i].State;
    String topic = Sensors[i].GetSensorPublishTopic();

    client.publish(topic, publishMessage, true, 0);

  }
}

void InitialiseConfig() {
  //Sensors - Name, Pin, JMRIId, IsInverted = false, Pinmode = INPUT, Lastknownvalue = 0
//  Sensors[0] = Sensor("Curved AC", 12, "3001", false, INPUT_PULLUP);
//  Sensors[1] = Sensor("Curved CW", 13, "3002", false, INPUT_PULLUP);
//  Sensors[2] = Sensor("", 27, "3003", false, INPUT_PULLUP);
//  Sensors[3] = Sensor("", 14, "3004", true, INPUT_PULLUP);
//
//  Sensors[4] = Sensor("", 26, "3005", false, INPUT_PULLUP);
//  Sensors[5] = Sensor("", 22, "3006", true, INPUT_PULLUP);
//  Sensors[6] = Sensor("", 32, "3007", true, INPUT_PULLUP);
//  Sensors[7] = Sensor("", 25, "3008", false, INPUT_PULLUP);
//  Sensors[8] = Sensor("", 33, "3012", true, INPUT_PULLUP);

  Turnouts[0] = Turnout(1, "1017", 23, 1700, 900, false, 1, 5, true); // JMRIID, ESP32 pin, thrown, closed, slow step, slow delay, useDetach
  Turnouts[1] = Turnout(2, "1018", 22, 1000, 1800, false, 1, 5, false);
  Turnouts[2] = Turnout(3, "1005", 21, 1800, 1200, false,  1, 5, true);
//  Turnouts[3] = Turnout(4, "3004", 4, 55, 110, true,  1, 30, false);
//  Turnouts[4] = Turnout(5, "3005", 17, 140, 70, true,  1, 30, true);
//  Turnouts[5] = Turnout(6, "3006", 5, 155, 85, true,  1, 30, false);
//  Turnouts[6] = Turnout(7, "3007", 18, 70, 140, true, 1, 30, false);
//  Turnouts[7] = Turnout(8, "3008", 19, 100, 40, true, 1, 30, false);
//  Turnouts[8] = Turnout(9, "1006", 21, 60, 120, true, 1, 30, true);

//  for (int i = 0; i < NumberOfSensors; i++) {
//    Sensors[i].SetPinMode();
//    UpdateSensor(i);
//
//    String publishMessage = Sensors[i].State;
//    String topic = Sensors[i].GetSensorPublishTopic();
//  }

  delay(5000); //allow initial PWM settings to work themselves out - assuming persistence on MQTT server

  Serial.println("Setup complete");
}
