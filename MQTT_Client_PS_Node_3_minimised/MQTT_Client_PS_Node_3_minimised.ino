#include <SPI.h>
#include <Ethernet.h>
#include <PubSubClient.h>
#include "Sensor.h"
#include "RFIDOutput.h"

#define NumberOfSensors 16
#define NumberOfRFIDReaders 2

// Update these with values suitable for your network.
byte mac[6] = { 0x90, 0xA2, 0xDA, 0x90, 0x68, 0xC5 };
IPAddress server(192, 168, 1, 29);

EthernetClient ethClient;
PubSubClient client(ethClient);

Sensor Sensors[NumberOfSensors];
RFIDOutput TagReaders[NumberOfRFIDReaders] = {
  RFIDOutput(Serial1, "acstationexit"),
  RFIDOutput(Serial2, "cwyardexit")
};

void callback(char* topic, byte* payload, unsigned int length) {
  String message = "";
  for (int i = 0; i < length; i++) {
    message = message + (char)payload[i];
  }
  Serial.println("New message " + String(topic) + " - " + message);
}

void setup()
{
  Serial.begin(19200);
  Serial1.begin(9600);
  Serial2.begin(9600);

  client.setServer(server, 1883);
  client.setCallback(callback);
  Ethernet.init(53);
  //Ethernet.begin(mac, ip, gateway, subnet);
  Ethernet.begin(mac);
  // Allow the hardware to sort itself out
  delay(1500);

  //Serial,print("-----Startup ");
  Serial.println(Ethernet.localIP());

  if (!client.connected()) {
    reconnect();
  }

  InitialiseConfig();
}

void loop()
{

  if (!client.connected()) {
    reconnect();
  }

  for (int i = 0; i < NumberOfSensors; i++) {
    UpdateSensor(i);
  }

  for (int r = 0; r < NumberOfRFIDReaders; r++) {
    bool newTag = TagReaders[r].CheckForTag();
    if (newTag) {
      Serial.println("Found " + TagReaders[r].Name);
      Serial.println(TagReaders[r].Tag);
      String topic = "track/reporter/" + TagReaders[r].Name;
      PublishToMQTT(topic, String(TagReaders[r].Tag), false);
    }
  }

  //ProcessShiftIn();
  client.loop();
}

void PublishToMQTT(String topic, String message, bool retain)  {
  byte topicBuffer[topic.length() + 1];
  byte messageBuffer[message.length() + 1];

  topic.toCharArray(topicBuffer, topic.length() + 1);
  message.toCharArray(messageBuffer, message.length() + 1);

  Serial.println("Publish: " + topic + " - " + message);
  client.publish(topicBuffer, messageBuffer, message.length(), retain);
}

void UpdateSensor(int i) {
  bool hasChanged = Sensors[i].UpdateSensor();
  //Serial.println("Checking sensor "+String(i));
  if (hasChanged) {
    Serial.println("Checking sensor " + String(i) + " has changed");
    String publishMessage = Sensors[i].State;
    String topic = Sensors[i].GetSensorPublishTopic();
    if (topic.length() > 0) PublishToMQTT(topic, publishMessage, true);
  }
}

void reconnect() {
  // Loop until we're reconnected

  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("arduinoNode3")) {
      Serial.println("connected");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void InitialiseConfig() {

  //Sensors - Name, Pin, JMRIId, IsInverted = false, Pinmode = INPUT, Lastknownvalue = 0
  Sensors[0] = Sensor("IR AC Yard Entry", 33, "3016", true, INPUT, 1000, 0);
  Sensors[1] = Sensor("zzCD CW Yard Bypass no longer used", 23, "3017", false, INPUT_PULLUP, 300, 2);
  //Sensors[13] = Sensor("zzCD AC Yard Bypass no longer used", 24, "3018", false,INPUT_PULLUP);
  Sensors[2] = Sensor("CD PC End AC", 45, "3019", true, INPUT_PULLUP, 2000, 0, 0);
  Sensors[3] = Sensor("CD PC End CW", 41, "3020", true, INPUT_PULLUP, 2000, 0); //debounce to off only
  //Sensors[4] = Sensor("CD PC end incline", 34, "3021", true, INPUT_PULLUP, 1000, 0);
  Sensors[4] = Sensor("", -1, "");
  Sensors[5] = Sensor("IR CW Yard Exit", 31, "3022", true, INPUT, 300, 0);
  Sensors[6] = Sensor("CD AC Yard bypass PC End", 29, "3023", true, INPUT_PULLUP);
  Sensors[7] = Sensor("CD CW Yard bypass PC End", 30, "3024", true, INPUT_PULLUP);
  Sensors[8] = Sensor("CD UD-UL PC End", A15, "3027", true, INPUT_PULLUP);
  Sensors[9] = Sensor("CD UD-CW PC End", A13, "3028", true, INPUT_PULLUP);
  Sensors[10] = Sensor("CD UD-AC PC End", A12, "3029", true, INPUT_PULLUP);
  Sensors[11] = Sensor("CD Incline PC End DTC", A14, "3030", true, INPUT_PULLUP);
  Sensors[12] = Sensor("CD UD-UL Yard PC End", A11, "3031", true, INPUT_PULLUP);
  Sensors[13] = Sensor("CD UD-CW Yard PC End", A10, "3032", true, INPUT_PULLUP);
  Sensors[14] = Sensor("CD UD-AC Yard side", A9, "3033", true, INPUT_PULLUP);
  Sensors[15] = Sensor("CD UD-UL Yard side", A8, "3034", true, INPUT_PULLUP);
  //Sensors[8] = Sensor("IR AC PC End Stopping sensor", 31, "3025", false);

  for (int i = 0; i < NumberOfSensors; i++) {
    Sensors[i].SetPinMode();
    UpdateSensor(i);

    String publishMessage = Sensors[i].State;
    String topic = Sensors[i].GetSensorPublishTopic();
  }

  Serial.println("Setup complete");
}
