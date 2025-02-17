#include <SPI.h>
#include <Ethernet.h>
#include <PubSubClient.h>
#include "Sensor.h"
#include "RFIDOutput.h"

#define NumberOfSensors 18
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
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  String message = "";
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
    message = message + (char)payload[i];
  }

  Serial.println("Message: " + message);
  String strTopic = (String)topic;
  int pos = strTopic.lastIndexOf("/");
  int cmriId = -1;
  if (pos >= 0 && strTopic.indexOf("sensorhold") >= 0) {
    String justTheID = strTopic.substring(pos + 1);
    Serial.println("Just the ID " + justTheID);
    for (int i = 0; i < NumberOfSensors; i++) {
      if (Sensors[i].JMRIId == justTheID) {
        Serial.println("Sensor found");
        if (message == "1") {
          Sensors[i].onHold = true;
          Sensors[i].millisAtOnHold = millis();
          Serial.println(justTheID + " on hold");
        }
        else {
          Sensors[i].onHold = false;
          Serial.println(justTheID + " hold released");
          Sensors[i].millisAtOnHold = millis();
        }
      }
    }
  }
}

void setup()
{
  delay(5000);
  Serial.begin(19200);
  Serial1.begin(9600);
  Serial2.begin(9600);

  Serial.println("Starting");

  client.setServer(server, 1883);
  client.setCallback(callback);
  Ethernet.init(53);
  //Ethernet.begin(mac, ip, gateway, subnet);
  Ethernet.begin(mac);
  // Allow the hardware to sort itself out
  Serial.println("Ethernet begun");
  delay(1500);

  //Serial,print("-----Startup ");
  Serial.println(Ethernet.localIP());

    InitialiseConfig();

  if (!client.connected()) {
    reconnect();
  }


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
      for (int i = 0; i < NumberOfSensors; i++) {
        String jmriId = Sensors[i].JMRIId;
        String sub = "layout/sensorhold/" + jmriId;
        // Define

        // Length (with one extra character for the null terminator)
        int str_len = sub.length() + 1;

        // Prepare the character array (the buffer)
        char char_array[str_len];

        // Copy it over
        Serial.println("Sub "+sub);
        sub.toCharArray(char_array, str_len);
        client.subscribe(char_array);
      }
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
  Sensors[0] = Sensor("IR AC Yard Entry", 33, "3016", true, INPUT, 100, 1000);
  Sensors[1] = Sensor("zzCD CW Yard Bypass no longer used", 23, "3017", false, INPUT_PULLUP, 300, 200);
  //Sensors[13] = Sensor("zzCD AC Yard Bypass no longer used", 24, "3018", false,INPUT_PULLUP);
  Sensors[2] = Sensor("CD PC End AC", 45, "3019", true, INPUT_PULLUP, 20, 2000);
  Sensors[3] = Sensor("CD PC End CW", 42, "3020", true, INPUT_PULLUP, 20, 2000);
  //Sensors[4] = Sensor("CD PC end incline", 34, "3021", true, INPUT_PULLUP, 1000, 0);
  Sensors[4] = Sensor("IR C3C4IR2", 7, "3049", true, INPUT, 100, 300);
  Sensors[5] = Sensor("IR CW Yard Exit", 31, "3022", true, INPUT, 100, 300);
  Sensors[6] = Sensor("CD AC Yard bypass PC End", 29, "3023", true, INPUT_PULLUP);
  Sensors[7] = Sensor("CD CW Yard bypass PC End", 30, "3024", true, INPUT_PULLUP);
  Sensors[8] = Sensor("CD UD-UL PC End", 37, "3027", true, INPUT_PULLUP);
  Sensors[9] = Sensor("CD UD-CW PC End", 39, "3028", true, INPUT_PULLUP);
  Sensors[10] = Sensor("CD UD-AC PC End", 38, "3029", true, INPUT_PULLUP);
  Sensors[11] = Sensor("CD Incline PC End DTC", 36, "3030", true, INPUT_PULLUP);
  Sensors[12] = Sensor("CD UD-UL Yard PC End", 35, "3031", true, INPUT_PULLUP);
  Sensors[13] = Sensor("CD UD-CW Yard PC End", 34, "3032", true, INPUT_PULLUP);
  Sensors[14] = Sensor("CD UD-AC Yard side", 41, "3033", true, INPUT_PULLUP);
  Sensors[15] = Sensor("CD UD-UL Yard side", 40, "3034", true, INPUT_PULLUP);
  Sensors[16] = Sensor("CD UD-CW Yard", 28, "1034", true, INPUT_PULLUP, 100, 2000);
  //Sensors[8] = Sensor("IR AC PC End Stopping sensor", 31, "3025", false);
  Sensors[17] = Sensor("IR CW Yard Bypass PC End", 8, "3035", false, INPUT, 100, 300);

  for (int i = 0; i < NumberOfSensors; i++) {
    Sensors[i].SetPinMode();
    UpdateSensor(i);

    String publishMessage = Sensors[i].State;
    String topic = Sensors[i].GetSensorPublishTopic();
  }

  Serial.println("Setup complete");
}
