#include <SPI.h>
#include <Ethernet.h>
#include "TBPubSubClient.h"
#include "ShiftRegister.h"
#define NumberOfShiftOutRegisters 4
#define NumberOfShiftInRegisters 2

int dataPin = 2;
int latchPin = 3;
int clockPin = 4;

int dataIn = 5;
int load = 6;
int clockIn = 7;
int clockEnablePin = 8;

bool inSetup = true;

byte mac[] = {
  0x98, 0x4F, 0xEE, 0x05, 0x71, 0x81
};

IPAddress server(192, 168, 1, 29);

EthernetClient ethClient;
PubSubClient client(ethClient);

ShiftRegister shiftOutRegisters[NumberOfShiftOutRegisters];
ShiftRegister shiftInRegisters[NumberOfShiftInRegisters];

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
  if (pos >= 0 && strTopic.indexOf("turnout") >= 0) {
    int turnoutStatus = 0;
    if (message == "THROWN") turnoutStatus = 1;
    String justTheID = strTopic.substring(pos + 1);
    Serial.println("Just the ID " + justTheID);
    int fullId = justTheID.toInt();
    cmriId = fullId - 1001;
    //now set_bit for CMRI legacy compatibility

    Serial.println("Looking in SRs for " + justTheID);
    int foundRegisterIndex = -1;
    int foundPinIndex = -1;
    for (int sr = 0; sr < NumberOfShiftOutRegisters; sr++) {
      for (int pin = 0; pin < 8; pin++) {
        if (shiftOutRegisters[sr].Outputs[pin].JMRIId == justTheID) {
          foundRegisterIndex = sr;
          foundPinIndex = pin;
          Serial.println("Found at register " + String(sr) + " pin " + String(pin));
          break;
        }
      }
      if (foundRegisterIndex >= 0 && foundPinIndex >= 0) break;
    }
    if (foundRegisterIndex >= 0 && foundPinIndex >= 0) {
      int boolVal = 0;
      if (message == "THROWN") boolVal = 1;
      if (shiftOutRegisters[foundRegisterIndex].Outputs[foundPinIndex].isInverted) boolVal = !boolVal;

      int pv = shiftOutRegisters[foundRegisterIndex].PreviousValues;
      Serial.print("Bitwriting " + String(pv) + " adding " + String(boolVal) + " to ");
      bitWrite(pv, foundPinIndex, boolVal);
      Serial.println(String(pv));
      shiftOutRegisters[foundRegisterIndex].PreviousValues = pv;
      ProcessShiftOut();
    }
  }
}

void PublishToMQTT(String topic, String message)  {
  char topicBuffer[topic.length() + 1];
  char messageBuffer[message.length() + 1];

  topic.toCharArray(topicBuffer, topic.length() + 1);
  message.toCharArray(messageBuffer, message.length() + 1);

  Serial.println("Publish: " + topic + " - " + message);
  client.publish((const char*)topicBuffer, (const char*)messageBuffer,  true);
}

void reconnect() {
  // Loop until we're reconnected

  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("G3RelayC2")) {
      Serial.println("connected");

      //Turnouts - electrofrog through relay - corner 3
      //Ninth turnout on yard junctions
      client.subscribe("track/turnout/1005");
      client.subscribe("track/turnout/1006");

      //Yard junctions
      client.subscribe("track/turnout/1009");
      client.subscribe("track/turnout/1010");
      client.subscribe("track/turnout/1011");
      client.subscribe("track/turnout/1012");
      client.subscribe("track/turnout/1013");
      client.subscribe("track/turnout/1014");
      client.subscribe("track/turnout/1015");
      client.subscribe("track/turnout/1016");

      //C3 Relay
      client.subscribe("track/turnout/1017");
      client.subscribe("track/turnout/1018");

      //Relays for the two incline side turnouts
      client.subscribe("track/turnout/5001");
      client.subscribe("track/turnout/5002");
      client.subscribe("track/turnout/5008");

      //Signals
      client.subscribe("track/turnout/1041");
      client.subscribe("track/turnout/1042");
      client.subscribe("track/turnout/1043");
      client.subscribe("track/turnout/1044");
      client.subscribe("track/turnout/1045");
      client.subscribe("track/turnout/1046");

      client.subscribe("track/turnout/1049");
      client.subscribe("track/turnout/1050");
      client.subscribe("track/turnout/1051");
      client.subscribe("track/turnout/1052");
      client.subscribe("track/turnout/1053");
      client.subscribe("track/turnout/1054");

    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void setup()
{
  Serial.begin(115200);
  Serial.println("Network startup");

  system("ifup enp0s20f6");
  delay(10000);

  InitialiseConfig();
  pinMode(dataPin, OUTPUT);
  pinMode(clockPin, OUTPUT);
  pinMode(latchPin, OUTPUT);

  // Setup 74HC165 connections
  pinMode(load, OUTPUT);
  pinMode(clockEnablePin, OUTPUT);
  pinMode(clockIn, OUTPUT);
  pinMode(dataIn, INPUT);


  client.setServer(server, 1883);
  client.setCallback(callback);
  //Ethernet.begin(mac, ip, gateway, subnet);
  Ethernet.begin(mac);
  // Allow the hardware to sort itself out
  delay(1500);
  IPAddress localIp = Ethernet.localIP();

  Serial.print(localIp);
  Serial.println(" Network init done");

  if (!client.connected()) {
    reconnect();
  }

  ProcessShiftIn();
  inSetup = false;
  Serial.println("Setup finished");
}

void loop()
{


  client.loop();

  ProcessShiftIn();
}

void ProcessShiftOut() {
  digitalWrite(latchPin, HIGH);
  delay(20);
  digitalWrite(latchPin, LOW);

  //for (int i = NumberOfShiftOutRegisters - 1; i >= 0; i--) {
  for (int i = 0; i < NumberOfShiftOutRegisters; i++) {
    shiftOut(dataPin, clockPin, MSBFIRST, shiftOutRegisters[i].PreviousValues);
    //Serial.println("Shift out " + String(i));
  }
  delay(20);
  digitalWrite(latchPin, HIGH);
}

void ProcessShiftIn() {
  // Write pulse to load pin
  digitalWrite(load, LOW);
  delayMicroseconds(5);
  digitalWrite(load, HIGH);
  delayMicroseconds(5);

  // Get data from 74HC165
  digitalWrite(clockIn, HIGH);
  digitalWrite(clockEnablePin, LOW);
  for (int i = 0; i < NumberOfShiftInRegisters; i++) {
    bool publishRequired = shiftInRegisters[i].ProcessShiftIn(dataIn, clockIn, MSBFIRST);


    if (publishRequired || inSetup) {
      Serial.println("Publish required");
      for (int j = 0; j < 8; j++) {
        if (shiftInRegisters[i].Sensors[j].JMRIId != "" && (shiftInRegisters[i].Sensors[j].publishRequired || inSetup)) {
          String publishMessage = shiftInRegisters[i].Sensors[j].State;
          String topic = shiftInRegisters[i].Sensors[j].GetSensorPublishTopic();
          //Serial.print("Topic "+topic+" message "+publishMessage);
          if (topic != NULL && topic != "" && publishMessage != NULL && publishMessage != "")
            PublishToMQTT(topic, publishMessage);
          shiftInRegisters[i].Sensors[j].publishRequired = false;
        }
        else if (inSetup) {
          String publishMessage = shiftInRegisters[i].Sensors[j].State;
          String topic = shiftInRegisters[i].Sensors[j].GetSensorPublishTopic();
          Serial.println(" setup no publish Topic " + topic + " message " + publishMessage);
        }
      }
    }
  }
  digitalWrite(clockEnablePin, HIGH);
  digitalWrite(load, LOW);
}

void InitialiseConfig() {

  //Shift in registers

  shiftInRegisters[0].Sensors[0] = Sensor("IR C2AC1IR2", -1, "1049");
  shiftInRegisters[0].Sensors[1] = Sensor("IR C2AC3IR2", -1, "1050");
  shiftInRegisters[0].Sensors[2] = Sensor("IR C2AC2R2", -1, "1051");
  shiftInRegisters[0].Sensors[3] = Sensor("IR C2AC4IR2", -1, "1052");
  //shiftInRegisters[0].Sensors[4] = Sensor("IR C2C2IR3", -1, "1053");
  shiftInRegisters[0].Sensors[4] = Sensor("IR C2C2IR3xx", -1, "xx1048");
  shiftInRegisters[0].Sensors[5] = Sensor("IR C2C5IR3", -1, "1054");
  shiftInRegisters[0].Sensors[6] = Sensor("IIR C2C4IR3", -1, "1055");
  shiftInRegisters[0].Sensors[7] = Sensor("IR C2C3IR3", -1, "1056");

  shiftInRegisters[1].Sensors[0] = Sensor("IR C2AC1IR1", -1, "1041");
  shiftInRegisters[1].Sensors[1] = Sensor("IR C2AC2IR1", -1, "1042");
  shiftInRegisters[1].Sensors[2] = Sensor("IR C2AC3IR1", -1, "1043");
  shiftInRegisters[1].Sensors[3] = Sensor("IR C2AC4IR1", -1, "1044");
  shiftInRegisters[1].Sensors[4] = Sensor("IR CW Yard 5 Entry", -1, "1046");
  shiftInRegisters[1].Sensors[5] = Sensor("IR C2C1IR3b", -1, "1047");
  shiftInRegisters[1].Sensors[6] = Sensor("IR C2C2IR3b", -1, "1048");
  shiftInRegisters[1].Sensors[7] = Sensor("Empty", -1, "xx1");

  //Shift out registers
  //Signals at corner 1 - furthest away so first to be published
  shiftOutRegisters[0].Outputs[0] = Output("1049");
  shiftOutRegisters[0].Outputs[1] = Output("1050");
  shiftOutRegisters[0].Outputs[2] = Output("1051");
  shiftOutRegisters[0].Outputs[3] = Output("1052");
  shiftOutRegisters[0].Outputs[4] = Output("1053");
  shiftOutRegisters[0].Outputs[5] = Output("1054");

  //Signals at corner 2
  shiftOutRegisters[1].Outputs[0] = Output("1041");
  shiftOutRegisters[1].Outputs[1] = Output("1042");
  shiftOutRegisters[1].Outputs[2] = Output("1043");
  shiftOutRegisters[1].Outputs[3] = Output("1044");
  shiftOutRegisters[1].Outputs[4] = Output("1045");
  shiftOutRegisters[1].Outputs[5] = Output("1046");



  //Relay at corner 2
  shiftOutRegisters[2].Outputs[0] = Output("1009", true);
  shiftOutRegisters[2].Outputs[1] = Output("1010", true);
  shiftOutRegisters[2].Outputs[2] = Output("1011", true);
  shiftOutRegisters[2].Outputs[3] = Output("1012", false);
  shiftOutRegisters[2].Outputs[4] = Output("1013", true);
  shiftOutRegisters[2].Outputs[5] = Output("xx1014", false);//cable fallen out or relay
  shiftOutRegisters[2].Outputs[6] = Output("1015", false);
  shiftOutRegisters[2].Outputs[7] = Output("1016", true);

  //Relay between corner 2 and incline
  shiftOutRegisters[3].Outputs[0] = Output("1017", true);
  shiftOutRegisters[3].Outputs[1] = Output("5001", true);
  shiftOutRegisters[3].Outputs[2] = Output("5002", true);
  shiftOutRegisters[3].Outputs[3] = Output("1018", true);
  shiftOutRegisters[3].Outputs[4] = Output("1005", false);
  shiftOutRegisters[3].Outputs[5] = Output("1006", true);
  shiftOutRegisters[3].Outputs[6] = Output("5008", false);
  shiftOutRegisters[3].Outputs[7] = Output("1014", true);

}
