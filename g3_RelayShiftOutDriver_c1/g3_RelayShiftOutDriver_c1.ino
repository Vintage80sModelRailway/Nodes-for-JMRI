#include <SPI.h>
#include <Ethernet.h>
#include "TBPubSubClient.h"
#include "ShiftRegister.h"
#include <Adafruit_PWMServoDriver.h>
#include "PWMBoard.h"
#include "Turnout.h"

#define NumberOfShiftOutRegisters 2
#define NumberOfShiftInRegisters 0
#define NumberOfPWMBoards 1

int dataPin = 2;
int latchPin = 3;
int clockPin = 4;

int dataIn = 5;
int load = 6;
int clockIn = 7;
int clockEnablePin = 8;

bool inSetup = true;

byte mac[] = {
  0x98, 0x4F, 0xEE, 0x05, 0x44, 0xD4
};


IPAddress ip(192, 168, 1, 48);
IPAddress myDns(192, 168, 1, 1);
IPAddress gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 255, 0);
IPAddress server(192, 168, 1, 29);

EthernetClient ethClient;
PubSubClient client(ethClient);

ShiftRegister shiftOutRegisters[NumberOfShiftOutRegisters];
ShiftRegister shiftInRegisters[NumberOfShiftInRegisters];

PWMBoard PWMBoards[NumberOfPWMBoards];

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

    //Find turnout with this ID
    for (int board = 0; board < NumberOfPWMBoards; board++) {
      for (int pin = 0; pin < PWMBoards[board].numberOfTurnouts; pin++) {
        if (PWMBoards[board].turnouts[pin].jMRIId == justTheID) {
          if (PWMBoards[board].turnouts[pin].inSetup == true) {
            //don't do loads of moving at startup
            if (message == "CLOSED") {
              PWMBoards[board].turnouts[pin].currentPWMVal = PWMBoards[board].turnouts[pin].closedVal+1;
            } else {
              PWMBoards[board].turnouts[pin].requiredPWMVal = PWMBoards[board].turnouts[pin].thrownVal+1;
            }
            PWMBoards[board].turnouts[pin].inSetup = false;
          }
          PWMBoards[board].turnouts[pin].requiredState = message;
          Serial.println(justTheID + " - needs to go " + message);
          if (PWMBoards[board].turnouts[pin].useSlowMotion) {
            PWMBoards[board].turnouts[pin].requiredState = message;
            if (message == "CLOSED") {
              PWMBoards[board].turnouts[pin].requiredPWMVal = PWMBoards[board].turnouts[pin].closedVal;

            } else {
              PWMBoards[board].turnouts[pin].requiredPWMVal = PWMBoards[board].turnouts[pin].thrownVal;
            }
            Serial.println("Required PWM VAL " + String(PWMBoards[board].turnouts[pin].requiredPWMVal));
            Serial.println("Current PWM VAL " + String(PWMBoards[board].turnouts[pin].currentPWMVal));
          }
          else {
            //MoveServoFast(board, pin, PWMBoards[board].turnouts[pin], message);
          }
          break;
        }
      }
    }

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
    if (client.connect("G3RelayC1")) {
      Serial.println("connected");

      //Turnouts - electrofrog through relay - corner 3
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
  //pinMode(load, OUTPUT);
  //pinMode(clockEnablePin, OUTPUT);
  //pinMode(clockIn, OUTPUT);
  //pinMode(dataIn, INPUT);


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

  //ProcessShiftIn();
  inSetup = false;
  Serial.println("Setup finished");
}

void loop()
{
  for (int board = 0; board < NumberOfPWMBoards; board++) {
    for (int pin = 0; pin < PWMBoards[board].numberOfTurnouts; pin++) {
      if (PWMBoards[board].turnouts[pin].useSlowMotion) {
        ProcessPointsMoveWithSpeedControl(board, pin);
      }
    }
  }

  client.loop();

  //ProcessShiftIn();
}

bool ProcessPointsMoveWithSpeedControl(int board, int pin)
{
  //Serial.println("Number of servos moving "+String(numberOfServosMoving));
  bool moveIsComplete = false;
  unsigned long currentMillis = millis();
  int requiredPosition = PWMBoards[board].turnouts[pin].requiredPWMVal;


  //    int totaldStepSize =  PWMBoards[board].turnouts[pin].stepSize + calculatedStepSize;
  int totaldStepSize =  PWMBoards[board].turnouts[pin].stepSize;
  if ((requiredPosition != PWMBoards[board].turnouts[pin].currentPWMVal && currentMillis - PWMBoards[board].turnouts[pin].previousMillis >= PWMBoards[board].turnouts[pin].delayTime))
  {
    //Serial.println(String(PWMBoards[board].turnouts[pin].currentPWMVal && currentMillis - PWMBoards[board].turnouts[pin].previousMillis)+" delay time "+String(PWMBoards[board].turnouts[pin].delayTime));
    PWMBoards[board].turnouts[pin].previousMillis = currentMillis;

    //Protect thee motors from accidentally silly, potentially damaging values
    if (requiredPosition > 800 && requiredPosition < 2351)
    {
      if (requiredPosition > PWMBoards[board].turnouts[pin].currentPWMVal)
      {
        int intendedPWMValue = PWMBoards[board].turnouts[pin].currentPWMVal + totaldStepSize;
        if (intendedPWMValue > requiredPosition)
        {
          intendedPWMValue = requiredPosition;
        }

        PWMBoards[board].turnouts[pin].currentPWMVal = intendedPWMValue;
        //Serial.println("Points move required on board "+String(board)+", device " + String(pin) + " to position " + String(intendedPWMValue) + ".");
        PWMBoards[board].pwm.writeMicroseconds(pin, intendedPWMValue);

        //if required position is now equal to current position, set frog polarity too
        if (requiredPosition == intendedPWMValue)
        {
          moveIsComplete = true;
        }
      }
      else// (requiredPosition < CurrentPWMValue[pin])
      {
        int intendedPWMValue = PWMBoards[board].turnouts[pin].currentPWMVal - totaldStepSize;
        if (intendedPWMValue < requiredPosition)
        {
          intendedPWMValue = requiredPosition;
        }

        PWMBoards[board].turnouts[pin].currentPWMVal = intendedPWMValue;
        //Serial.println("Points move required on board "+String(board)+", device " + String(pin) + " to position " + String(intendedPWMValue) + ".");
        PWMBoards[board].pwm.writeMicroseconds(pin, intendedPWMValue);

        //if required position is now equal to current position, set frog polarity too
        if (requiredPosition == intendedPWMValue)
        {
          moveIsComplete = true;
        }
      }
    }
    else {
      //Serial.println(String(PWMBoards[board].turnouts[pin].currentPWMVal && currentMillis - PWMBoards[board].turnouts[pin].previousMillis)+" delay out of  time "+String(PWMBoards[board].turnouts[pin].delayTime));
    }

    //Set frog polarity by bit value not servo position as some turnouts are inverted
    if (moveIsComplete) {
      PWMBoards[board].turnouts[pin].currentState = PWMBoards[board].turnouts[pin].requiredState;
      PWMBoards[board].turnouts[pin].currentPWMVal = PWMBoards[board].turnouts[pin].requiredPWMVal;
      if (PWMBoards[board].turnouts[pin].switchOff == true) {
        Serial.println("Switching servo "+String(pin)+" board "+String(board)+" off");
        PWMBoards[board].pwm.setPWM(pin, 0, 0);
      }
    }
  }
  return moveIsComplete;
}

void ProcessShiftOut() {
  digitalWrite(latchPin, LOW);

  //for (int i = NumberOfShiftOutRegisters - 1; i >= 0; i--) {
  for (int i = 0; i < NumberOfShiftOutRegisters; i++) {
    shiftOut(dataPin, clockPin, MSBFIRST, shiftOutRegisters[i].PreviousValues);
    //Serial.println("Shift out " + String(i));
  }

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

  //4 relay board - furthest away so first to be pushed
  shiftOutRegisters[0].Outputs[0] = Output("6009", false);//Frog - DS PC End RHS
  shiftOutRegisters[0].Outputs[1] = Output("6010", true); //Frog - DS Pi end RHS
  shiftOutRegisters[0].Outputs[2] = Output("6011", true); //Frog - DS PC end LHS
  shiftOutRegisters[0].Outputs[3] = Output("x5003", true); //Frog - DS Pi end LHS
  shiftOutRegisters[0].Outputs[4] = Output("x5007", true);
  shiftOutRegisters[0].Outputs[5] = Output("x5008", false);
  shiftOutRegisters[0].Outputs[6] = Output("x", false);
  shiftOutRegisters[0].Outputs[7] = Output("x", true);


  //Station relays - nearest so last to be pushed
  shiftOutRegisters[1].Outputs[0] = Output("6001", true);
  shiftOutRegisters[1].Outputs[1] = Output("6002", true);
  shiftOutRegisters[1].Outputs[2] = Output("6003", true);
  shiftOutRegisters[1].Outputs[3] = Output("6004", false);
  shiftOutRegisters[1].Outputs[4] = Output("6005", true);
  shiftOutRegisters[1].Outputs[5] = Output("6006", false);
  shiftOutRegisters[1].Outputs[6] = Output("6007", false);
  shiftOutRegisters[1].Outputs[7] = Output("6008", false);

  //Shift in registers
  //shiftInRegisters[0].Sensors[0] = Sensor("IR C2AC2IR3", -1, "3047", true);


  PWMBoards[0].pwm = Adafruit_PWMServoDriver(0x40);
  PWMBoards[0].numberOfTurnouts = 10;
  PWMBoards[0].turnouts[0] = Turnout("6001", 1300, 2100, 1,5); //station xover access line over hatch
  PWMBoards[0].turnouts[1] = Turnout("6002", 1870, 1150, 1,5); //station approach xover access line med LH near C2
  PWMBoards[0].turnouts[2] = Turnout("6003", 1100, 1750, 1,5); //station DS PC side
  PWMBoards[0].turnouts[3] = Turnout("6004", 1250, 1950, 1,5); //station DS Pi side
  PWMBoards[0].turnouts[4] = Turnout("6005", 1450, 2250, 1,5); //station approach DS yard side
  PWMBoards[0].turnouts[5] = Turnout("6006", 2200, 1350, 1,5); //station approach DS station side
  PWMBoards[0].turnouts[6] = Turnout("6007", 1700, 1100, 1,5); //station approach xover AC
  PWMBoards[0].turnouts[7] = Turnout("6008", 1200, 1750, 1,5); //station xover AC
  PWMBoards[0].turnouts[8] = Turnout("6009", 1550, 1000, 1,5,true); //station AC bay
  PWMBoards[0].turnouts[9] = Turnout("6010", 1300, 2200, 1,5,true); //station AC entrance
  
  PWMBoards[0].pwm.begin();
  PWMBoards[0].pwm.setPWMFreq(50);  // This is the maximum PWM frequency
  PWMBoards[0].pwm.setOscillatorFrequency(25000000);
}
