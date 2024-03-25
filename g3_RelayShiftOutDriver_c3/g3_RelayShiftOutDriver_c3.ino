#include <SPI.h>
#include <Ethernet.h>
#include "TBPubSubClient.h"
#include "ShiftRegister.h"
#include <Adafruit_PWMServoDriver.h>
#include "PWMBoard.h"
#include "Turnout.h"

#define NumberOfShiftOutRegisters 4
#define NumberOfShiftInRegisters 2
#define NumberOfPWMBoards 2

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


IPAddress ip(192, 168, 1, 47);
IPAddress myDns(192, 168, 1, 1);
IPAddress gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 255, 0);
IPAddress server(192, 168, 1, 29);

EthernetClient ethClient;
PubSubClient client(ethClient);

ShiftRegister shiftOutRegisters[NumberOfShiftOutRegisters];
ShiftRegister shiftInRegisters[NumberOfShiftInRegisters];
Output relays[8];

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

    Serial.println("Looking in relays for " + justTheID);
    int foundRelayIndex = -1;
    for (int r = 0; r < 8; r++) {
      if (relays[r].JMRIId == justTheID) {
        foundRelayIndex = r;
        break;
      }
    }

    if (foundRelayIndex >=0) {
      Serial.println("Found matching local relay on pin "+String(relays[foundRelayIndex].pin));
      int rVal = 0;
      if (message == "THROWN") rVal = 1;
      if (relays[foundRelayIndex].isInverted) rVal = !rVal;
      digitalWrite(relays[foundRelayIndex].pin,rVal);
      return; //no need to start searching through SRs, already found a match and dealt with it
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
    if (client.connect("G3RelayC3")) {
      Serial.println("connected");

      //Turnouts - electrofrog through relay - corner 3
      client.subscribe("track/turnout/3001");
      client.subscribe("track/turnout/3002");
      client.subscribe("track/turnout/3003");
      client.subscribe("track/turnout/3004");
      client.subscribe("track/turnout/3005");
      client.subscribe("track/turnout/3006");
      client.subscribe("track/turnout/3007");
      client.subscribe("track/turnout/3008");
      client.subscribe("track/turnout/1006");
      client.subscribe("track/turnout/6011");

      //Turnouts - electrofrog through relay - incline top
      client.subscribe("track/turnout/5001");
      client.subscribe("track/turnout/5002");
      client.subscribe("track/turnout/5003");
      client.subscribe("track/turnout/5004");
      client.subscribe("track/turnout/5005");
      client.subscribe("track/turnout/5006");
      client.subscribe("track/turnout/5007");

      //Signals
      client.subscribe("track/turnout/3041");
      client.subscribe("track/turnout/3042");
      client.subscribe("track/turnout/3043");
      client.subscribe("track/turnout/3044");
      client.subscribe("track/turnout/3045");
      client.subscribe("track/turnout/3046");

      //Signals
      client.subscribe("track/turnout/3049");
      client.subscribe("track/turnout/3050");
      client.subscribe("track/turnout/3051");
      client.subscribe("track/turnout/3052");
      client.subscribe("track/turnout/3053");
      client.subscribe("track/turnout/3054");
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
  for (int board = 0; board < NumberOfPWMBoards; board++) {
    for (int pin = 0; pin < PWMBoards[board].numberOfTurnouts; pin++) {
      if (PWMBoards[board].turnouts[pin].useSlowMotion) {
        ProcessPointsMoveWithSpeedControl(board, pin);
      }
    }
  }

  client.loop();

  ProcessShiftIn();
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
  //digitalWrite(latchPin, HIGH);
  //delay(20);
  digitalWrite(latchPin, LOW);
  delayMicroseconds(100);
  //for (int i = NumberOfShiftOutRegisters - 1; i >= 0; i--) {
  for (int i = 0; i < NumberOfShiftOutRegisters; i++) {
    shiftOut(dataPin, clockPin, MSBFIRST, shiftOutRegisters[i].PreviousValues);
    //Serial.println("Shift out " + String(i));
  }
  delayMicroseconds(100);
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

  relays[0] = Output("3001",9,true);
  relays[1] = Output("3002",10,true);
  relays[2] = Output("3003",11,true);
  relays[3] = Output("3004",12,true);
  relays[4] = Output("3005",13,true);
  relays[5] = Output("3006",15,false);
  relays[6] = Output("3007",16,false);
  relays[7] = Output("3008",17,true);

  for (int i = 0; i <= 7; i++) {
    pinMode(relays[i].pin, OUTPUT);
  }

  //Relay incline top - furthest away so first to be pushed
  shiftOutRegisters[0].Outputs[0] = Output("x5006", false);//Frog - DS PC End RHS
  shiftOutRegisters[0].Outputs[1] = Output("x5004", false); //Frog - DS Pi end RHS
  shiftOutRegisters[0].Outputs[2] = Output("x5005", true); //Frog - DS PC end LHS
  shiftOutRegisters[0].Outputs[3] = Output("x5003", true); //Frog - DS Pi end LHS
  shiftOutRegisters[0].Outputs[4] = Output("x5007", true);
  shiftOutRegisters[0].Outputs[5] = Output("x5008", false);
//  shiftOutRegisters[0].Outputs[6] = Output("", false);
//  shiftOutRegisters[0].Outputs[7] = Output("", true);

    //Signals corner 3
  shiftOutRegisters[1].Outputs[0] = Output("3049");
  shiftOutRegisters[1].Outputs[1] = Output("3050");
  shiftOutRegisters[1].Outputs[2] = Output("3051");
  shiftOutRegisters[1].Outputs[3] = Output("3052");
  shiftOutRegisters[1].Outputs[4] = Output("3053");
  shiftOutRegisters[1].Outputs[5] = Output("3054");

  //Signals under incline top 
  shiftOutRegisters[2].Outputs[0] = Output("3041");
  shiftOutRegisters[2].Outputs[1] = Output("3042");
  shiftOutRegisters[2].Outputs[2] = Output("3043");
  shiftOutRegisters[2].Outputs[3] = Output("3044");
  shiftOutRegisters[2].Outputs[4] = Output("3045");
  shiftOutRegisters[2].Outputs[5] = Output("3046");



  //Relay corner 3 - nearest so last to be pushed
  shiftOutRegisters[3].Outputs[0] = Output("x3001", true);
  shiftOutRegisters[3].Outputs[1] = Output("x3002", true);
  shiftOutRegisters[3].Outputs[2] = Output("x3003", true);
  shiftOutRegisters[3].Outputs[3] = Output("x3004", true);
  shiftOutRegisters[3].Outputs[4] = Output("x3005", true);
  shiftOutRegisters[3].Outputs[5] = Output("x3006", false);
  shiftOutRegisters[3].Outputs[6] = Output("x3007", false);
  shiftOutRegisters[3].Outputs[7] = Output("x3008", true);

  //Shift in registers
  shiftInRegisters[0].Sensors[0] = Sensor("IR C2AC2IR3", -1, "3047", true);
  shiftInRegisters[0].Sensors[1] = Sensor("IR C2AC1IR3", -1, "3041", true);
  shiftInRegisters[0].Sensors[2] = Sensor("IR C3C4IR1", -1, "3043", true);
  shiftInRegisters[0].Sensors[3] = Sensor("IR C3C3IR1", -1, "3044", true);
  shiftInRegisters[0].Sensors[4] = Sensor("IR C3C1IR1", -1, "3045", true);
  shiftInRegisters[0].Sensors[5] = Sensor("IR C3C2IR1", -1, "3046", true);
  shiftInRegisters[0].Sensors[6] = Sensor("IR C3C5IR1", -1, "3042", true);
  shiftInRegisters[0].Sensors[7] = Sensor("IR C2AC3IR3", -1, "3050", true);

  shiftInRegisters[1].Sensors[0] = Sensor("IR C3C3IR2", -1, "3054", true);
  shiftInRegisters[1].Sensors[1] = Sensor("xxIR C2AC3IR3", -1, "");
  shiftInRegisters[1].Sensors[2] = Sensor("IR C3C5IR2", -1, "3052", true);
  shiftInRegisters[1].Sensors[3] = Sensor("IR C2AC4IR3", -1, "3051", true);
  shiftInRegisters[1].Sensors[4] = Sensor("", -1, "", true);
  shiftInRegisters[1].Sensors[5] = Sensor("IR C3C4IR2", -1, "3049", true);
  shiftInRegisters[1].Sensors[6] = Sensor("IR C3C2IR2", -1, "3055", true);
  shiftInRegisters[1].Sensors[7] = Sensor("IR C3C1IR2", -1, "3053", true);

  PWMBoards[0].pwm = Adafruit_PWMServoDriver(0x40);
  PWMBoards[0].numberOfTurnouts = 7;
  PWMBoards[0].turnouts[0] = Turnout("5007", 1050, 2050, 2,3,true);
  PWMBoards[0].turnouts[1] = Turnout("5001", 1550, 1100, 2,3,true);
  PWMBoards[0].turnouts[2] = Turnout("5002", 1300, 1720, 2,3);
  PWMBoards[0].turnouts[3] = Turnout("5003", 900, 1550, 2,3,true);
  PWMBoards[0].turnouts[4] = Turnout("5004", 1100, 1700, 2,3,true);
  PWMBoards[0].turnouts[5] = Turnout("5005", 2200, 1300, 2,3);
  PWMBoards[0].turnouts[6] = Turnout("5006", 1900, 1300, 2,3);
  
  PWMBoards[0].pwm.begin();
  PWMBoards[0].pwm.setPWMFreq(50);  // This is the maximum PWM frequency
  PWMBoards[0].pwm.setOscillatorFrequency(25000000);

  PWMBoards[1].pwm = Adafruit_PWMServoDriver(0x41);
  PWMBoards[1].numberOfTurnouts = 10;
  PWMBoards[1].turnouts[0] = Turnout("3001", 1700, 950, 2,3);
  PWMBoards[1].turnouts[1] = Turnout("3002", 1900, 1100, 2,3);
  PWMBoards[1].turnouts[2] = Turnout("3003", 1100, 1700, 2,3);
  PWMBoards[1].turnouts[3] = Turnout("3004", 1100, 1900, 2,3);
  PWMBoards[1].turnouts[4] = Turnout("3005", 1900, 1100, 2,3);
  PWMBoards[1].turnouts[5] = Turnout("3006", 2050, 1300, 2,3);
  PWMBoards[1].turnouts[6] = Turnout("3007", 1150, 2000, 2,3);
  PWMBoards[1].turnouts[7] = Turnout("3008", 1600, 900, 2,3);
  PWMBoards[1].turnouts[8] = Turnout("1006", 1100, 1850, 2,3);
  PWMBoards[1].turnouts[9] = Turnout("6011", 1020, 1850, 2,3);
  
  
  PWMBoards[1].pwm.begin();
  PWMBoards[1].pwm.setPWMFreq(50);  // This is the maximum PWM frequency
  PWMBoards[1].pwm.setOscillatorFrequency(25000000);
}
