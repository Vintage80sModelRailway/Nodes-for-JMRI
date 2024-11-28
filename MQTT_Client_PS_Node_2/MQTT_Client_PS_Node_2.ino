#include <SPI.h>
#include <Ethernet.h>
#include <PubSubClient.h>
#include <Adafruit_PWMServoDriver.h>
#include <Wire.h>
#include "PWMBoard.h"
#include "Turnout.h"
#include "Sensor.h"
#include "ShiftRegister.h"
#include "RFIDOutput.h"

#define NumberOfPWMBoards 1
#define NumberOfSensors 11
//#define NumberOfShiftInRegisters 0
#define NumberOfShiftOutRegisters 2
#define NumberOfRFIDReaders 2
#define OutputTurnoutThresholdID 2041

// Update these with values suitable for your network.
byte mac[6] = { 0x90, 0xA2, 0xDA, 0x58, 0x66, 0x25 };
IPAddress ip(192, 168, 1, 132);
IPAddress myDns(192, 168, 1, 1);
IPAddress gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 255, 0);
IPAddress server(192, 168, 1, 29);

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
EthernetClient ethClient;
PubSubClient client(ethClient);

PWMBoard PWMBoards[NumberOfPWMBoards];
Sensor Sensors[NumberOfSensors];
RFIDOutput TagReaders[NumberOfRFIDReaders] = {
  RFIDOutput(Serial1, "bayplatform"),
  RFIDOutput(Serial2, "inclinebottom")
};

int numberOfServosMoving = 0;
int servoSpeedMultipluer = 1;

bool inSetup = true;

//ShiftRegister shiftInRegisters[NumberOfShiftInRegisters];
ShiftRegister shiftOutRegisters[NumberOfShiftOutRegisters];

////Pin connected to DS of 74HC595 - SERIN
int dataPinOut = 32;
//Pin connected to ST_CP of 74HC595 - RCLCK
int latchPinOut = 31;
//Pin connected to SH_CP of 74HC595 - SRCLCK
int clockPinOut = 30;

//ShiftIn pins
//int load = 38;
//int clockEnablePin = 39;
//int dataIn = 40;
//int clockIn = 41;

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
  ProcessIncomingMessage(message, String(topic));

}

void setup()
{
  delay(5000);
  Serial.begin(19200);
  Serial1.begin(9600);
  Serial2.begin(9600);

  client.setServer(server, 1883);
  client.setCallback(callback);
  pwm.setPWMFreq(50);  // Analog servos run at ~50 Hz updates
  pwm.begin();
  //pwm.setOscillatorFrequency(27000000);

  Ethernet.init(53);
  //Ethernet.begin(mac, ip, gateway, subnet);
  Ethernet.begin(mac);

  // Allow the hardware to sort itself out
  delay(1500);
  InitialiseConfig();

  if (!client.connected()) {
    reconnect();
  }
  Serial.println("Finished connection");

  inSetup = false;

}

void loop()
{

  if (!client.connected()) {
    reconnect();
  }

  int numberOfServosMoving = 0;

  for (int i = 0; i < NumberOfSensors; i++) {
    UpdateSensor(i);
  }

  //ProcessShiftIn();

  for (int board = 0; board < NumberOfPWMBoards; board++) {
    for (int pin = 0; pin < PWMBoards[board].numberOfTurnouts; pin++) {
      if (PWMBoards[board].turnouts[pin].useSlowMotion) {
        ProcessPointsMoveWithSpeedControl(board, pin);
      }
    }
  }

  //RFID

  for (int r = 0; r < NumberOfRFIDReaders; r++) {
    bool newTag = TagReaders[r].CheckForTag();
    if (newTag) {
      Serial.println("Found " + TagReaders[r].Name);
      //Serial.println(TagReaders[r].Tag);
      String topic = "track/reporter/" + TagReaders[r].Name;
      PublishToMQTT(topic, String(TagReaders[r].Tag), false);
    }
  }
  client.loop();
}

void ProcessIncomingMessage(String message, String topic) {
  String strTopic = (String)topic;
  int pos = strTopic.lastIndexOf("/");
  int pin = -1;
  int boardId = -1;
  if (pos >= 0 && strTopic.indexOf("turnout") >= 0) {
    int turnoutStatus = 0;
    String justTheID = strTopic.substring(pos + 1);
    int iID = justTheID.toInt();

    if (iID > 0 && iID < OutputTurnoutThresholdID) {
      //THis is a PWM connected turnout
      //Find turnout with this ID
      for (int board = 0; board < NumberOfPWMBoards; board++) {
        for (int pin = 0; pin < PWMBoards[board].numberOfTurnouts; pin++) {
          if (PWMBoards[board].turnouts[pin].jMRIId == justTheID) {
            if (PWMBoards[board].turnouts[pin].inSetup == true) {
              //don't do loads of moving at startup
              if (message == "CLOSED") {
                PWMBoards[board].turnouts[pin].currentPWMVal = PWMBoards[board].turnouts[pin].closedVal + 1;
              } else {
                PWMBoards[board].turnouts[pin].requiredPWMVal = PWMBoards[board].turnouts[pin].thrownVal + 1;
              }
              PWMBoards[board].turnouts[pin].inSetup = false;
            }
            PWMBoards[board].turnouts[pin].requiredState = message;
            //Serial.println(justTheID + " - needs to go " + message);
            if (PWMBoards[board].turnouts[pin].useSlowMotion) {
              numberOfServosMoving++;
              PWMBoards[board].turnouts[pin].requiredState = message;
              if (message == "CLOSED") {
                PWMBoards[board].turnouts[pin].requiredPWMVal = PWMBoards[board].turnouts[pin].closedVal;
                if (inSetup) {
                  PWMBoards[board].turnouts[pin].currentPWMVal = PWMBoards[board].turnouts[pin].closedVal + 1;
                  //Serial.println("Setup closed " + justTheID + " " + String(PWMBoards[board].turnouts[pin].currentPWMVal));
                }
              } else {
                PWMBoards[board].turnouts[pin].requiredPWMVal = PWMBoards[board].turnouts[pin].thrownVal;
                if (inSetup) {
                  PWMBoards[board].turnouts[pin].currentPWMVal = PWMBoards[board].turnouts[pin].thrownVal - 1;
                  //Serial.println("Setup thrown " + justTheID + " " + String(PWMBoards[board].turnouts[pin].currentPWMVal));
                }
                //Serial.println("Required PWM VAL " + String(PWMBoards[board].turnouts[pin].requiredPWMVal));
                //Serial.println("Current PWM VAL " + String(PWMBoards[board].turnouts[pin].currentPWMVal));
              }
            }
            else {
              MoveServoFast(board, pin, PWMBoards[board].turnouts[pin], message);
            }
            break;
          }
        }
      }
    }
    else {
      //This is a shift register connected LED
      //get a matching ID;
      //Serial.println("Looking in SRs for " + justTheID);
      int foundRegisterIndex = -1;
      int foundPinIndex = -1;
      for (int sr = 0; sr < NumberOfShiftOutRegisters; sr++) {
        for (int pin = 0; pin < 8; pin++) {
          if (shiftOutRegisters[sr].LEDs[pin].JMRIId == justTheID) {
            foundRegisterIndex = sr;
            foundPinIndex = pin;
            //Serial.println("Found at register " + String(sr) + " pin " + String(pin));
            break;
          }
        }
        if (foundRegisterIndex >= 0 && foundPinIndex >= 0) break;
      }
      if (foundRegisterIndex >= 0 && foundPinIndex >= 0) {
        int boolVal = 0;
        if (message == "THROWN") boolVal = 1;

        int pv = shiftOutRegisters[foundRegisterIndex].PreviousValues;
        //Serial.print("Bitwriting " + String(pv, BIN) + " adding " + String(boolVal) + " to ");
        bitWrite(pv, foundPinIndex, boolVal);
        //Serial.println(String(pv, BIN));
        shiftOutRegisters[foundRegisterIndex].PreviousValues = pv;
        ProcessShiftOut();
      }
    }
  }
  else   if (pos >= 0 && strTopic.indexOf("sensorhold") >= 0) {
    String justTheID = strTopic.substring(pos + 1);
    //Serial.println("Just the ID " + justTheID);
    for (int i = 0; i < NumberOfSensors; i++) {
      if (Sensors[i].JMRIId == justTheID) {
        //Serial.println("Sensor found");
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

void PublishToMQTT(String topic, String message, bool retain)  {
  byte topicBuffer[topic.length() + 1];
  byte messageBuffer[message.length() + 1];

  topic.toCharArray(topicBuffer, topic.length() + 1);
  message.toCharArray(messageBuffer, message.length() + 1);

  Serial.println("Publish: " + topic + " - " + message);
  client.publish(topicBuffer, messageBuffer, message.length(), retain);
}

void MoveServoFast(int board, int pin, Turnout thisTurnout, String message) {
  Serial.println("Closed " + String(thisTurnout.closedVal) + " Thrown " + String(thisTurnout.thrownVal));
  bool servoMoved = false;
  if (message == "CLOSED") {
    if (thisTurnout.closedVal > 800 && thisTurnout.closedVal < 2200) {
      PWMBoards[board].pwm.writeMicroseconds(pin, thisTurnout.closedVal);
      servoMoved = true;
    }
  }
  else if (message == "THROWN" && thisTurnout.thrownVal > 800 && thisTurnout.thrownVal < 2200) {
    PWMBoards[board].pwm.writeMicroseconds(pin, thisTurnout.thrownVal);
    servoMoved = true;
  }

  if (servoMoved) PWMBoards[board].turnouts[pin - PWMBoards[board].CMRIIndexModifier].currentState = message;

  if (servoMoved && thisTurnout.needsFrogPolarityControl) {
    bool fullyOn = message != "CLOSED";
    if (fullyOn == true)
    {
      Serial.println("Fully on for board " + String(board) + " turnout " + String(pin) + " value " + message);
      if (thisTurnout.invertFrog == false)
      {
        setPWMStateFullyOn(board, pin + 8);
      }
      else
      {
        setPWMStateFullyOff(board, pin + 8);
      }
    }
    else
    {
      Serial.println("Fully off for board " + String(board) + " turnout " + String(pin) + " value " + message);
      if (thisTurnout.invertFrog == false)
      {
        setPWMStateFullyOff(board, pin + 8);
      }
      else
      {
        setPWMStateFullyOn(board, pin + 8);
      }
    }
  }
}

bool ProcessPointsMoveWithSpeedControl(int board, int pin)
{
  //Serial.println("Number of servos moving "+String(numberOfServosMoving));
  bool moveIsComplete = false;
  int calculatedStepSize = numberOfServosMoving;
  unsigned long currentMillis = millis();
  int requiredPosition = PWMBoards[board].turnouts[pin].requiredPWMVal;
  for (int i = 0; i < servoSpeedMultipluer; i++) {
    calculatedStepSize = calculatedStepSize * numberOfServosMoving;
  }

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
      numberOfServosMoving--;
      if (PWMBoards[board].turnouts[pin].switchOff == true) {
        //Serial.println("Switching servo " + String(pin) + " board " + String(board) + " off");
        PWMBoards[board].pwm.setPWM(pin, 0, 0);
      }
      bool fullyOn = PWMBoards[board].turnouts[pin].currentState != "CLOSED";
      if (fullyOn == true)
      {
        //Serial.println("Fully on in slow for board " + String(board) + " turnout " + String(pin) + " message " + PWMBoards[board].turnouts[pin].currentState);
        if (PWMBoards[board].turnouts[pin].invertFrog == false)
        {
          setPWMStateFullyOn(board, pin + 8);
        }
        else
        {
          setPWMStateFullyOff(board, pin + 8);
        }
      }
      else
      {
        Serial.println("Fully off in slow for board " + String(board) + " turnout " + String(pin) + " message " + PWMBoards[board].turnouts[pin].currentState);
        if (PWMBoards[board].turnouts[pin].invertFrog == false)
        {
          setPWMStateFullyOff(board, pin + 8);
        }
        else
        {
          setPWMStateFullyOn(board, pin + 8);
        }
      }
    }
  }
  return moveIsComplete;
}

void setPWMStateFullyOn(int board, int pin)
{
  if (pin < 8) return;
  PWMBoards[board].pwm.setPWM(pin, 4096, 0);
}

void setPWMStateFullyOff(int board, int pin)
{
  if (pin < 8) return;
  PWMBoards[board].pwm.setPWM(pin, 0, 4096);
}

void UpdateSensor(int i) {
  bool hasChanged = Sensors[i].UpdateSensor();
  if (hasChanged) {
    String publishMessage = Sensors[i].State;
    String topic = Sensors[i].GetSensorPublishTopic();

    PublishToMQTT(topic, publishMessage, true);

  }
}

void ProcessShiftOut() {
  digitalWrite(latchPinOut, LOW);

  for (int i = NumberOfShiftOutRegisters - 1; i >= 0; i--) {
    shiftOut(dataPinOut, clockPinOut, MSBFIRST, shiftOutRegisters[i].PreviousValues);
    //Serial.println("Shift out " + String(i));
  }

  digitalWrite(latchPinOut, HIGH);
}

void reconnect() {
  // Loop until we're reconnected

  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("arduinoClientNode2")) {
      Serial.println("connected");
      // Once connected, publish an announcement...
      //client.publish("outTopic","hello world");
      // ... and resubscribe
      //client.subscribe("#");
      client.subscribe("track/turnout/2001");
      client.subscribe("track/turnout/2002");
      client.subscribe("track/turnout/2003");
      client.subscribe("track/turnout/2004");
      client.subscribe("track/turnout/2005");
      client.subscribe("track/turnout/2006");
      client.subscribe("track/turnout/2007");
      client.subscribe("track/turnout/2008");

      client.subscribe("track/turnout/2041");
      client.subscribe("track/turnout/2042");
      client.subscribe("track/turnout/2043");
      client.subscribe("track/turnout/2044");
      client.subscribe("track/turnout/2045");
      client.subscribe("track/turnout/2046");

      client.subscribe("track/turnout/2049");
      client.subscribe("track/turnout/2050");
      client.subscribe("track/turnout/2051");
      client.subscribe("track/turnout/2052");
      client.subscribe("track/turnout/2053");
      client.subscribe("track/turnout/2054");

      for (int i = 0; i < NumberOfSensors; i++) {
        String jmriId = Sensors[i].JMRIId;
        String sub = "layout/sensorhold/" + jmriId;
        int str_len = sub.length() + 1;
        char char_array[str_len];

        Serial.println("Sub " + sub);
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
  //Board 1 - under upper incline junction
  PWMBoards[0].pwm = Adafruit_PWMServoDriver();
  PWMBoards[0].numberOfTurnouts = 8;
  PWMBoards[0].CMRIIndexModifier = 0;
  PWMBoards[0].turnouts[0] = Turnout("2001", 1000, 1800, 1, 5, false); //thrown, closed,feedback sensor pin, invertfrog, invert sensor
  PWMBoards[0].turnouts[1] = Turnout("2002", 1650, 1150, 1, 5, true); //thrown, closed, invertfrog
  PWMBoards[0].turnouts[2] = Turnout("2003", 1200, 2100, 1, 5, true); //thrown, closed, invertfrog
  PWMBoards[0].turnouts[3] = Turnout("2004", 1290, 1800, 1, 5, false); //thrown, closed, invertfrog
  PWMBoards[0].turnouts[4] = Turnout("2005", 1700, 1250, 1, 5, true); //thrown, closed, invertfrog
  PWMBoards[0].turnouts[5] = Turnout("2006", 1810, 1200, 1, 5, false); //thrown, closed, invertfrog
  PWMBoards[0].turnouts[6] = Turnout("2007", 1850, 1200, 1, 5, true); //thrown, closed, invertfrog
  PWMBoards[0].turnouts[7] = Turnout("2008", 2000, 1250, 1, 5, false); //thrown, closed, invertfrog

  PWMBoards[0].pwm.begin();
  PWMBoards[0].pwm.setPWMFreq(50);  // This is the maximum PWM frequency
  PWMBoards[0].pwm.setOscillatorFrequency(25000000);


  //Shift out registers
  shiftOutRegisters[0].LEDs[0] = LED("2041");
  shiftOutRegisters[0].LEDs[1] = LED("2042");
  shiftOutRegisters[0].LEDs[2] = LED("2043");
  shiftOutRegisters[0].LEDs[3] = LED("2044");
  shiftOutRegisters[0].LEDs[4] = LED("2045");
  shiftOutRegisters[0].LEDs[5] = LED("2046");

  shiftOutRegisters[1].LEDs[0] = LED("2049");
  shiftOutRegisters[1].LEDs[1] = LED("2050");
  shiftOutRegisters[1].LEDs[2] = LED("2051");
  shiftOutRegisters[1].LEDs[3] = LED("2052");
  shiftOutRegisters[1].LEDs[4] = LED("2053");
  shiftOutRegisters[1].LEDs[5] = LED("2054");

  for (int sr = 0; sr < NumberOfShiftOutRegisters; sr++) {
    shiftOutRegisters[sr].PreviousValues = 0;
  }


  //Sensors - Name, Pin, JMRIId, IsInverted = false, Pinmode = INPUT, Lastknownvalue = 0
  Sensors[0] = Sensor("IR CW Junction PC End Stopping sensor", 33, "2033", true);
  Sensors[1] = Sensor("CD CW Lower junction Pi End", 34, "2034", true, INPUT_PULLUP);
  Sensors[2] = Sensor("CD CW lower junction PC End", 35, "2035", true, INPUT_PULLUP);
  Sensors[3] = Sensor("CD AC junction Pi End", 36, "2036", true, INPUT_PULLUP);
  Sensors[4] = Sensor("CD Incline lower junction", 37, "2037", true, INPUT_PULLUP);
  Sensors[5] = Sensor("CD spare", 38, "2038", true, INPUT_PULLUP);
  Sensors[6] = Sensor("CD AC Junction PC End", 39, "2039", true, INPUT_PULLUP);
  Sensors[7] = Sensor("UD-UL Station Platform 2", 26, "2040", true, INPUT_PULLUP);
  Sensors[8] = Sensor("UD-CW Station CW", 29, "2041", true, INPUT_PULLUP);
  Sensors[9] = Sensor("UD-AC Station AC", 28, "2042", true, INPUT_PULLUP);
  Sensors[10] = Sensor("UD-AC Station Platform 1", 27, "2043", true, INPUT_PULLUP);

  for (int i = 0; i < NumberOfSensors; i++) {
    Sensors[i].SetPinMode();
    UpdateSensor(i);

    String publishMessage = Sensors[i].State;
    String topic = Sensors[i].GetSensorPublishTopic();
  }

  // Setup 74HC165 connections
  //  pinMode(load, OUTPUT);
  //  pinMode(clockEnablePin, OUTPUT);
  //  pinMode(clockIn, OUTPUT);
  //  pinMode(dataIn, INPUT);
  pinMode(latchPinOut, OUTPUT);
  pinMode(clockPinOut, OUTPUT);
  pinMode(dataPinOut, OUTPUT);

  Serial.println("Setup complete");
}
