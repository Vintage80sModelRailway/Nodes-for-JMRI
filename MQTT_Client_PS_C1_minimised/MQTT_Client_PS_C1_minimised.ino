#include <SPI.h>
#include <Ethernet.h>
#include <PubSubClient.h>
#include <Adafruit_PWMServoDriver.h>
#include <Wire.h>
#include "PWMBoard.h"
#include "Turnout.h"
#include "Sensor.h"
#include "RFIDInput.h"
#include <SoftwareSerial.h>

#define NumberOfPWMBoards 1
#define NumberOfSensors 23
#define NumberOfRFIDReaders 2

// Update these with values suitable for your network.
byte mac[6] = { 0x90, 0xA2, 0xDA, 0x3A, 0xD4, 0x8D };
IPAddress server(192, 168, 1, 29);

EthernetClient ethClient;
PubSubClient client(ethClient);

PWMBoard PWMBoards[NumberOfPWMBoards];
Sensor Sensors[NumberOfSensors];
SoftwareSerial ss(11,5);

RFIDInput TagReaders[NumberOfRFIDReaders] =
{
  RFIDInput("yardexitac", 10, 4),
  RFIDInput("yardentrycw", 11, 5)
};

int numberOfServosMoving = 0;
int servoSpeedMultipluer = 1;

void callback(char* topic, byte* payload, unsigned int length) {
  String message = "";
  for (int i = 0; i < length; i++) {
    message = message + (char)payload[i];
  }
  Serial.println("New message " + String(topic) + " - " + message);
  ProcessIncomingMessage(message, String(topic));
}

void setup()
{
  Serial.begin(9600);
  ss.begin(9600);
  client.setServer(server, 1883);
  client.setCallback(callback);

  Ethernet.init(53);
  Ethernet.begin(mac);
  // Allow the hardware to sort itself out
  delay(1500);

  if (!client.connected()) {
    reconnect();
  }
  Serial.println("Past initial connection");
  InitialiseConfig();
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


  for (int board = 0; board < NumberOfPWMBoards; board++) {
    for (int pin = 0; pin < PWMBoards[board].numberOfTurnouts; pin++) {
      if (PWMBoards[board].turnouts[pin].useSlowMotion) {
        ProcessPointsMoveWithSpeedControl(board, pin);
      }
    }
  }
  if (ss.available()) {
    while (ss.available()) {
      char c = ss.read();
      Serial.write(c);
    }
    Serial.println("Found in main main");
  }
  /*
  for (int r = 0; r < NumberOfRFIDReaders; r++) {
    if (TagReaders[r].SoftSerial.available()) {
      Serial.println("Found in main");
    }
    bool newTagFound = TagReaders[r].CheckForTag();
    if (newTagFound) {
      Serial.write(TagReaders[r].buffer, 64);
      Serial.println();
      Serial.println("New tag");
      String topic = "track/reporter/" + TagReaders[r].Name;
      byte topicBuffer[topic.length() + 1];
      topic.toCharArray(topicBuffer, topic.length() + 1);
      client.publish(topicBuffer, TagReaders[r].buffer, 64, false);
    }
  }
  */
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
          Serial.println(justTheID + " - needs to go " + message);
          if (PWMBoards[board].turnouts[pin].useSlowMotion) {
            numberOfServosMoving++;
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
            MoveServoFast(board, pin, PWMBoards[board].turnouts[pin], message);
          }
          break;
        }
      }
    }
  }
}

void PublishToMQTT(String topic, String message)  {
  byte topicBuffer[topic.length() + 1];
  byte messageBuffer[message.length() + 1];

  topic.toCharArray(topicBuffer, topic.length() + 1);
  message.toCharArray(messageBuffer, message.length() + 1);

  Serial.println("Publish: " + topic + " - " + message);
  client.publish(topicBuffer, messageBuffer, message.length(), true);
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

  if (servoMoved) PWMBoards[board].turnouts[pin].currentState = message;

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
        Serial.println("Switching servo " + String(pin) + " board " + String(board) + " off");
        PWMBoards[board].pwm.setPWM(pin, 0, 0);
      }
    }
  }
  return moveIsComplete;
}

void UpdateSensor(int i) {
  bool hasChanged = Sensors[i].UpdateSensor();
  if (hasChanged) {
    String publishMessage = Sensors[i].State;
    String topic = Sensors[i].GetSensorPublishTopic();

    PublishToMQTT(topic, publishMessage);

  }
}

void reconnect() {
  // Loop until we're reconnected

  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("arduinoClientCorner2")) {
      Serial.println("connected");
      // Once connected, publish an announcement...
      //client.publish("outTopic","hello world");
      // ... and resubscribe
      //client.subscribe("#");
      //client.subscribe("track/turnout/1001");
      //client.subscribe("track/turnout/1002");
      //client.subscribe("track/turnout/1003");
      //client.subscribe("track/turnout/1004");
      //client.subscribe("track/turnout/1005");
      //client.subscribe("track/turnout/1006");

      client.subscribe("track/turnout/1009");
      client.subscribe("track/turnout/1010");
      client.subscribe("track/turnout/1011");
      client.subscribe("track/turnout/1012");
      client.subscribe("track/turnout/1013");
      client.subscribe("track/turnout/1014");
      client.subscribe("track/turnout/1015");
      client.subscribe("track/turnout/1016");
      client.subscribe("track/turnout/1017");
      client.subscribe("track/turnout/1018");
      client.subscribe("track/turnout/1005");
      client.subscribe("track/turnout/5008");
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
  //PWMBoards[0].pwm = Adafruit_PWMServoDriver(0x40);
  //PWMBoards[0].numberOfTurnouts = 3;
  //PWMBoards[0].turnouts[0] = Turnout("5001", 1900, 1150, true);
  //PWMBoards[0].turnouts[1] = Turnout("xxRemove", 1900, 1150, false);
  //PWMBoards[0].turnouts[2] = Turnout("5002", 1350, 2150, true); //thrown, closed, invertfrog
  //PWMBoards[0].turnouts[3] = Turnout("xxRemove", 1250, 2100, false);
  //PWMBoards[0].turnouts[4] = Turnout("1005xxRemove", 2100, 1250, 1, 5, 11, false, false);
  //PWMBoards[0].turnouts[5] = Turnout("1006xxRemove", 1200, 1900, 1, 5, true);


  //Board 2 - by Arduino, corner 2
  PWMBoards[0].pwm = Adafruit_PWMServoDriver(0x41);
  PWMBoards[0].numberOfTurnouts = 12;
  PWMBoards[0].turnouts[0] = Turnout("1009", 1000, 2100, 1, 5);
  PWMBoards[0].turnouts[1] = Turnout("1010", 1200, 1800, 1, 5);
  PWMBoards[0].turnouts[2] = Turnout("1011", 1900, 1100, 1, 5);
  PWMBoards[0].turnouts[3] = Turnout("1012", 2000, 1200, 1, 5);
  PWMBoards[0].turnouts[4] = Turnout("1013", 1250, 2150, 1, 5);
  PWMBoards[0].turnouts[5] = Turnout("1014", 1250, 2100, 1, 5);
  PWMBoards[0].turnouts[6] = Turnout("1015", 1000, 1900, 1, 5);
  PWMBoards[0].turnouts[7] = Turnout("1016", 1200, 1900, 1, 5);
  PWMBoards[0].turnouts[8] = Turnout("1017", 1700, 1000, 1, 5);
  PWMBoards[0].turnouts[9] = Turnout("1018", 1000, 1750, 1, 5);
  PWMBoards[0].turnouts[10] = Turnout("1005", 1850, 1260, 1, 5);
  PWMBoards[0].turnouts[11] = Turnout("5008", 2140, 1470, 1, 5);

  PWMBoards[0].pwm.begin();
  PWMBoards[0].pwm.setPWMFreq(50);  // This is the maximum PWM frequency
  PWMBoards[0].pwm.setOscillatorFrequency(25000000);

  //Sensors - Name, Pin, JMRIId, IsInverted = false, Pinmode = INPUT, Lastknownvalue = 0
  Sensors[0] = Sensor("CD Pi end CW", 32, "1017", true, INPUT_PULLUP, 2000, 0);
  Sensors[1] = Sensor("CD Pi end AC", 33, "1018", true, INPUT_PULLUP, 2000, 0);
  Sensors[2] = Sensor("CD Pi end incline", 28, "1019", true, INPUT_PULLUP, 2000, 0);
  Sensors[3] = Sensor("IR CW Yard Entry", 29, "1020", true, INPUT, 800, 0);
  Sensors[4] = Sensor("IR CW Yard entry lower", 27, "1021", true, INPUT, 800, 0);
  Sensors[5] = Sensor("IR AC Yard Exit", 26, "1022", true, INPUT, 800, 0);
  Sensors[6] = Sensor("IR AC Yard Bypass stopping sensor", 36, "1023", true, INPUT, 800, 0);
  Sensors[7] = Sensor("CD AC Yard bypass Pi End", 25, "1024", false, INPUT_PULLUP);
  Sensors[8] = Sensor("CD CW Yard bypass Pi end", 24, "1025", false, INPUT_PULLUP);
  Sensors[9] = Sensor("CD Incline Top", 22, "1026", false, INPUT_PULLUP);
  Sensors[10] = Sensor("CD corner 2 unused 1", 23, "1027", false, INPUT_PULLUP);
  Sensors[11] = Sensor("CD Incline station 1", 47, "1028", true, INPUT_PULLUP, 2000, 0);
  Sensors[12] = Sensor("CD Incline station 2", 46, "1029", true, INPUT_PULLUP, 2000, 0);

  Sensors[13] = Sensor("IR C2AC3IR1", 2, "1043", true, INPUT_PULLUP, 2000, 0);
  Sensors[14] = Sensor("IR C2AC2IR1", 44, "1042", true, INPUT_PULLUP, 2000, 0);
  Sensors[15] = Sensor("IR C2AC1IR1", 45, "1041", true, INPUT_PULLUP, 2000, 0);
  Sensors[16] = Sensor("IR CW Yard 5 Entry", 43, "1046", true, INPUT_PULLUP, 2000, 0);
  Sensors[17] = Sensor("IR C2AC4IR12", 42, "1044", true, INPUT_PULLUP, 2000, 0);

  Sensors[18] = Sensor("CD UD-UL Yard Pi End", 41, "1030", false, INPUT_PULLUP);
  Sensors[19] = Sensor("CD UD-CW Yard Pi end", 40, "1031", false, INPUT_PULLUP);
  Sensors[20] = Sensor("CD Incline Pi End DTC", 39, "1032", false, INPUT_PULLUP);
  Sensors[21] = Sensor("CD UD-AC Yard Pi end", 38, "1033", false, INPUT_PULLUP);
  Sensors[22] = Sensor("CD UD-CW Yard", 35, "1034", false, INPUT_PULLUP);




  for (int i = 0; i < NumberOfSensors; i++) {
    Sensors[i].SetPinMode();
    UpdateSensor(i);

    String publishMessage = Sensors[i].State;
    String topic = Sensors[i].GetSensorPublishTopic();
  }

  for (int i = 0; i < NumberOfRFIDReaders; i++) {
    TagReaders[i].SoftSerial = SoftwareSerial(TagReaders[i].rxPin,TagReaders[i].txPin);
    TagReaders[i].SoftSerial.begin(9600);
  }



  Serial.println("Setup complete");
}
