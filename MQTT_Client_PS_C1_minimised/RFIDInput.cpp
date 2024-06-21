#include "RFIDInput.h"
#include <Arduino.h>
#include <SoftwareSerial.h>

RFIDInput::RFIDInput(String SensorName, int RXPin, int TXPin)
  : Name(SensorName)
  , txPin(TXPin)
  , rxPin(RXPin)
  , SoftSerial(RXPin, TXPin)
{
  //SoftSerial = SoftwareSerial(RXPin, TXPin);
  //SoftSerial.begin(9600);
  NewTagRead = false;
  count = 0;
}

void RFIDInput::InitSoftwareSerial() {
  SoftSerial = SoftwareSerial(rxPin, txPin);
}

bool RFIDInput::CheckForTag() {
  NewTagRead = false;
  //Serial.println("Checking "+Name);
  if (SoftSerial.available())
  {
    Serial.println("Found something");
    ClearBufferArray();
    while (SoftSerial.available())              // reading data into char array
    {
      buffer[count] = SoftSerial.read();
      count++;
      if (count == 64)
      {
        NewTagRead = true;
        //for (int i = 0; i < count; i++)
        //{
        //  CurrentTagID[i] = buffer[i];
        //}
        break;
      }
    }
    count = 0;                      // set counter of while loop to zero
  }
}

void RFIDInput::ClearBufferArray()                 // function to clear buffer array
{
  // clear all index of array with command NULL
  for (int i = 0; i < count; i++)
  {
    buffer[i] = NULL;
  }
}
