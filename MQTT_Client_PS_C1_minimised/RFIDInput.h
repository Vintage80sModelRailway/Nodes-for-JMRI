// RFIDInput.h
#ifndef RFIDInput_h
#define RFIDInput_h

#include <Arduino.h>
#include <SoftwareSerial.h>

class RFIDInput {
  private:


    int count = 0;


    void ClearBufferArray();

  public:
    bool NewTagRead;
    String Name;
    unsigned char buffer[64];
    SoftwareSerial SoftSerial;
    int txPin;
    int rxPin;

    RFIDInput(String SensorName, int RXPin, int TXPin);

    bool CheckForTag();
    void InitSoftwareSerial();
};

#endif
