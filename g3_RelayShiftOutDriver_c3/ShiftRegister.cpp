#include "ShiftRegister.h"
#include <Arduino.h>

ShiftRegister::ShiftRegister(int DebounceMS)
  :  debounceMS(DebounceMS)
  ,  inDebounce(false)
  ,  millisAtLastChange(0)
{

}

bool ShiftRegister::ProcessShiftIn(int dataIn, int clockIn, uint8_t bitOrder) {
  bool publishRequired = false;
  byte data = shiftIn(dataIn, clockIn, bitOrder);
  if (data != PreviousValues) {
      bool updateDetected = UpdateShiftInputs(data, PreviousValues);
    if (updateDetected)
      publishRequired = true;
    //Serial.println(data, BIN);
    PreviousValues = data;
  //Serial.println(String(data));

  }
  //inDebounce = false;


  //    if (inDebounce == false)
  //    {
  //      Serial.println("Setting to in debounce");
  //      millisAtLastChange = millis();
  //      inDebounce = true;
  //    }
  //    unsigned long millisSinceLastChange = (millis() - millisAtLastChange);
  //
  //    Serial.println("Millis since last change " + String(millisSinceLastChange));
  //
  //    if (millisSinceLastChange >= debounceMS)
  //    {
  //      bool updateDetected = UpdateShiftInputs(data, PreviousValues);
  //      if (updateDetected)
  //        publishRequired = true;
  //      Serial.println(data, BIN);
  //      PreviousValues = data;
  //      inDebounce = false;
  //    }
  //  }
  //  else
  //  {
  //    //Status change has flicked on then off, or off then on, within the debounce period so don't change anything and set inDebounce back to false
  //    inDebounce = false;
  //  }
  return publishRequired;
}

uint8_t ShiftRegister::shiftIn(uint8_t dataPin, uint8_t clockPin, uint8_t bitOrder) {
  uint8_t value = 0;
  uint8_t i;
  for (i = 0; i < 8; ++i) {
    digitalWrite(clockPin, HIGH);
    if (bitOrder == LSBFIRST)
      value |= digitalRead(dataPin) << i;
    else
      value |= digitalRead(dataPin) << (7 - i);
    digitalWrite(clockPin, LOW);
  }
  return value;
}

bool ShiftRegister::UpdateShiftInputs(int data, int previousData) {
  bool publishRequired = false;
  int bits[8];
  int previousBits[8];

  for (int i = 0; i < 8; i++) {
    //bits[i] = (bool)((data >> (i % 8)) & 0x01);
    bits [i] = bitRead(data, i);
    previousBits[i] = bitRead(previousData, i);
    bool hasChanged = Sensors[i].UpdateShiftRegisterSensor(bits[i]);
    if (hasChanged) {
      publishRequired = true;
    }
  }
  return publishRequired;
}
