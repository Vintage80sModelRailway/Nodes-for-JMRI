// ShiftRegister.h
#ifndef ShiftRegister_h
#define ShiftRegister_h

#include <Arduino.h>
#include "Sensor.h"
#include "Output.h"

class ShiftRegister {
  private:    
    int debounceMS;
    bool inDebounce;
    unsigned long millisAtLastChange;
    uint8_t shiftIn(uint8_t dataPin, uint8_t clockPin, uint8_t bitOrder);
    bool UpdateShiftInputs(int data, int previousData);

  public:     
    Output Outputs[8];
    Sensor Sensors[8];
    char PreviousValues;
    bool NeedsToPublish;

    ShiftRegister(int DebounceMS = 300);
    bool ProcessShiftIn(int dataIn, int clockIn, uint8_t bitOrder);
    
};


#endif
