// LDR.h
#ifndef LDR_h
#define LDR_h

#include <Arduino.h>

class LDR {
  private:
    int lastKnownState;
       
    String sensorName;
    bool usingOnDebounce;
    bool usingOffDebounce;
    int debounceOnMS;
    int debounceValue;
    int debounceOffMS;
    int lightDarkThreshold;

    //Mode - 0 to off only, 1 to on only, 2 both

  public: 
    String JMRIId;
    String State;
    bool inDebounce;
    bool testMode;
    int pin; 
    int analogVal;
    unsigned long millisAtLastChange;
    
    LDR(String SensorName = "", int InputPin = -1, String JMRIID = "", int LightDarkThreshold = -1, int DebounceOnMS = -1, int DebounceOffMS = -1);  
    bool UpdateSensor();
    int GetRaw();
    String GetSensorPublishTopic();
};


#endif
