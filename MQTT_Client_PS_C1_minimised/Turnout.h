// Turnout.h
#ifndef Turnout_h
#define Turnout_h

#include <Arduino.h>

class Turnout {
  private:

  public: 
    int thrownVal;
    int closedVal;
    int currentPWMVal;
    int requiredPWMVal;
    String currentState;
    String requiredState;
    int stepSize;
    int delayTime;
    bool useSlowMotion;
    bool inDebounce;
    bool isAnLED;
    String jMRIId;
    bool switchOff;
    bool inSetup;
    unsigned long millisAtLastChange;
    unsigned long previousMillis;
    
    Turnout();
    Turnout(String JMRIId);
    Turnout(String JMRIId, int ThrownVal, int ClosedVal);  
    Turnout(String JMRIId,int ThrownVal, int ClosedVal, int StepSize, int DelayTime);  
    Turnout(String JMRIId,int ThrownVal, int ClosedVal, int StepSize, int DelayTime, bool SwitchOff); 
   
};

#endif
