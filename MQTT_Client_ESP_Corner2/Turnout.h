// Turnout.h
#ifndef Turnout_h
#define Turnout_h

#include <Arduino.h>
#include "ESP32Servo.h"

class Turnout {
  private:
    Servo thisServo;
    void ProcessMoveSlow();
    void ProcessMove();
    int pin;  
    int stepSize;
    int delayTime;
    bool useSlowMotion;
    bool usesDegrees;
    unsigned long millisAtLastChange;
    unsigned long previousMillis;
    int detachTimeout;
    unsigned long millisAtLastMoveCompletion;
    bool useDetach;
    int channel;
    
  public:  
    String jMRIId;
    String requiredState;
    int requiredPWMVal;
    int thrownVal;
    int closedVal;
    String currentState; 
    int currentPWMVal;  
    
    Turnout();
    Turnout(String JMRIId);
    Turnout(int Channel, String JMRIId, int Pin, int ThrownVal, int ClosedVal, bool UsesDegrees, bool useDetach);
    Turnout(int Channel, String JMRIId, int Pin, int ThrownVal, int ClosedVal, bool UsesDegrees, int StepSize, int DelayTime, bool UseDetach);

    void CheckState();
    void Start();
    void Stop();
};

#endif
