// Output.h
#ifndef Output_h
#define Output_h

#include <Arduino.h>

class Output {
  private:
    

  public:     
    Output(String JMRIID = "", int pin = -1, bool IsInverted = false);  
    String JMRIId;   
    int pin;
    bool isInverted; 
    bool currentState;

};


#endif
