// Output.h
#ifndef Output_h
#define Output_h

#include <Arduino.h>

class Output {
  private:
    

  public:     
    Output(String JMRIID = "", bool IsInverted = false);  
    String JMRIId;   
    bool isInverted; 

};


#endif
