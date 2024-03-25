#include "Output.h"
#include <Arduino.h>

Output::Output(String JMRIID, int Pin, bool IsInverted)
:  JMRIId(JMRIID)
,   pin(Pin)
,  isInverted(IsInverted)

{
  currentState = false;
}
