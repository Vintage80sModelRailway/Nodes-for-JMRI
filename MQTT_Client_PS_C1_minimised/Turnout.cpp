// Turnout.cpp
#include "Turnout.h"

Turnout::Turnout() {
  useSlowMotion = false;
  currentPWMVal = 1500;
}

Turnout::Turnout(String JMRIId, int ThrownVal, int ClosedVal) {
  jMRIId = JMRIId;
  thrownVal = ThrownVal;
  closedVal = ClosedVal;
  useSlowMotion = false;
  currentPWMVal = 1500;
  switchOff = false;
  inSetup = true;
}

Turnout::Turnout(String JMRIId, int ThrownVal, int ClosedVal, int StepSize, int DelayTime) {
  jMRIId = JMRIId;
  thrownVal = ThrownVal;
  closedVal = ClosedVal;
  useSlowMotion = true;
  stepSize = StepSize;
  delayTime = DelayTime;
  currentPWMVal = 1500;
  switchOff = false;
  inSetup = true;
}

Turnout::Turnout(String JMRIId, int ThrownVal, int ClosedVal, int StepSize, int DelayTime, bool SwitchOff) {
  jMRIId = JMRIId;
  thrownVal = ThrownVal;
  closedVal = ClosedVal;
  useSlowMotion = true;
  stepSize = StepSize;
  delayTime = DelayTime;
  inSetup = true;
  currentPWMVal = 1500;
  switchOff = SwitchOff;
  inSetup = true;
}

Turnout::Turnout(String JMRIId) {
  jMRIId = JMRIId;
  isAnLED = true;
}
