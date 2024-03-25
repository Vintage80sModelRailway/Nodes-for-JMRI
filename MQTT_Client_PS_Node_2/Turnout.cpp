// Turnout.cpp
#include "Turnout.h"

Turnout::Turnout() {
  needsFrogPolarityControl = false;
  useSlowMotion = false;
  hasFeedbackSensor = false;
  motorHasNotMovedYet  = true;
  currentPWMVal = 1500;
  switchOff = false;
  inSetup = true;
}

Turnout::Turnout(String JMRIId, int ThrownVal, int ClosedVal) {
  jMRIId = JMRIId;
  thrownVal = ThrownVal;
  closedVal = ClosedVal;
  needsFrogPolarityControl = false;
  useSlowMotion = false;
  hasFeedbackSensor = false;
  motorHasNotMovedYet  = true;
  currentPWMVal = 1500;
  switchOff = false;
  inSetup = true;
}

Turnout::Turnout(String JMRIId, int ThrownVal, int ClosedVal, bool InvertFrog) {
  jMRIId = JMRIId;
  thrownVal = ThrownVal;
  closedVal = ClosedVal;
  invertFrog = InvertFrog;
  needsFrogPolarityControl = true;
  hasFeedbackSensor = false;
  useSlowMotion = false;
  motorHasNotMovedYet  = true;
  relayHasNotBeenSetYet = true;
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
  needsFrogPolarityControl = false;
  hasFeedbackSensor = false;
  motorHasNotMovedYet  = true;
  currentPWMVal = 1500;
  switchOff = false;
  inSetup = true;
}

Turnout::Turnout(String JMRIId, int ThrownVal, int ClosedVal, int StepSize, int DelayTime, bool InvertFrog) {
  jMRIId = JMRIId;
  thrownVal = ThrownVal;
  closedVal = ClosedVal;
  invertFrog = InvertFrog;
  needsFrogPolarityControl = true;
  useSlowMotion = true;
  hasFeedbackSensor = false;
  stepSize = StepSize;
  delayTime = DelayTime;
  motorHasNotMovedYet  = true;
  relayHasNotBeenSetYet = true;
  currentPWMVal = 1500;
  switchOff = false;
  inSetup = true;
}

Turnout::Turnout(String JMRIId, int ThrownVal, int ClosedVal, int StepSize, int DelayTime, bool InvertFrog, bool SwitchOff) {
  jMRIId = JMRIId;
  thrownVal = ThrownVal;
  closedVal = ClosedVal;
  invertFrog = InvertFrog;
  needsFrogPolarityControl = true;
  useSlowMotion = true;
  hasFeedbackSensor = false;
  stepSize = StepSize;
  delayTime = DelayTime;
  motorHasNotMovedYet  = true;
  relayHasNotBeenSetYet = true;
  currentPWMVal = 1500;
  switchOff = SwitchOff;
  inSetup = true;
}


Turnout::Turnout(String JMRIId) {
  jMRIId = JMRIId;
  isAnLED = true;
}
