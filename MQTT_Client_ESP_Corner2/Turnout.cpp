// Turnout.cpp
#include "Turnout.h"
#include "ESP32Servo.h"

#define DetachTimeout 1000;

Turnout::Turnout() {
  useSlowMotion = false;
  currentPWMVal = 1500;
  //thisServo.setPeriodHertz(50);
  pin = -1;
  detachTimeout = DetachTimeout;
}

Turnout::Turnout(int Channel, String JMRIId, int Pin, int ThrownVal, int ClosedVal, bool UsesDegrees, bool UseDetach) {
  jMRIId = JMRIId;
  thrownVal = ThrownVal;
  closedVal = ClosedVal;
  useSlowMotion = false;
  currentPWMVal = 1500;
  //thisServo.setPeriodHertz(50);
  pin = Pin;
  usesDegrees = UsesDegrees;
  if (usesDegrees) {
    currentPWMVal = 90;
  }
  else {
    currentPWMVal = 1500;
  }
  detachTimeout = DetachTimeout;
  useDetach = UseDetach;
  thisServo = Servo(Channel);
  channel = Channel;
}

Turnout::Turnout(int Channel, String JMRIId, int Pin, int ThrownVal, int ClosedVal, bool UsesDegrees, int StepSize, int DelayTime, bool UseDetach) {
  jMRIId = JMRIId;
  thrownVal = ThrownVal;
  closedVal = ClosedVal;
  useSlowMotion = true;
  stepSize = StepSize;
  delayTime = DelayTime;
  //currentPWMVal = 1500;
  //thisServo.setPeriodHertz(50);
  pin = Pin;
  usesDegrees = UsesDegrees;
  detachTimeout = DetachTimeout;
  useDetach = UseDetach;
  thisServo = Servo(Channel);
  channel = Channel;
}

void Turnout::CheckState() {
  if (requiredState == currentState) {
    if ((millis() - millisAtLastMoveCompletion) > detachTimeout && thisServo.attached() && useDetach) {
      Serial.println("Detach timeout complete - detaching " + jMRIId+" on status "+String(thisServo.attached()));
      thisServo.detach();
    }
    return;
  }
  if (useSlowMotion && requiredPWMVal != currentPWMVal) {
    ProcessMoveSlow();
  }
}

void Turnout::ProcessMoveSlow() {
  bool moveIsComplete = false;
  unsigned long currentMillis = millis();

  if (requiredPWMVal != currentPWMVal && currentMillis - previousMillis >= delayTime)
  {
    //Serial.println("Turnout "+jMRIId+" current PWM "+String(currentPWMVal));
    previousMillis = currentMillis;
    if (requiredPWMVal > currentPWMVal)
    {
      int intendedPWMValue = currentPWMVal + stepSize;
      if (intendedPWMValue > requiredPWMVal)
      {
        intendedPWMValue = requiredPWMVal;
      }

      currentPWMVal = intendedPWMValue;
      //Serial.println("Intended PWM " + String(intendedPWMValue));
      if (usesDegrees) {
        //Serial.println("Degrees move to " + String(intendedPWMValue) + " " + jMRIId);
        thisServo.write(intendedPWMValue);
      } else {
        //Serial.println("Microseconds move to " + String(intendedPWMValue) + " " + jMRIId);
        thisServo.writeMicroseconds(intendedPWMValue);
      }

      if (requiredPWMVal == intendedPWMValue)
      {
        moveIsComplete = true;
      }
    }
    else// (requiredPosition < CurrentPWMValue[pin])
    {
      int intendedPWMValue = currentPWMVal - stepSize;
      if (intendedPWMValue < requiredPWMVal)
      {
        intendedPWMValue = requiredPWMVal;
      }

      currentPWMVal = intendedPWMValue;
      if (usesDegrees) {
        //Serial.println("Degrees move to " + String(intendedPWMValue) + " " + jMRIId);
        thisServo.write(intendedPWMValue);
      } else {
        //Serial.println("Microseconds move to " + String(intendedPWMValue)+" "+jMRIId);
        thisServo.writeMicroseconds(intendedPWMValue);
      }
      if (requiredPWMVal == intendedPWMValue)
      {
        moveIsComplete = true;
      }
    }
    //}
    if (moveIsComplete) {
      currentState = requiredState;
      currentPWMVal = requiredPWMVal;
      millisAtLastMoveCompletion = millis();
      Serial.println("Slow move complete current PWM " + String(currentPWMVal) + " " + jMRIId);
    }
  }
}

void Turnout::ProcessMove() {

}

void Turnout::Start() {
  //Serial.println("Attach attempt, value "+String(thisServo.attached()));
  if (!thisServo.attached()) {
    int channel = thisServo.attach(pin);
    //Serial.println("Attached " + jMRIId + " pin " + String(pin) + " result " + String(channel));
  }
}

void Turnout::Stop() {
  if (useDetach)
    thisServo.detach();
}
