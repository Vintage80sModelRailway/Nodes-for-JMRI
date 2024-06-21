#include "LDR.h"
#include <Arduino.h>

LDR::LDR(String SensorName, int InputPin, String JMRIID, int LightDarkThreshold, int DebounceMS, int DebounceMode)
  : sensorName(SensorName)
  , pin(InputPin)
  ,  JMRIId(JMRIID)
  , lightDarkThreshold(LightDarkThreshold)
{
  lastKnownState = -1;
  testMode = false;
  if (DebounceMS > -1 && DebounceMode > -1) {
    usingDebounce = true;
    debounceMS = DebounceMS;
    debounceMode = DebounceMode;
    Serial.println("Debounce activated on " + JMRIId);
    millisAtLastChange = millis();
  }
  else usingDebounce = false;
}

bool LDR::UpdateSensor() {
  if (pin == -1) return false;
  if (JMRIId == "") return false;

  bool hasChanged = false;
  int sensorVal = analogRead(pin);
  int sensorState = 1;

  if (sensorVal >= lightDarkThreshold)
    sensorState = 0;
//  if (JMRIId == "5021") {
//    Serial.println("Val "+String(sensorVal)+" state "+String(sensorState));
//  }

  if (!usingDebounce) {

    if (sensorState == lastKnownState) {
      return false;
    }
    if (sensorState == 0)
    {
      State = "INACTIVE";
    }
    else
    {
      State = "ACTIVE";
    }

    lastKnownState = sensorState;
    Serial.println("Val "+String(sensorVal)+" state "+String(sensorState));
    return true;
  }
  bool bypassDebounce = false;

  if (!inDebounce && lastKnownState == sensorState)
  {
    return false;
  }

  if (!inDebounce) {
    //Serial.println("Into debounce current " + String(lastKnownState) + " new " + String(sensorState));

    switch (debounceMode) {
      case 0: //to off only - we get here when debounce is on and pin reading has changed
        if (sensorState == false) {
          //sensor has gone to DARK - handle debounce
          millisAtLastChange = millis();
          inDebounce = true;
          debounceValue = sensorState;
          //Serial.println("In debounce for off only - " + JMRIId);
        } else {
          bypassDebounce = true; //Bypass debounce if we're on off only and the value has switched to 1
          //Serial.println("Bypassing for " + JMRIId + " corrected val " + String(sensorState) + " debounce val " + String(debounceValue));
        }
        break;
      case 1:
        if (sensorState == 1) {
          //sensor has gone to High - handle debounce
          millisAtLastChange = millis();
          inDebounce = true;
          debounceValue = sensorState;
          //Serial.println("In debounce for on only - " + JMRIId);
        } else {
          bypassDebounce = true;
        }
        break;
      case 2:
        //sensor has changed handle debounce
        millisAtLastChange = millis();
        inDebounce = true;
        debounceValue = sensorState;
        //Serial.println("In debounce for both - " + JMRIId);
        break;
      default:
        Serial.println("Unhandled debounce setting for " + JMRIId);
        break;
    }
  }


  if (inDebounce) {
    if (sensorState != debounceValue) {
      //Serial.println("Flapping ? debounce " + String(debounceValue) + " new " + String(sensorState));
      inDebounce = false;
    }
  }
  unsigned long millisSinceLastChange = (millis() - millisAtLastChange);

  if ((inDebounce && millisSinceLastChange >= debounceMS) || bypassDebounce) {
    if (lastKnownState != sensorState) {
      Serial.println("Safe to change now "+String(sensorVal));
      analogVal = sensorVal;
      lastKnownState = sensorState;
      inDebounce = false;
      if (sensorState == 0)
        State = "INACTIVE";
      else
        State = "ACTIVE";
      return true;
    }

    else {
      inDebounce = false;
      return false;
    }
  }


  return false;
}

int LDR::GetRaw() {
  return analogRead(pin);
}

String LDR::GetSensorPublishTopic() {
  return "track/sensor/" + JMRIId;
}
