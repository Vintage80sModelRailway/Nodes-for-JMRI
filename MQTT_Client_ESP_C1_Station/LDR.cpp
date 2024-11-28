#include "LDR.h"
#include <Arduino.h>

LDR::LDR(String SensorName, int InputPin, String JMRIID, int LightDarkThreshold, int DebounceOnMS, int DebounceOffMS)
  : sensorName(SensorName)
  , pin(InputPin)
  ,  JMRIId(JMRIID)
  , lightDarkThreshold(LightDarkThreshold)
{
  lastKnownState = -1;
  testMode = false;
  if (DebounceOnMS > -1) {
    usingOnDebounce = true;
    debounceOnMS = DebounceOnMS;
    Serial.println("Debounce on activated on " + JMRIId);
    millisAtLastChange = millis();
  }
  else usingOnDebounce = false;

  if (DebounceOffMS > -1) {
    usingOffDebounce = true;
    debounceOffMS = DebounceOffMS;
    Serial.println("Debounce off activated on " + JMRIId);
    millisAtLastChange = millis();
  }
  else usingOffDebounce = false;
}

bool LDR::UpdateSensor() {
  if (pin == -1) return false;
  if (JMRIId == "") return false;

  bool hasChanged = false;
  int sensorVal = analogRead(pin);
  int correctedVal = 1;

  if (sensorVal >= lightDarkThreshold)
    correctedVal = 0;

  if (!usingOnDebounce && !usingOffDebounce) {

    if (correctedVal == lastKnownState) {
      return false;
    }
    if (correctedVal == 0)
    {
      State = "INACTIVE";
    }
    else
    {
      State = "ACTIVE";
    }

    lastKnownState = correctedVal;

    return true;
  }
  bool bypassDebounce = false;

  if (!inDebounce && lastKnownState == correctedVal)
  {
    return false;
  }

  if (!inDebounce && (usingOffDebounce || usingOnDebounce)) {
    //Serial.println("Into debounce current " + String(lastKnownState) + " new " + String(correctedVal));
    //sensor has gone to off - handle debounce
    millisAtLastChange = millis();
    inDebounce = true;
    debounceValue = correctedVal;
    //Serial.println("In debounce - " + JMRIId);
  }

  if (inDebounce) {
    unsigned long millisSinceLastChange = (millis() - millisAtLastChange);
    if (correctedVal != debounceValue) {
      //Serial.println("Flapping ? debounce " + String(debounceValue) + "("+String(sensorVal) + ") new " + String(correctedVal)+ " - T - "+String(lightDarkThreshold));
      inDebounce = false;
    }
    else {
      if ((correctedVal ==  true && debounceValue == true && millisSinceLastChange >= debounceOnMS)
          || (correctedVal == false && debounceValue == false && millisSinceLastChange >= debounceOffMS) ) {
        if (lastKnownState != correctedVal) {
          Serial.println("Safe to change now " + String(correctedVal) + " - debounce millis - " + String(millisSinceLastChange));
          lastKnownState = correctedVal;
          inDebounce = false;
          analogVal = sensorVal;
          if (correctedVal == 0)
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
