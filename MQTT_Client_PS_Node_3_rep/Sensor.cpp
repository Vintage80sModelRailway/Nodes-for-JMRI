#include "Sensor.h"
#include <Arduino.h>





Sensor::Sensor(String SensorName, int InputPin, String JMRIID, bool IsInverted, uint8_t PinMode, int DebounceMS, int DebounceMode, int LastKnownValue)
  : _name(SensorName)
  , _pin(InputPin)
  ,  JMRIId(JMRIID)
  , _inverted(IsInverted)
  , _pinMode(PinMode)
  , _lastKnownValue(LastKnownValue)
{
  if (DebounceMS > -1 && DebounceMode > -1) {
    usingDebounce = true;
    debounceMS = DebounceMS;
    debounceMode = DebounceMode;
    Serial.println("Debounce activated on " + JMRIId);
    millisAtLastChange = millis();
  }
  else usingDebounce = false;
}


bool Sensor::UpdateSensor() {
  if (_pin == -1) return false;
  if (JMRIId == "") return false;

  bool hasChanged = false;
  int correctedVal = -1;
  if (_inverted) {
    correctedVal = !digitalRead(_pin);
  }  else {
    correctedVal = digitalRead(_pin);
  }

  if (!usingDebounce) {
    if (correctedVal == _lastKnownValue) {
      return false;
    }
    if (correctedVal == 0)
      State = "INACTIVE";
    else
      State = "ACTIVE";

    _lastKnownValue = correctedVal;
    //millisAtLastChange = millis();

    return true;
  }
  bool bypassDebounce = false;

  if (!inDebounce && _lastKnownValue == correctedVal)
  {
    return false;
  }

  if (!inDebounce) {
    Serial.println("Into debounce for "+_name+" current " + String(_lastKnownValue) + " new " + String(correctedVal));


    switch (debounceMode) {
      case 0: //to off only - we get here when debounce is on and pin reading has changed
        if (correctedVal == 0) {
          //sensor has gone to LOW - handle debounce
          millisAtLastChange = millis();
          inDebounce = true;
          debounceValue = correctedVal;
          //Serial.println("In debounce for off only - " + JMRIId);
        } else {
          bypassDebounce = true; //Bypass debounce if we're on off only and the value has switched to 1
          Serial.println("Bypassing for " + JMRIId + " corrected val " + String(correctedVal) + " debounce val " + String(debounceValue));
        }
        break;
      case 1:
        if (correctedVal == 1) {
          //sensor has gone to High - handle debounce
          millisAtLastChange = millis();
          inDebounce = true;
          debounceValue = correctedVal;
          //Serial.println("In debounce for on only - " + JMRIId);
        } else {
          bypassDebounce = true;
        }
        break;
      case 2:
        //sensor has changed handle debounce
        millisAtLastChange = millis();
        inDebounce = true;
        debounceValue = correctedVal;
        Serial.println("In debounce for both - " + JMRIId);
        break;
      default:
        Serial.println("Unhandled debounce setting for " + JMRIId);
        break;
    }
  }


  if (inDebounce) {
    if (correctedVal != debounceValue) {
      Serial.println("Flapping? debounce " + String(debounceValue) + " new " + String(correctedVal));
      inDebounce = false;
    }
  }
  unsigned long millisSinceLastChange = (millis() - millisAtLastChange);

  if ((inDebounce && millisSinceLastChange >= debounceMS) || bypassDebounce) {
    if (_lastKnownValue != correctedVal) {
      Serial.println("Safe to change now");
      _lastKnownValue = correctedVal;
      inDebounce = false;
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


  return false;
}





bool Sensor::UpdateSensorOld() {
  if (_pin == -1) return false;
  if (JMRIId == "") return false;

  bool hasChanged = false;
  int correctedVal = -1;
  if (_inverted) {
    correctedVal = !digitalRead(_pin);
  }  else {
    correctedVal = digitalRead(_pin);
  }

  if (correctedVal == _lastKnownValue && !usingDebounce) {
    return false;
  }

  if (!usingDebounce) {
    if (correctedVal == 0)
      State = "INACTIVE";
    else
      State = "ACTIVE";

    _lastKnownValue = correctedVal;
    //millisAtLastChange = millis();

    return true;
  }
  Serial.print(String(correctedVal) + " " + JMRIId + " ");


  bool bypassDebounce = false;
  if (inDebounce == false)
  {
    //Serial.println("Setting to in debounce " + JMRIId + " debounce mode " + String(debounceMode));
    switch (debounceMode) {
      case 0: //to off only - we get here when debounce is on and pin reading has changed
        if (correctedVal == 0) {
          //sensor has gone to LOW - handle debounce
          millisAtLastChange = millis();
          inDebounce = true;
          debounceValue = correctedVal;
          //Serial.println("In debounce for off only - " + JMRIId);
        } else {
          bypassDebounce = true; //Bypass debounce if we're on off only and the value has switched to 1
          //Serial.println("Bypassing for " + JMRIId + " corrected val " + String(correctedVal) + " debounce val " + String(debounceValue));
          debounceValue = correctedVal;
          _lastKnownValue = correctedVal;
        }
        break;
      case 1:
        if (correctedVal == 1) {
          //sensor has gone to High - handle debounce
          millisAtLastChange = millis();
          inDebounce = true;
          debounceValue = correctedVal;
          //Serial.println("In debounce for on only - " + JMRIId);
        } else {
          bypassDebounce = true;
          debounceValue = correctedVal;
          _lastKnownValue = correctedVal;
        }
        break;
      case 2:
        //sensor has changed handle debounce
        millisAtLastChange = millis();
        inDebounce = true;
        debounceValue = correctedVal;
        Serial.println("In debounce for both - " + JMRIId);
        break;
      default:
        Serial.println("Unhandled debounce setting for " + JMRIId);
        break;
    }
  }

  if (correctedVal != debounceValue && debounceValue >= 0 && debounceValue <= 1) {
    //Serial.println("---Debounce Changed reset debounce " + String(correctedVal) + " debounce val " + String(debounceValue));
    millisAtLastChange = millis();
    inDebounce = false;
    return false;
  }


  unsigned long millisSinceLastChange = (millis() - millisAtLastChange);
  //  Serial.println("Millis since last change " + String(millisSinceLastChange) + " " + JMRIId + " current val "
  //                 + String(_lastKnownValue) + " corrected val " + String(correctedVal) + " debounce val "
  //                 + String(debounceValue) + " bypass " + String(bypassDebounce));

  if ((millisSinceLastChange >= debounceMS && correctedVal == debounceValue
       && inDebounce == true && correctedVal != _lastKnownValue) || (bypassDebounce && correctedVal != _lastKnownValue))
  {
    Serial.println(JMRIId + " change confirmed " + String(_lastKnownValue) + " to " + String(correctedVal));
    if (correctedVal == 0)
      State = "INACTIVE";
    else
      State = "ACTIVE";

    _lastKnownValue = correctedVal;
    inDebounce = false;
    hasChanged = true;
  }
  else {
    //in debounce, time limit not met yet
    if (correctedVal != debounceValue) {
      inDebounce = false;
      millisAtLastChange = millis();
      Serial.println("Flicker? Debounce off for " + JMRIId);
    }
  }

  return hasChanged;
}

bool Sensor::UpdateShiftRegisterSensor(int val) {
  int correctedVal = -1;
  if (_inverted) {

    correctedVal = !val;
  }  else {
    correctedVal = val;
  }

  if (correctedVal == _lastKnownValue) return false;

  if (correctedVal == 0)
    State = "INACTIVE";
  else
    State = "ACTIVE";

  _lastKnownValue = correctedVal;

  return true;
}

void Sensor::SetPinMode() {
  pinMode(_pin, _pinMode);
}

String Sensor::GetSensorPublishTopic() {
  if (JMRIId.length() > 0) return "track/sensor/" + JMRIId;
  else return "";
}
