#include "Sensor.h"
#include <Arduino.h>

Sensor::Sensor(String SensorName, int InputPin, String JMRIID, bool IsInverted, uint8_t PinMode, int LastKnownValue, int DebounceMS)
  : _name(SensorName)
  , _pin(InputPin)
  ,  JMRIId(JMRIID)
  , _inverted(IsInverted)
  , _pinMode(PinMode)
  , _lastKnownValue(LastKnownValue)
  ,   inDebounce(false)
  ,   millisAtLastChange(0)
  ,   debounceMS(DebounceMS)
{

}

bool Sensor::UpdateSensor() {
  if (_pin == -1) return false;
  if (JMRIId == "") return false;

  int val = -1;
  if (_inverted) {
    val = !digitalRead(_pin);
    //if (_pin == 3) Serial.println("PIn3 val "+String(val)+" inverted");
  } else {
    val = digitalRead(_pin);
    //if (_pin == 3) Serial.println("PIn3 val "+String(val)+" not inverted");

  }
  //Serial.println("After val "+String(val) +" pin "+String(_pin));
  //if (_pin == 4) Serial.println("PIn3 val comparing "+String(val)+" to "+String(_lastKnownValue));
  if (val == _lastKnownValue) return false;

  Serial.println("After lkv");

  if (val == 0)
    State = "INACTIVE";
  else
    State = "ACTIVE";

  _lastKnownValue = val;

  return true;

}

bool Sensor::UpdateShiftRegisterSensor(int val) {
  bool hasChanged = false;
  int correctedVal = -1;
  if (JMRIId == "") return false;
  if (_inverted) {

    correctedVal = !val;
  }  else {
    correctedVal = val;
  }

  if (correctedVal == _lastKnownValue) {
    return false;
  }

  Serial.println(JMRIId + " change confirmed " + String(_lastKnownValue) + " to " + String(correctedVal));
  if (correctedVal == 0)
    State = "INACTIVE";
  else
    State = "ACTIVE";

  _lastKnownValue = correctedVal;
  hasChanged = true;
  publishRequired = true;
  
  return hasChanged;
}

void Sensor::SetPinMode() {
  pinMode(_pin, _pinMode);
}

String Sensor::GetSensorPublishTopic() {
  if (JMRIId.length() > 0) return "track/sensor/" + JMRIId;
  else return "";
}
