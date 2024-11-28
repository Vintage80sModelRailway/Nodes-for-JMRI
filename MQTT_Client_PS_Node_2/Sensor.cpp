#include "Sensor.h"
#include <Arduino.h>

Sensor::Sensor(String SensorName, int InputPin, String JMRIID, bool IsInverted, uint8_t PinMode, int LastKnownValue)
  : _name(SensorName)
  , _pin(InputPin)
  ,  JMRIId(JMRIID)
  , _inverted(IsInverted)
  , _pinMode(PinMode)
  , _lastKnownValue(LastKnownValue == 0 ? "INACTIVE" : "ACTIVE")
{
  onHold = false;
  millisAtOnHold = millis();
}

bool Sensor::UpdateSensor() {
  if (_pin == -1) return false;
  if (JMRIId == "") return false;

  if (millis() - millisAtOnHold > 20000 && _lastKnownValue == 1 && onHold) {
    Serial.println("Timeout releasing hold for "+JMRIId);
    onHold = false; 
  }
  if (onHold && _lastKnownValue == 1) return false;

  int val = -1;
  if (_inverted) {
    val = !digitalRead(_pin);
  } else {
    val = digitalRead(_pin);
  }
  if (val == _lastKnownValue) return false;

  if (val == 0)
    State = "INACTIVE";
  else
    State = "ACTIVE";

  _lastKnownValue = val;

  return true;

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
  return "track/sensor/" + JMRIId;
}
