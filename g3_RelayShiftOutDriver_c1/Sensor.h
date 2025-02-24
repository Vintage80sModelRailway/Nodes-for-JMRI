// Sensor.h
#ifndef Sensor_h
#define Sensor_h

#include <Arduino.h>

class Sensor {
  private:
    int _lastKnownValue;
    int _pin;
    String _name;
    bool _inverted;
    uint8_t _pinMode;
    int debounceMS;

  public:
    String JMRIId;
    String State;
    bool publishRequired;
    bool inDebounce;
    unsigned long millisAtLastChange;
    bool onHold;
    unsigned long millisAtOnHold;

    Sensor(String SensorName = "", int InputPin = -1, String JMRIID = "", bool IsInverted = false, uint8_t PinMode = INPUT, int LastKnownValue = -1, int DebounceMS = 300);
    bool UpdateSensor();
    void SetPinMode();
    String GetSensorPublishTopic();
    bool UpdateShiftRegisterSensor(int val);
};


#endif
