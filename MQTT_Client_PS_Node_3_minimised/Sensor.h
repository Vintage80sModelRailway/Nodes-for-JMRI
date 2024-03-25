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
    bool usingDebounce;
    int debounceMS;
    int debounceValue;
    int debounceMode;
    //Mode - 0 to off only, 1 to on only, 2 both

  public: 
    String JMRIId;
    String State;
    bool inDebounce;
    unsigned long millisAtLastChange;
    
    Sensor(String SensorName = "", int InputPin = -1, String JMRIID = "", bool IsInverted = false, uint8_t PinMode = INPUT, int DebounceMS = -1, int DebounceMode = -1, int LastKnownValue = -1);  
    bool UpdateSensor();
    bool UpdateSensorOld() ;
    void SetPinMode();
    String GetSensorPublishTopic();
    bool UpdateShiftRegisterSensor(int val);
};


#endif
