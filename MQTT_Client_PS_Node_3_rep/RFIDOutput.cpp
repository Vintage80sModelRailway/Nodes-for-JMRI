#include "RFIDOutput.h"
#include <Arduino.h>

RFIDOutput::RFIDOutput(Stream &SerialPort, String SensorName)
  : ser(SerialPort)
  , Name(SensorName)
{
  ValidTagRead = false; 
}

bool RFIDOutput::CheckForTag() {
  String thisId = "";
  ValidTagRead = false;
  if (ser.available())
  {
    while (ser.available())
    {
      char val = ser.read();
      //Serial.print(val);
      if (val == 0x02) {
        //Serial.println("Start");
        _bytesRead = 0;
        checksum = 0;
      }
      else if (val == 0x03 && _bytesRead == 12) {
        // ID completely read
        thisId = Tag;
        mfr = hex2dec(thisId.substring(0, 4));
        id  = hex2dec(thisId.substring(4, 10));
        chk = hex2dec(thisId.substring(10, 12));

        // Do checksum calculation
        int i2;
        for (int i = 0; i < 5; i++) {
          i2 = 2 * i;
          checksum ^= hex2dec(thisId.substring(i2, i2 + 2));
        }
        //Serial.println("checksum " + String(checksum) + " chk " + String(chk));
        if (checksum == chk) {
          ValidTagRead = true;
        }
      }
      else if (_bytesRead <= 12) {
        Tag[_bytesRead++] = val;
      }

      if (_bytesRead > 12) {
        Serial.println("BR "+String(_bytesRead));
        return false;
      }
      if (val == 0x03) {
        //Serial.println("End char " + String(_bytesRead));
      }
    }
  }
  return ValidTagRead;
}

long RFIDOutput::hex2dec(String hexCode) {
  char buf[19] = "";
  hexCode = "0x" + hexCode;
  hexCode.toCharArray(buf, 18);
  //Serial.print("Decoding ");
  //Serial.print(hexCode);
  //Serial.print(": ");
  //Serial.println(strtol(buf, NULL, 0));
  return strtol(buf, NULL, 0);
}
