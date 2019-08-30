
#include <Arduino.h>
#include <Wire.h>

class GyroRead
{
  private:
    float angle;
    uint8_t _addr;
    float rtvalue;
  public:
    void begin(int addr)
    {
      _addr = addr;
      Wire.begin();
      Wire.beginTransmission(_addr);
    }
    float getAngle()
    {
      if (Wire.requestFrom(_addr, 4) == 4)
      I2C_readAnything (angle);
      
      return angle;
    }
};
