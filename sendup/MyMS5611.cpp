/* This is a specialization of the MS5611 class that permits a 
   user specified delay function. It also includes the Wire lib and
   instatiates the class with the Wire instance! We do not have to do this
   in the sketch anymore and can remove the inclusion of the Wire lib 
   from the sketch.
   
   This makes it possible to use the sketch (in different environments) 
   with the Wire lib as well as with the SoftWire lib (using different I2C pins!)
*/
#include "MyMS5611.h"

MyMS5611::MyMS5611(void) : MS5611(&Wire){
	setI2Caddr(I2C_MS5611);
}


unsigned long MyMS5611::read_adc(unsigned char aCMD)
{
  unsigned long value=0;
  unsigned long c=0;
  
  send_cmd(MS5xxx_CMD_ADC_CONV+aCMD); // start DAQ and conversion of ADC data
  switch (aCMD & 0x0f)
  {
    case MS5xxx_CMD_ADC_256 : delayMicroseconds(900);
    break;
    case MS5xxx_CMD_ADC_512 : _delay(3);
    break;
    case MS5xxx_CMD_ADC_1024: _delay(4);
    break;
    case MS5xxx_CMD_ADC_2048: _delay(6);
    break;
    case MS5xxx_CMD_ADC_4096: _delay(10);
    break;
  }
  send_cmd(MS5xxx_CMD_ADC_READ); // read out values
  _Wire->requestFrom(i2caddr, 3);
  c = _Wire->read();
  value = (c<<16);
  c = _Wire->read();
  value += (c<<8);
  c = _Wire->read();
  value += c;
  _Wire->endTransmission(true);
 
  return value;
}

void MyMS5611::setDelayFunction(void (*f) (long unsigned int))
{
  _delay = f;
}
