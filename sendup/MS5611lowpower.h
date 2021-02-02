/* This is a specialization of the MS5611 class that permits a 
   user specified delay function.
*/

#ifndef MS5611LOWPOWER_H
#define MS5611LOWPOWER_H

#include <Arduino.h>
#include <MS5611.h>

class MS5611lowpower : public MS5611
{
	
  public:
    MS5611lowpower(TwoWire *aWire);
    virtual unsigned long int read_adc(unsigned char aCMD);
    void setDelayFunction(void (*f) (long unsigned int));

 private:
    void (*_delay) (long unsigned int) = delay;
};

#endif
