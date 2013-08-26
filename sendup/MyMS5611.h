/* This is a specialization of the MS5611 class that permits a 
   user specified delay function.
*/

#ifndef MYMS5611_H
#define MYMS5611_H

#include <Arduino.h>
#include <MS5611.h>

class MyMS5611 : public MS5611
{
	
  public:
  MyMS5611(void);
    virtual unsigned long int read_adc(unsigned char aCMD);
    void setDelayFunction(void (*f) (long unsigned int));

 private:
    void (*_delay) (long unsigned int) = delay;
};

#endif
