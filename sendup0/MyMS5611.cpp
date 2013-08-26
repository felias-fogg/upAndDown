/*
  MyMS5611.cpp - Library for accessing MS5611 sensors via I2C
  Copyright (c) 2012 Roman Schmitz, adapted by Bernhard Nebel (2021)
  
*/

#include "MyMS5611.h"

#define PPB_VERSION4 '0'
#include <PETPreformBoard.h>
#include <SoftWire.h>

SoftWire MyWire = SoftWire();

MyMS5611::MyMS5611() : i2caddr(I2C_MS5611) {
}

void MyMS5611::setI2Caddr(char aAddr) {
  i2caddr=aAddr;
}

byte MyMS5611::send_cmd(byte aCMD)
{
  MyWire.beginTransmission(i2caddr);
  MyWire.write(aCMD);
  uint8_t ret=MyWire.endTransmission(true);
  return ret;
}

uint8_t MyMS5611::connect() {
  MyWire.begin();
  MyWire.beginTransmission(i2caddr);
  uint8_t ret=MyWire.endTransmission(true);
  return ret;
}

void MyMS5611::ReadProm() {
  send_cmd(MS5611_CMD_RESET);
  delay(3);
  
  for(uint8_t i=0;i<8;i++) 
    {
      C[i]=0x0000;
      send_cmd(MS5611_CMD_PROM_RD+2*i);
      MyWire.requestFrom(i2caddr, 2);
      
      unsigned int c = MyWire.read();
      C[i] = (c << 8);
      c = MyWire.read();
      C[i] += c;
      MyWire.endTransmission(true);
    }
}

unsigned int MyMS5611::Calc_CRC4(unsigned char poly)
{
  int cnt;                   		// simple counter
  unsigned int n_rem;                 // CRC remainder
  unsigned int crc_read;              // original value of the CRC
  unsigned int l_pol = poly;
  unsigned char n_bit;
  
  l_pol = ( l_pol << 8 ) & 0xf000;	// shift bits and apply mask
  n_rem = 0x0000;
  
  crc_read = C[ 7 ];                  // save read RCR
  C[ 7 ] = ( 0xFF00 & ( C[ 7 ] ) );   // CRC byte is replaced by 0
  for ( cnt = 0; cnt < 16; cnt++ )    // operation is performed on bytes
    {// choose LSB or MSB
      if ( cnt % 2 == 1 ) n_rem ^= ( unsigned short ) ( ( C[ cnt >> 1 ] ) & 0x00FF );
      else n_rem ^= ( unsigned short ) ( ( C[ cnt >> 1 ] >> 8) & 0x00FF );
      
      for ( n_bit = 8; n_bit > 0; n_bit-- )
        {
	  if ( n_rem & ( 0x8000 ) )
            {
	      n_rem = ( n_rem << 1 ) ^ l_pol;
            }
	  else
            {
	      n_rem = ( n_rem << 1 );
            }
        }
    }
  C[ 7 ] = crc_read;
  n_rem = (0x000F & (n_rem >> 12)); // final 4-bit remainder is CRC code
  return n_rem;
}

unsigned int MyMS5611::Read_CRC4()
{
  unsigned int crc_read = ( 0x000F & ( C[ 7 ] ) );
  return ( crc_read );
}

unsigned int MyMS5611::Read_C( unsigned int index)
{
  unsigned int retval = 0;
  if ( ( index >= 0) && ( index <= 7 ) )
    retval = C[ index ];
  return retval;
}

unsigned long MyMS5611::read_adc(unsigned char aCMD)
{
  unsigned long value=0;
  unsigned long c=0;
  
  send_cmd(MS5611_CMD_ADC_CONV+aCMD); // start DAQ and conversion of ADC data
  switch (aCMD & 0x0f)
    {
    case MS5611_CMD_ADC_256 : delayMicroseconds(900);
      break;
    case MS5611_CMD_ADC_512 : _delay(3);
      break;
    case MS5611_CMD_ADC_1024: _delay(4);
      break;
    case MS5611_CMD_ADC_2048: _delay(6);
      break;
    case MS5611_CMD_ADC_4096: _delay(11);
      break;
    }
  send_cmd(MS5611_CMD_ADC_READ); // read out values
  MyWire.requestFrom(i2caddr, 3);
  c = MyWire.read();
  value = (c<<16);
  c = MyWire.read();
  value += (c<<8);
  c = MyWire.read();
  value += c;
  MyWire.endTransmission(true);
  
  return value;
}

void MyMS5611::Readout() {
  unsigned long D1=0, D2=0;
  
  double dT;
  double OFF;
  double SENS;
  
  D2=read_adc(MS5611_CMD_ADC_D2+MS5611_CMD_ADC_4096);
  D1=read_adc(MS5611_CMD_ADC_D1+MS5611_CMD_ADC_4096);
  
  // calculate 1st order pressure and temperature (MS5611 1st order algorithm)
  dT=D2-C[5]*pow(2,8);
  OFF=C[2]*pow(2,16)+dT*C[4]/pow(2,7);
  SENS=C[1]*pow(2,15)+dT*C[3]/pow(2,8);
  TEMP=(2000+(dT*C[6])/pow(2,23));
  P=(((D1*SENS)/pow(2,21)-OFF)/pow(2,15));
  
  // perform higher order corrections
  double T2=0., OFF2=0., SENS2=0.;
  if(TEMP<2000) {
    T2=dT*dT/pow(2,31);
    OFF2=5*(TEMP-2000)*(TEMP-2000)/pow(2,1);
    SENS2=5*(TEMP-2000)*(TEMP-2000)/pow(2,2);
    if(TEMP<-1500) {
      OFF2+=7*(TEMP+1500)*(TEMP+1500);
      SENS2+=11*(TEMP+1500)*(TEMP+1500)/pow(2,1);
    }
  }
  
  TEMP-=T2;
  OFF-=OFF2;
  SENS-=SENS2;
  P=(((D1*SENS)/pow(2,21)-OFF)/pow(2,15));	
}

double MyMS5611::GetTemp() {
  return TEMP;
}

double MyMS5611::GetPres() {
  return P;
}


void MyMS5611::setDelayFunction(void (*f) (long unsigned int))
{
  _delay = f;
}
