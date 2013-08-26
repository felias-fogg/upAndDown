// Sketch to test MS5611 on OpenV4 board

#define USEDOTMATRIX 
#include <stdio.h>
#define PPB_VERSION4 'A'
#include <PETPreformBoard.h>
#include <DotMatrix5x7.h>
#include "MyMS5611.h"

#define SHOWTIME 700
#define SCROLLTIME 50
#define SCROLLOFFSET 1

MyMS5611 sensor;

void setup()
{
  char hex[5];
#if defined(__AVR_ATtiny1634__)
  Serial.begin(9600);
  UCSR0B&=~(1<<RXEN0); // disable RX
#else
  Serial.begin(57600);
#endif
  Serial.println(F("baroTest setup..."));
  pinMode(POWER, OUTPUT);
  digitalWrite(POWER, HIGH); // enable MS5611
#ifdef USEDOTMATRIX
  Dot5x7.begin(DMCOL1, DMCOL2, DMCOL3, DMCOL4, DMCOL5,                  // column pins
	       DMROW1, DMROW2, DMROW3, DMROW4, DMROW5, DMROW6, DMROW7,  // row pins
	       LOW,                   // value when row pin is active (default value)
	       HIGH);                 // value when column pin is active (default value)
  Dot5x7.setFramesPerSecond(100);      // display 50 frames per second (default value)
  Dot5x7.scrollLeftString(F("Setup..."), SHOWTIME, SCROLLTIME, SCROLLOFFSET);
#endif
  delay(2);
  if(sensor.connect()>0) {
    Serial.println(F("Error connecting..."));
#ifdef USEDOTMATRIX
    Dot5x7.scrollLeftString(F("Error connecting..."), SHOWTIME, SCROLLTIME, SCROLLOFFSET);
#endif
    while (1);
  }
  sensor.ReadProm();
  Serial.print(F("PROM-Id: 0x"));
  Serial.println(sensor.Read_C(0), HEX);
#ifdef USEDOTMATRIX  
  sprintf(hex, "%x", sensor.Read_C(0));
  Dot5x7.scrollLeftString(F("PROM-Id: 0x"), SHOWTIME, SCROLLTIME, SCROLLOFFSET);
  Dot5x7.scrollLeftString(hex, SHOWTIME, SCROLLTIME, SCROLLOFFSET);
  delay(1000);
#endif
  if (sensor.Calc_CRC4() == sensor.Read_CRC4()) {
    Serial.println(F("CRC OK"));
#ifdef USEDOTMATRIX  
    Dot5x7.scrollLeftString(F("CRC OK"), SHOWTIME, SCROLLTIME, SCROLLOFFSET);
#endif
  } else {
    Serial.println(F("CRC failed"));
#ifdef USEDOTMATRIX  
    Dot5x7.scrollLeftString(F("CRC failed"), SHOWTIME, SCROLLTIME, SCROLLOFFSET);
#endif
    while (1);
  }
}	 

void loop()
{
  char str[60];
  int deg, decimaldeg;
  sensor.Readout();
  deg = (int)(sensor.GetTemp()/100);
  decimaldeg = (int)(sensor.GetTemp()/10) - (int)(sensor.GetTemp()/100)*10;
  sprintf(str, "Temp: %d.%d" "°" "C", deg, decimaldeg);
  Serial.println(str);
#ifdef USEDOTMATRIX    
  sprintf(str, "Temp: %d.%d" DEGREE "C", deg, decimaldeg);
  Dot5x7.scrollLeftString(str, SHOWTIME, SCROLLTIME, SCROLLOFFSET);
  delay(2000);
#endif
  sprintf(str, "Pressure: %d mBar", (int)(sensor.GetPres()/100));
  Serial.println(str);
#ifdef USEDOTMATRIX    
  Dot5x7.scrollLeftString(str, SHOWTIME, SCROLLTIME, SCROLLOFFSET);
  delay(5000);
#else
  delay(5000);
#endif
}
