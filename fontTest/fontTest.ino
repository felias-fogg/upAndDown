// -*- c++ -*-
#include <Arduino.h>
#include <stdio.h>
#include <string.h>
#include <DotMatrix5x7.h>
#include <PETPreformBoard.h>

#define SCROLLTIME 70
#define SHOWTIME 500

void setup(void)
{
  Dot5x7.begin(DMCOL1, DMCOL2, DMCOL3, DMCOL4, DMCOL5,
	       DMROW1, DMROW2, DMROW3, DMROW4, DMROW5, DMROW6, DMROW7);
  Dot5x7.setFramesPerSecond(42);
  Dot5x7.setUpsideDown(false);
#if defined(__AVR_ATtiny1634__)
  Serial.begin(2400);
#else
  Serial.begin(57600);
#endif
  Serial.println(F("Hello"));
}

void loop(void)
{
  long c;
  c = Serial.parseInt();
  if (c != 0) {
    Serial.print(c);
    Serial.print(F(" 0x"));
    Serial.println(c, HEX);
    Dot5x7.show(c);
  }
}
