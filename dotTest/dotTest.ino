// -*- c++ -*-
// Sketch to test dot matrix display on OPEN-V4 board
#include <DotMatrix5x7.h>
#define PPB_VERSION4 'A'
#include <PETPreformBoard.h>
#include "testfont.h"

void setup()
{
  Dot5x7.begin(DMCOL1, DMCOL2, DMCOL3, DMCOL4, DMCOL5,                  // column pins
	       DMROW1, DMROW2, DMROW3, DMROW4, DMROW5, DMROW6, DMROW7,  // row pins
	       LOW,                   // value when row pin is active (default value)
	       HIGH);                 // value when column pin is active (default value)
  Dot5x7.setFramesPerSecond(50);      // display 50 frames per second (default value)
}	 

void loop()
{

  Dot5x7.setFont(testfont);
  for (byte i=0; i < 14; i++) {
    Dot5x7.show(i);
    delay(500);
  }
  Dot5x7.setFont();
  Dot5x7.show('X'); // display 'X'
  delay(1000);      // for 2 seconds

  Dot5x7.clear();   // clear display
  delay(1000);      // show empty display for 1 second
  Dot5x7.scrollLeftString("Hello World!", 650, 50, 1); // display string, with 0.7sec time for showing each char
                                             // and 0.04sec pause between two steps of scrolling the char horizontally
  delay(2000);
}

