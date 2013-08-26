#include <avr/io.h>
#include <avr/pgmspace.h> 
 
#ifndef TESTFONT_H
#define TESTFONT_H


const unsigned char  testfont[] PROGMEM = {
  0xFF, 0x00, 0x00, 0x00, 0x00, // first column
  0x00, 0xFF, 0x00, 0x00, 0x00, // second column
  0x00, 0x00, 0xFF, 0x00, 0x00, // third column
  0x00, 0x00, 0x00, 0xFF, 0x00, // fourth column
  0x00, 0x00, 0x00, 0x00, 0xFF, // fifth column
  0x00, 0x00, 0x00, 0x00, 0x00, // space
  0x01, 0x01, 0x01, 0x01, 0x01, // first row
  0x02, 0x02, 0x02, 0x02, 0x02, // second row
  0x04, 0x04, 0x04, 0x04, 0x04, // third row
  0x08, 0x08, 0x08, 0x08, 0x08, // fourth row
  0x10, 0x10, 0x10, 0x10, 0x10, // fifth row
  0x20, 0x20, 0x20, 0x20, 0x20, // sixth  row
  0x40, 0x40, 0x40, 0x40, 0x40, // seventh row
  0x00, 0x00, 0x00, 0x00, 0x00, // space
};
#endif
