# upAndDown

This repository contains all the information to create a Geocache (or a similar tool) that uses a baromter as an altimeter for revealing a code displayed by a 5x7 dot matrix display when arriving at the top of a hill. It uses an ATtiny1634 MCU, the MS5611 barometer, and a dot matrix display TA07-11SEKWA (or something similar). 

You need a couple of other resources to build this gadget:

* the [ATtiny core by SpenceKonde](https://github.com/SpenceKonde/ATTinyCore) 
* the [MS5xxx library](https://github.com/Schm1tz1/arduino-ms5xxx) 
* the [ArduinoSort library](https://github.com/emilv/ArduinoSort)
* the [DotMatrix5x7 library](https://github.com/felias-fogg/DotMatrix5x7) by myself
* the [Vcc library](https://github.com/felias-fogg/Vcc) by myself
* the [TXOnlySerial library](https://github.com/felias-fogg/TXOnlySerial) by myself
* a PET preform board and an [accompanying library](https://github.com/felias-fogg/PETPreformBoard) by myself

In addition you need the following sketches to tune the OSCCAL and thte INTREF value of a particular chip:

*  a sketch to tune the [INTREF value](https://github.com/felias-fogg/intrefTune)
*  a sketch to tune the [OSCCAL value](https://github.com/felias-fogg/osccalTune) 

The directory contains the following sketches:

* *archive*: the old sketch (using an attiny84 and a 7-segment display)
* *baroTest*: a sketch to the the barometer breakout 
* *dotTest*: a sketch to test the dot matrix display
* *fontTest*: a sketch to interactively display characters
* *measureHeight*: a sketch that measures the relative height
* *sendup*: the target sketch
* *sendup0*: the same sketch, but tailored for the 0-version of the PET preform board

In addition, *BOM.md* contains the bill of materials.