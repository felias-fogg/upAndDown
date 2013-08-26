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
* *baroTest*: a sketch to test the barometer breakout 
* *dotTest*: a sketch to test the dot matrix display
* *fontTest*: a sketch to interactively display characters
* *measureHeight*: a sketch that measures the relative height
* *sendup*: the target sketch
* *sendup0*: the same sketch, but tailored for the 0-version of the PET preform board based on the SoftI2CMaster library. The sketch is actually identical (using a soft link). The environment is different though. The accompanying MyMS5611 header and source files are quite different.

In addition, *BOM.md* contains the bill of materials (with actual prices and sources).

As a side remark, I would like to add that I found an undocumented silicon bug on the ATtiny1634 when developing this piece of software. When you configure PB3 (Arduino pin 14) as output, set it to high level, disable the watch dog counter, and put the MCU to sleep, then the MCU will draw 0.5 mA instead of 100nA! This is  roughly 5000 times more than promised! Well, on the other hand, one could have deduced this behavior from the Errata, which states that the port is pulled down when the WDT is inactive. Well, it took me some time to figure this out.