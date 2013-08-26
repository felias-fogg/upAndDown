/* A sketch that has the mission of sending people up for a couple of meters
   and then bringing them down again. It uses the MS5611 altimeter/barometer.

   Developed on an Arduino Pro Mini, deployed on a ATtiny84, meanwhile ATtiny1634.
   PCB is the Open-V4a board that fits together with a 3.6V/2400mA lithium
   battery into a preform tube (13cm).

   The sketch is published under the MIT license (see included license.txt)

   ATtiny Fusebits: Divide clock by 8, internal OSC., Brown-out disabled, preserve EEPROM
   Arduino board: ATtiny 1634 / 1MHz

   Special modes:
   - Press button 5 times before the first character is scrolled away: RESET and recalibrate height
   - Press button when 2nd, 3rd, 4th, 5th, and 6th character is shown: TRANSPORT mode
     This mode can only be be exited by pressing the button once and then three times after the middle dot blinked.
   - Press button 5 times when the sixth character is shown: Display statistics.
     Exit this mode by pressing the button. When the button is pressed while "PRESS TO CLEAR" is shown, memory
     is cleared.
*/

#define VERSION "5.0.3"
//#define DEBUG

/* Version 0.9 - first version out in the wild
 * Version 1.0 - switched to SoftI2CMaster (faster and less memory consumption)
 * Version 2.0 - improved power saving
 * Version 2.1 - simplified, less states, output only when key pressed, 
 *               no scpecial descend mode, sleep only after 5 minutes 
 *               after inactivity
 * Version 2.2 - removed bug introduced by using idleDelay - too inaccurate!
 * Version 2.3 - restricted pressure measuring & blinking - only every 3rd second
 * Version 2.4 - added "Laufen" to the message
 * Version 2.5 - parameter + message are now in EEPROM - and are initialized by defaults
 * Version 2.6 - height is now a global var in order to avoid unintialized values 
 *                in the loop; we also now have a MAXCHANGE constant that leads to 
 *		 a longer measurement if the height change over a 3 second period 
 *		 is too large
 *		 -> Version to go into the wild after 5.10.2013
 * Version 2.7 - switched SCL and SDA to conform with wiring description
 *               EEPROM parameters are now at the beginning of the EEPROM
 *               after power-up, the summit message is displayed
 * Version 2.8 - changed internally from meters do decimeters for:
 *               - meters_up -> decimeters_up
 *               - epsilon
 *               - height
 *               - lastheight
 *               The display is still in meters!
 *               Added IMPERIAL as a compile time option which will
 *               enable to display height in FEET instead of METERS
 *               21.01.2014
 * Version 2.9 - changed errcode to char
 *               new error: OUTLIER_ERROR
 *               changed retry limit in measure to 4
 *               removed the MAXCHANGE clause (when 4 meter change, try again)
 *               reject measurement if the max-min value is higher than 0.6 hPa = 5 meters
 *               we check whether error code is set and then go into error state
 *               introduced compile time SETTINGS 
 *               and included timeout there!
 * Version 3.0 - added  __attribute__((used)) to wdt_init
 * Version 3.1 - added I2C_ERROR when i2c initialization failed
 *             - added RESET: Press key five times when GO is displayed
 * Version 3.2 (19.4.2019)
 *             - increased SCHOENBERG epsilon by 2
 *             - allow recovery when OUTLIER error
 *             - ignoring bad measurements (but increasing errcnt!)
 *             - RESET can be activated when key is pressed five times during GO or number.
 *             - added ADDRESS constant to SETTING-specific settings
 * Version 4.0
 *             - added ATtiny167 as one of the possibilities, which now allows to power-cycle the MS5611
 *             - not yet fully integrated!
 *             - resetAlti does no internal recovery, this is now handled externally.
 *             - For ATtiny84, we added some recovery mechanism, when SDA is stuck. I am not sure, if 
 *               it helps all the time!
 *            
 * Version 4.1 (24.5.2020)
 *            - When volt < LOWBATT_ERR then we check again and reset the error bit EEPROM if volt is above limit.
 *              This means, it does not get stuck when we have one wrong measurement, as it seemed to have happened here!
 *
 * Version 5.0.1 (1.2.2021)
 *           - Now we have a dot matrix display and attiny1634, which gives us more pins, more flash memory and more
 *             RAM. We therefore use now libraries for driving the display and reading the sensor. And it will be
 *             much easier for the user to understand us, because of the dot matrix display.
 *           - Use one pin to control powering the MS5611 chip, allowing recovery from I2C stuck conditions
 *           - Use WDT in order to detect I2C stuck conditions.
 *           - Added statistics mode
 *           - All in all, pretty much of the program has been rewritten.
 *
 * Version 5.0.2 (3.2.2021)
 *           - switched to TXOnlySerial for serial debug communication (saves some space and appears to work more consistently)
 * 
 * Version 5.0.3 (5.2.2021)
 *          - modified the code so that there can be one sketch that is based on different MyMS5611 
 *            include files which cater for the original OPEN-V4 board as well as for the new 
 *            OPEN-V4A board! You have two different folders, but they have the identical 
 *            sketch (using soft links!)
 *          - found a problem in the Tiny1634: if you go in power-down mode and leave the PB3 port 
 *            pin as an output in the high state, the chip will draw 0.4mA instead of 100nA! 
 *            Simply setting the output to low is good enough to
 *            get around this problem.
 * Version 5.0.4 (23.4.2021)
 *          - adjusted meters to 50 (instead of 60) for the Dagstuhl setting (this is what the topo map says)
 *          - changed framerate to 50 (from 42) to elimate the flicker
 * Version 5.0.5 (24.1.2022)
 *          - added config folder with configuration files for single geocaches
 */

#include "MyMS5611.h"

#include <avr/wdt.h>
#include <avr/sleep.h>
#include <avr/pgmspace.h>
#include <avr/boot.h>
#include <avr/power.h>
#include <PETPreformBoard.h>
#include <EEPROM.h>
#include <DotMatrix5x7.h>
#include <Vcc.h>
#include <ArduinoSort.h>
#ifdef DEBUG
#include <TXOnlySerial.h>
#endif

// Fuse settings:
#define LOW_FUSE 0x62 // divide clock by 8, use internal oscillator
#define HIGH_FUSE 0xD7 // SPI enabled, EEPROM preservation
#define EXT_FUSE 0xFF

// some timing constants
#define SCROLLMS 50
#define SHOWMS 650
#define FRAMERATE 50 // frames per second for the dot matrix display

// level for entering special modes
#ifdef DEBUG
#define MAXLEVEL 2
#else
#define MAXLEVEL 5
#endif

// repetition
#ifndef DEBUG
#define ERROR_REPEAT 4
#else
#define ERROR_REPEAT 2
#endif
#define GREETING_REPEAT 4
#define EXPLANATION_REPEAT 2
#define LONG_REPEAT 2

// measurement
#ifdef DEBUG
#define MAXSAMPLE 30 // we take this many samples
#define THROWAWAY 5 // this many lowest and highest samples will be thrown away (in order to ignore outliers)
#else
#define MAXSAMPLE 100 // we take this many samples
#define THROWAWAY 20 // this many lowest and highest samples will be thrown away (in order to ignore outliers)
#endif
#define MAXTEMP 9000.0 // 90 degrees Celsius
#define MINTEMP -5000.00 // -50 degrees Celsius
#define MAXPRES 120000.0 // 1200 mBar
#define MINPRES 85000.0 // 850 mBar 

// possible settings
#define HOME_SETTING 1
#define SCHOENBERG_SETTING 2
#define DAGSTUHL_SETTING 3

#define SETTING DAGSTUHL_SETTING

#if SETTING==HOME_SETTING
#define START_HEIGHT_METERS 230 // start height in meters
#define METERS_UP 6 // meters you have to walk up
#define EPSILON 1  // potential error (in meters)
#define EXPLAIN_STR "Bring mich hoch!"
#define SUMMIT_STR "Angekommen!" // message displayed on summit
#define MAXWAKEUP_NOBUMP 60 // if there is no bump, we go to sleep again 
#define MAXWAKEUP_TOTAL 120 // 2 minutes for wakeup & start climbing
#define MINQUIET 4*120UL // 2 minutes no bump means we are back in the box
#define I2C_ADDRESS 0x77 // depends, of course!
#elif SETTING==DAGSTUHL_SETTING
#include "config/dagstuhl.h"
#elif SETTING==SCHOENBERG_SETTING
#include "config/schoenberg.h"
#else
#error "Undefined setting"
#endif

// logic levels for switching the MS5611 on and off
#define MSACTIVE HIGH // logic level if we use the PIN directly to source power
#define MSINACTIVE LOW // logic level if we use the PIN directly to source power

// imperial or metric!
#ifdef IMPERIAL
#define CONV_MEASURE_FAC 3.048 // 3.048 decimeters are one foot
#else
#define CONV_MEASURE_FAC 10.0  // 10 decimeters are one meter
#endif

#define MAXERRCNT 5 // after that many errors we give up
#define BATT_WEAK 2800 // warning that battery is low
#define BATT_EMPTY 2600 // battery almost empty, give error!

/* Debug macros */
#ifdef DEBUG
#if defined(__AVR_ATtiny1634__)
#define DEBINIT()  mySerial.begin(9600)
#define DEBPR(str) mySerial.print(str)
#define DEBLN(str) mySerial.println(str)
#else
#define DEBINIT() Serial.begin(9600)
#define DEBPR(str) Serial.print(str)
#define DEBLN(str) Serial.println(str)
#endif
#else
#define DEBPR(str)
#define DEBLN(str)
#define DEBINIT()
#endif

/* error codes */
#define NO_ERROR -1
#define TWI_ERROR 0
#define PROMID_ERROR 1
#define CRC_ERROR 2
#define TEMP_OUTLIER_ERROR 3
#define PRESS_OUTLIER_ERROR 4
#define FATAL_ERROR WAKEUP_ERROR // from here on, errors are not recoverable
#define WAKEUP_ERROR 5
#define STATE_ERROR 6
#define BATT_ERROR 7
#define BROWN_OUT_ERROR 8
#define UNDEF_RESTART_ERROR 9
#define OSCCAL_ERROR 10
#define INTREF_ERROR 11
#define FUSE_ERROR 12
#define ERROR_NUM 13

/* messages */
#ifndef ENGLISH
const char PROGMEM recalib[] = "Neustart!";
const char PROGMEM greeting[] = "Dr" uUML "cke meinen Knopf!";
const char PROGMEM explain[] = EXPLAIN_STR;
const char PROGMEM message1[] = "Bring mich noch";
const char PROGMEM shortmessage1[] = "Noch";
const char PROGMEM message2[] = "Meter h" oUML "her.";
const char PROGMEM message3[] = "  ... Angekommen!";
const char PROGMEM thanks[] = "Vielen Dank!";
const char PROGMEM reveal[] = SUMMIT_STR;
const char PROGMEM battweak[] = "Die Batterie ist fast leer!";
const char PROGMEM bye[] = "Tsch" uUML "ss";
const char PROGMEM transport[] = "Transportmodus";
const char PROGMEM twi_error[] = "TWI-Fehler";
const char PROGMEM promid_error[] = "PROM-ID-Fehler";
const char PROGMEM crc_error[] = "CRC-Fehler";
const char PROGMEM temp_outlier_error[] = "Temperatur-Fehler";
const char PROGMEM press_outlier_error[] = "Druck-Fehler";
const char PROGMEM wakeup_error[] = "Aktivierungs-Fehler";
const char PROGMEM state_error[] = "Zustands-Fehler";
const char PROGMEM batt_error[] = "Batterie ist leer";
const char PROGMEM brown_out_error[] = "Brown-Out-Fehler";
const char PROGMEM undef_reason_error[] = "Undef-Restart-Fehler";
const char PROGMEM osccal_error[] = "OSCCAL-Fehler";
const char PROGMEM intref_error[] = "INTREF-Fehler";
const char PROGMEM fuse_error[] = "Fuse-Fehler";
#else // ENGLISH
const char PROGMEM recalib[] = "Restart!";
const char PROGMEM greeting[] = "Press my button"; 
const char PROGMEM explain[] = EXPLAIN_STR;
const char PROGMEM message1[] = "Take me another";
const char PROGMEM shortmessage1[] = "Another";
const char PROGMEM message2[] = "feet up.";
const char PROGMEM message3[] = "  ... You did it!";
const char PROGMEM thanks[] = "Thanks!";
const char PROGMEM reveal[] = SUMMIT_STR;
const char PROGMEM battweak[] = "Battery is weak!";
const char PROGMEM bye[] = "Goodbye!";
const char PROGMEM transport[] = "Transport mode";
const char PROGMEM twi_error[] = "TWI error";
const char PROGMEM promid_error[] = "PROM-id error";
const char PROGMEM crc_error[] = "CRC error";
const char PROGMEM temp_outlier_error[] = "Temperature error";
const char PROGMEM press_outlier_error[] = "Pressure error";
const char PROGMEM wakeup_error[] = "Wakeup error";
const char PROGMEM state_error[] = "State error";
const char PROGMEM batt_error[] = "Battery empty";
const char PROGMEM brown_out_error[] = "Brown out error";
const char PROGMEM undef_reason_error[] = "Undefinded restart error";
const char PROGMEM osccal_error[] = "OSCCAL error";
const char PROGMEM intref_error[] = "INTREF error";
const char PROGMEM fuse_error[] = "Fuse error";
#endif

const char* const PROGMEM error_message[ERROR_NUM] = { twi_error, promid_error, crc_error, temp_outlier_error, press_outlier_error,
						       wakeup_error, state_error, batt_error, brown_out_error, undef_reason_error,
						       osccal_error, intref_error, fuse_error };
/* program episode */
#define SETUP_EPISODE 0
#define MAIN_EPISODE 1
#define SLEEPING_EPISODE 2

/* states */
#define NO_STATE 0
#define BYE_STATE 1
#define SLEEP_STATE 2
#define WAKEUP_STATE 3
#define EXPLAIN_STATE 4
#define CLIMB_STATE 5
#define THANKS_STATE 6
#define SUMMIT_STATE 7
#define ERROR_STATE 8
#define TRANSPORT_STATE 9
#define DEEPSLEEP_STATE 10
#define RESET_STATE 11
#define STATISTICS_STATE 12

/* restart reasons */
#define NO_REASON 0
#define POR_REASON 1
#define EXTR_REASON 2
#define WDR_SETUP_REASON 3
#define WDR_MAIN_REASON 4
#define WDR_SLEEP_REASON 5
#define BOR_REASON 6

/***** super global variables (surviving resets) *****/
volatile byte laststate __attribute__ ((section (".noinit"))); // last state
volatile byte episode __attribute__ ((section (".noinit"))); // program episode
volatile unsigned int quartersecs __attribute__ ((section (".noinit"))); // quarter of seconds counted by wdt
volatile unsigned long int wakemillis __attribute__ ((section (".noinit"))); // remember millis
volatile bool wakeenable __attribute__ ((section (".noinit"))); // mark period when one can exit transport mode
volatile bool clearstat  __attribute__ ((section (".noinit"))); // mark period when one can clear statistics 
unsigned long int accumillis __attribute__ ((section (".noinit"))); // accumulated millis
byte errcnt  __attribute__ ((section (".noinit"))); // global error counter
int errcode  __attribute__ ((section (".noinit"))); // global error code
byte mcusr __attribute__ ((section (".noinit"))); // mcu state at start
volatile byte state __attribute__ ((section (".noinit"))); // state var
unsigned int lastbump __attribute__ ((section (".noinit"))); // last bump in quarter seconds
float refPress __attribute__ ((section (".noinit"))); // reference pressure for 0 level
unsigned int charcnt __attribute__ ((section (".noinit"))); // count displayed chars (for maintenance mode, deep sleep mode, and reset)
int resetlevel  __attribute__ ((section (".noinit"))); // after a few resets, recalibration is triggered
int transportlevel __attribute__ ((section (".noinit"))); // after a few resets, we change into transport mode
int statlevel __attribute__ ((section (".noinit"))); // after a few resets, stats are displayed
int wakelevel __attribute__ ((section (".noinit"))); // after a few resets, we awake from transport mode
float temperature __attribute__ ((section (".noinit"))); // needed for converting pressure into height
float cf __attribute__ ((section (".noinit"))); // conversion factor for converting Pascals into decimeters
bool summit_reached __attribute__ ((section (".noinit"))); // success!
bool climbed __attribute__ ((section (".noinit"))); // tried!
byte longmess __attribute__ ((section (".noinit"))); // number of times long message displayed

#define EE_PARA (EE_INTREF-1) // from Vcc.h
struct {
  byte osccal;
  int intref;
} para  __attribute__ ((section (".noinit")));

#define EE_STAT 0
struct  {
  unsigned long totalsecs;
  unsigned long wakesecs;
  unsigned int totalwakeups;
  unsigned int totalclimbs;
  unsigned int totalsummits;
  unsigned int totalstat;
  int errcnt[ERROR_NUM];
} stat   __attribute__ ((section (".noinit")));


/***** global vars ****/
volatile bool bumped = false;
byte wdtcnt = 0;
int volt;
uint8_t pressed = false;
float sample[MAXSAMPLE];
float currPress = 0.0;
char numbuf[16]; // for converting integers to strings
int samplecnt = -1; // start measuring when set to 0
int remain; // remaining units to walk up

MyMS5611 sensor;

#if defined(DEBUG) && defined(__AVR_ATtiny1634__)
TXOnlySerial mySerial(0);
#endif

/**************************** IRQ and RESET routines *************************************************/

// mcusr contains the reason for the restart (if we do not use a bootloader as on an Pro mini!)
// wdt is disabled initially in order to avoid wdt loops!
void wdt_init(void) __attribute__((naked)) __attribute__((section(".init3"))) __attribute__((used));
void wdt_init(void)
{
  mcusr = MCUSR;
  MCUSR = 0;
  wdt_disable();
  return;
}

// fake the MCUSR on a Pro Mini
inline void fake_mcusr(void)
{
#ifdef __AVR_ATmega328P__
  mcusr = 0;
  pinMode(SIM_RESET, INPUT_PULLUP); // simulate external reset
  pinMode(SIM_POR, INPUT_PULLUP); // simlate power up reset
  if (digitalRead(SIM_RESET) == LOW) mcusr = _BV(EXTRF);
  if (digitalRead(SIM_POR) == LOW) mcusr = _BV(PORF);
  if (mcusr == 0) mcusr = _BV(WDRF); // must be WDT reset!
#elif defined(__AVR_ATtiny1634__)
  // nothing to do
#else
  #error "MCU type is not supported"
#endif
}
  
// WDT interrupt: watch the execution of setup and after that count quarter seconds
ISR(WDT_vect)
{
  wakemillis = millis(); // remember millis
  if (episode == SETUP_EPISODE) { // wdt during setup!
    wdt_enable(WDTO_15MS);
  } else {
    quartersecs++;
    if (wdtcnt++ > 3) { // stuck during executing main!
      wdt_enable(WDTO_15MS);
    } else WDTCSR |= (1<<WDIE); // re-enable watchdog interrupt
  }
}

inline void enable_wdt_setup(void)
{
  DEBLN(F("enable_wdt_setup"));
#ifdef DEBUG
  wdt_enable(WDTO_4S);
#else
  wdt_enable(WDTO_250MS);
#endif
}

void enable_wdt_counting(void)
{
  DEBLN(F("enable_wdt_counting"));
  wdt_reset();
  wdtcnt = 0; // the wdt counter that helps against getting stuck
  wdt_enable(WDTO_250MS);
  WDTCSR |= (1<<WDIE); // enable watchdog interrupt (instead of reset!)
}

inline void disable_wdt_counting(void)
{
  episode = SLEEPING_EPISODE; // so an early RESET leads to a restart
  wdt_reset();
#ifdef __AVR_ATtiny1634__
  CCP = 0xD8;
#endif
  WDTCSR = 0; // Important! Otherwise WDT continues to be triggered
  wdtcnt = 0;
}

// reset wdt counter, otherwise a wdt reset is triggered after wdtcnt reaches 4
inline void wdtcnt_reset(void)
{
  wdtcnt = 0;
}

// read quarter seconds 
unsigned int getQuarterSeconds(void)
{
  unsigned int res;
  cli();
  res = quartersecs;
  sei();
  return res;
}


// vibration switch
ISR(VIB_PCINT_vect)
{
  bumped = true;
}


void enableVibIRQ(void) 
{
  VIB_IF |= VIB_PCIF; // clear interrupt flag before enabling interrupts
  VIB_PCICR |= _BV(VIB_PCIE); // mark PCI enable in Pin Change Interrupt Control Reg
  VIB_MASK  |= _BV(VIB_PCINT); // enable pin in PCI mask register
}

void disableVibIRQ(void) 
{
  VIB_MASK &= ~_BV(VIB_PCINT); // disable pin in PCI mask register
}




/**************************** Setup routines *************************************************/

/* There are many possibilities for starting the setup routine:
   - Power on reset -> initialize everything and then start with wakeup
   - Watchdog reset:
   -   while episode=SETUP -> stuck while in setup, power cycle MS5611 and try again if < 5 times
   -   while episode=MAIN  -> stuck while in main,  power cycle MS5611 and try again if < 5 times
   -   while episode=SLEEPING -> woke up, set state=WAKEUP
   - External reset -> User requests a response (or we wake up from a sleep)
   - Brown-out reset: should not happen as we have not activated it.
 */
void setup(void) {
  byte reason;
  
  DEBINIT();
  DEBLN(F("\nsendup V" VERSION));
  DEBLN(F("Setup..."));
  DEBPR(F("state="));
  DEBLN(state);
 
#if 0
  DEBPR(F("episode="));
  DEBLN(episode);
  DEBPR(F("laststate="));
  DEBLN(laststate);
  DEBPR(F("quartersecs="));
  DEBLN(quartersecs);
  DEBPR(F("wakemillis="));
  DEBLN(wakemillis);
  DEBPR(F("accumillis="));
  DEBLN(accumillis);
  DEBPR(F("errcnt="));
  DEBLN(errcnt);
  DEBPR(F("errcode="));
  DEBLN(errcode);
  DEBPR(F("statlevel="));
  DEBLN(statlevel);
#endif

  fake_mcusr(); // only necessary when running with bootloader
  reason = NO_REASON;
  if (mcusr & _BV(PORF)) reason = POR_REASON;
  else if (mcusr & _BV(EXTRF)) reason = EXTR_REASON;
  else if (mcusr & _BV(WDRF)) reason = WDR_SETUP_REASON + episode;
  else if (mcusr & _BV(BORF)) reason = BOR_REASON;
  DEBPR(F("Restart reason: "));
  DEBLN(reason);
  
  episode = SETUP_EPISODE;
  enable_wdt_setup(); // watch dog reset after 0.25 seconds 

  switch (reason) {
  case POR_REASON:
  case WDR_SLEEP_REASON:
    init_all_vars();
    state = WAKEUP_STATE;
    break;
  case BOR_REASON:
    rec_error(BROWN_OUT_ERROR);
    break;
  case WDR_SETUP_REASON:
  case WDR_MAIN_REASON:
    rec_error(TWI_ERROR);
    break;
  case EXTR_REASON:
    if (state == SLEEP_STATE || state == NO_STATE) {
      init_all_vars();
      state = WAKEUP_STATE;
    }
    pressed = true;
    break;
  default:
    rec_error(UNDEF_RESTART_ERROR);
    state = ERROR_STATE;
    break;
  }
  DEBLN(F("  Vars initialized"));

  accumillis += wakemillis;
  wakemillis = millis();

  if (pressed) { // check for repeated presses
    if (charcnt == 0) resetlevel++;
    else resetlevel = 0;
    if ((charcnt - 1) == transportlevel) transportlevel++;
    else transportlevel = 0;
    if (charcnt == 5) statlevel++;
    else statlevel = 0;
  }
  charcnt = 0;

  DEBLN(F("  checked multiple presses"));

  if (errcnt <= MAXERRCNT && errcode < FATAL_ERROR) {
    start_ms5611();
    DEBLN(F("  MS5611 initialized"));
  }

  Dot5x7.begin(DMCOL1, DMCOL2, DMCOL3, DMCOL4, DMCOL5,                  // column pins
	       DMROW1, DMROW2, DMROW3, DMROW4, DMROW5, DMROW6, DMROW7); // row pins
  Dot5x7.setFramesPerSecond(FRAMERATE);

  if (errcnt <= MAXERRCNT && errcode < FATAL_ERROR) 
    Dot5x7.setDelayFunction(busyDelay);

  DEBLN(F("  DOT Matrix display initialized"));


#if defined(__AVR_ATtiny1634__ ) 
  if ((boot_lock_fuse_bits_get(GET_LOW_FUSE_BITS) != LOW_FUSE) ||
      (boot_lock_fuse_bits_get(GET_HIGH_FUSE_BITS) != HIGH_FUSE) ||
      (boot_lock_fuse_bits_get(GET_EXTENDED_FUSE_BITS) != EXT_FUSE)) rec_error(FUSE_ERROR);
#endif

  DEBLN(F("  Fuses checked"));

  volt = Vcc::measure(10); 
  if (volt < BATT_EMPTY) rec_error(BATT_ERROR);
  DEBPR(F("  Voltage checked: "));
  DEBLN(volt);
  powerdown_unused();
  episode = MAIN_EPISODE;
  enable_wdt_counting();
  enableVibIRQ();
}

// power down all unused hardware modules;
void powerdown_unused()
{
  ADCSRA = 0; // disable ADC
  power_adc_disable();
#ifdef __AVR_ATtiny1634__
  power_usart0_disable(); 
  power_usart1_disable();
#endif
}

// start the MS5611 chip -- and if necessary recover from a non-fatal error
void start_ms5611(void)
{
  //  DEBLN(F("    MS5611 init"));
  //  DEBPR(F("    errcode="));
  //  DEBLN(errcode);
  sensor.setDelayFunction(idleDelay);
  sensor.setI2Caddr(I2C_ADDRESS);
  while (errcnt <= MAXERRCNT && errcode < FATAL_ERROR) { // recoverable or no error
    //    DEBLN(F("    Retry ..."));
    wdt_reset(); // do not count this towards a WDT reset 
    errcode = NO_ERROR;
    power_cycle();
    if (sensor.connect() > 0) {
      //      DEBLN(F("MS5611 connect failure"));
      rec_error(TWI_ERROR);
      continue;
    }
    sensor.ReadProm();
    if (sensor.Read_C(0) == 0) {
      //      DEBLN(F("MS5611 PROM-ID error"));
      rec_error(PROMID_ERROR);
      continue;
    }
    if (sensor.Calc_CRC4() != sensor.Read_CRC4()) {
      //      DEBLN(F("MS5611 CRC failure"));
      rec_error(CRC_ERROR);
      continue;
    }
    break;
  }
  wdt_reset(); // do not count this towards a WDT reset 
  //  DEBLN(F("    Leave MS5611 init"));
}

// do a power-cycle for the MS5611 chip
inline void power_cycle(void)
{
  pinMode(POWER, INPUT); // cut off power from chip
  idleDelay(20); // wait some time
  pinMode(POWER, OUTPUT);
  digitalWrite(POWER, MSACTIVE); // activate power supply for chip
  idleDelay(5); // give some time to power up, 2 msec is minimum
}

// record one error 
void rec_error(byte err)
{
  errcode = err;
  errcnt++;
  stat.errcnt[errcode]++;
}

void init_all_vars()
{
  // init error vars
  errcnt = 0;
  errcode = NO_ERROR;

  // read stat from EEPROM and initialize
  EEPROM.get(EE_STAT, stat);
  if (stat.totalsecs == 0xFFFFFFFF) {
    stat.totalsecs = 0;
    stat.wakesecs = 0;
    stat.totalwakeups = 0;
    stat.totalclimbs = 0;
    stat.totalsummits = 0;
    stat.totalstat = 0;
  }
  EEPROM.get(EE_PARA, para);
  if (para.osccal == 0xFF) rec_error(OSCCAL_ERROR);
  else OSCCAL = para.osccal;
  if (para.intref == 0xFFFF) rec_error(INTREF_ERROR);
  laststate = NO_STATE;
  quartersecs = 0;
  state = WAKEUP_STATE;
  lastbump = 0;
  refPress = 0.0;
  temperature = 0.0;
  cf = 0.0;
  summit_reached = false;
  climbed = false;
  wakemillis = 0;
  accumillis = 0;
  statlevel = 0;
  transportlevel = 0;
  resetlevel = 0;
  longmess = 0;
  wakeenable = false;
  clearstat = false;
}

/**************************** The main loop *************************************************/

void loop()
{
  char *errstr;
  int heightdiff;
  bool none;

  wdtcnt_reset();
  
  if (bumped || pressed) {
    lastbump =  getQuarterSeconds();
    bumped = false;
    DEBPR(F("."));
  }

  if (getQuarterSeconds() - lastbump > MINQUIET) state = BYE_STATE;

  if (state != BYE_STATE && state != SLEEP_STATE)
    if (errcnt > MAXERRCNT || errcode >= FATAL_ERROR)
      state = ERROR_STATE;
  
  if (transportlevel >= MAXLEVEL) {
    state = TRANSPORT_STATE;
    wakelevel = 0;
  }
  if (resetlevel >= MAXLEVEL) state = RESET_STATE;
  if (statlevel >= MAXLEVEL)
    state = STATISTICS_STATE;

  if (laststate != state) {
    DEBPR(F("New state: "));
    DEBLN(state);
    laststate = state;
  }
  
  switch(state) {

  case RESET_STATE:
    resetlevel = 0;
    refPress = 0.0;
    temperature = 0.0;
    samplecnt = 0;
    Dot5x7.scrollLeftString_P(recalib, SHOWMS*2, SCROLLMS*2, 1);
    state = EXPLAIN_STATE;
    pressed = true;
    break;

  case BYE_STATE:
    disable_wdt_counting();
    DEBLN(F("Saying bye"));
    samplecnt = -1;
    Dot5x7.scrollLeftString_P(bye, SHOWMS, SCROLLMS, 1);
    state = SLEEP_STATE;
    // do not break but just fall through
    
  case SLEEP_STATE: // sleeping and waiting for a bump or reset to wake up
    disable_wdt_counting();
    gosleep(true);
    rec_error(WAKEUP_ERROR); // should not happen!
    state = ERROR_STATE;
    Dot5x7.wakeup();
    break;

  case WAKEUP_STATE: // somebody was moving us 
    if (pressed) {
      state = EXPLAIN_STATE;
      pressed = false;
    } else { // nobody pressed the button: ask for it!
      samplecnt = 0;
      for (byte i=0; i < GREETING_REPEAT; i++) {
	DEBLN(i);
	Dot5x7.scrollLeftString_P(greeting, SHOWMS, SCROLLMS, 1);
	if (refPress >= 1.0) {
	  pinMode(POWER,INPUT); // switch off power of MS5611 
	  //	  DEBPR(F("refPress="));
	  //	  DEBLN((long)refPress);
	}
	busyDelay(1500);
      }
      state = SLEEP_STATE;
    }
    break;

  case EXPLAIN_STATE: // explain task
    if (pressed) {
       if (refPress > 1.0) state = CLIMB_STATE;
    }
    if (state == EXPLAIN_STATE) {
      samplecnt = 0;
      for (byte i=0; i < EXPLANATION_REPEAT; i++) {
	Dot5x7.scrollLeftString_P(explain, SHOWMS, SCROLLMS, 1);
	if (refPress > 1.0) {
	  DEBPR(F("refPress="));
	  DEBLN((long)refPress);
	}
	busyDelay(1500);
      }
      if (refPress > 1.0) state = CLIMB_STATE;
    }
    if (state != CLIMB_STATE) break; // otherwise fall through with pressed=true!

  case CLIMB_STATE: // steadily climbing
    if (pressed) {
      climbed = true;
      samplecnt = 0;
      if (longmess++ < LONG_REPEAT) Dot5x7.scrollLeftString_P(message1, SHOWMS, SCROLLMS, 1);
      else {
	Dot5x7.scrollLeftString_P(shortmessage1, SHOWMS, SCROLLMS, 1);
	while (samplecnt != -1) doMeasure();
      }
      if (samplecnt < 0) pinMode(POWER, INPUT); // switch off chip
      heightdiff = currentHeightDiff();
      if (heightdiff <= EPSILON) {
	Dot5x7.scrollLeftString_P(message3, SHOWMS, SCROLLMS, 1);
	state = THANKS_STATE;
      } else {
	convertNum(heightdiff, numbuf);
	DEBPR(F("numbuf="));
	DEBLN(numbuf);
	Dot5x7.scrollLeftString(numbuf, SHOWMS, SCROLLMS, 1);
	Dot5x7.scrollLeftString_P(message2, SHOWMS, SCROLLMS, 1);
      }
      Dot5x7.clear();
      pressed = false;
    } else {
      pinMode(POWER, INPUT); // switch off chip
      sleepDelay(5UL*1000UL);
      blink(50);
    }
    break;

  case THANKS_STATE:
    pinMode(POWER, INPUT); // switch off chip
    Dot5x7.scrollLeftString_P(thanks, SHOWMS, SCROLLMS, 1);
    idleDelay(1000);
    pressed = true;
    state = SUMMIT_STATE;
    // continue

  case SUMMIT_STATE: // reached the summit
    summit_reached = true;
    samplecnt = 0;
    start_ms5611();
    if (pressed) {
      Dot5x7.scrollLeftString_P(reveal, SHOWMS, SCROLLMS, 1);
      idleDelay(1000);
      if (volt < BATT_WEAK) Dot5x7.scrollLeftString_P(battweak, SHOWMS, SCROLLMS, 1);
    } else {
      while (samplecnt > 0) doMeasure();
    }
    if (currentHeightDiff() > EPSILON*2) 
      state = CLIMB_STATE;
    else {
      sleepDelay(10000);
      blink(50);
    }
    break;

  case ERROR_STATE: // display error code
    disable_wdt_counting(); // in order to avoid wdt loops
    pinMode(POWER, INPUT); // switch off chip
    DEBPR(F("ercode="));
    DEBLN(errcode);
    state = BYE_STATE;
    errstr = pgm_read_word(&error_message[errcode]);
    for (int i=0; i<ERROR_REPEAT; i++) {
      DEBLN(i);
      Dot5x7.scrollLeftString_P(errstr, SHOWMS, SCROLLMS, 1);
      idleDelay(1000);
    }
    errcode = NO_ERROR;
    errcnt = 0;
    idleDelay(1000);
    break;

  case TRANSPORT_STATE: // sleep for transport
    Dot5x7.scrollLeftString_P(transport, SHOWMS, SCROLLMS, 1);
    state = DEEPSLEEP_STATE;
    wakelevel = 0;
    transportlevel = 0;
    wakeenable = false;
    break;

  case DEEPSLEEP_STATE:
    resetlevel = 0;
    DEBPR(F("pr="));
    DEBLN(pressed);
    DEBPR(F("we="));
    DEBLN(wakeenable);
    DEBPR(F("wl="));
    DEBLN(wakelevel);
    DEBLN(F("-------"));
    if (pressed) {
      if (wakeenable) {
	wakelevel++;
	DEBPR(F("wl="));
	DEBLN(wakelevel);
	idleDelay(500);
	if (wakelevel >= 3) {
	  state = RESET_STATE;
	  break;
	}
      } else {
	DEBLN(F("wl=0"));
	wakelevel = 0;
      }
      wakeenable = false;
      sleepDelay(4000);
      blink(30);
      DEBLN(F("we=1"));
      wakeenable = true;
      sleepDelay(10000);
      blink(500);
      DEBLN(F("we=0"));
      DEBLN(F("wl=0"));
      wakeenable = false;
      wakelevel = 0;
    }
    gosleep(false);
    rec_error(WAKEUP_ERROR);
    state = ERROR_STATE;
    Dot5x7.wakeup();
    break;

  case STATISTICS_STATE:
    if (pressed && statlevel == 0) {
      if (clearstat) {
	clearStatistics();
      }
      state = BYE_STATE;
      break;
    }
    if (statlevel > 0) {
      statlevel = 0;
      stat.totalstat++;
    }
    Dot5x7.scrollLeftString(F("STATISTICS:"), SHOWMS, SCROLLMS, 1);
    idleDelay(1000);
    scrollTimes(F("ON time: "), stat.totalsecs+(getQuarterSeconds()/4));
    scrollTimes(F("WAKE time: "), stat.wakesecs+(millis()/1000));
    scrollNum(F("Wakeups:"), stat.totalwakeups, false);
    scrollNum(F("Climbs:"), stat.totalclimbs, false);
    scrollNum(F("Summits:"), stat.totalsummits, false);
    scrollNum(F("Statistics:"), stat.totalstat, false);
    idleDelay(2000);
    Dot5x7.scrollLeftString(F("ERRORS:"), SHOWMS, SCROLLMS, 1);
    none = true;
    for (byte i=NO_ERROR+1; i < ERROR_NUM; i++) 
      if (stat.errcnt[i] > 0) {
	none = false;
	scrollNum(pgm_read_word(&error_message[i]), stat.errcnt[i], true);
      }
    if (none) Dot5x7.scrollLeftString(F("none"), SHOWMS, SCROLLMS, 1);
    idleDelay(2000);
    clearstat = true;
    Dot5x7.scrollLeftString(F("PRESS TO CLEAR! "), SHOWMS, SCROLLMS, 1);
    clearstat = false;
    break;
    
  default: // somehow, the state var got a wrong value
    rec_error(STATE_ERROR);
    break;

  }
  pressed = false;
}

void blink(unsigned long period)
{
  Dot5x7.sleep();
  digitalWrite(DMCOL3, HIGH);
  digitalWrite(DMROW4, LOW);
  idleDelay(period);
  digitalWrite(DMCOL3, LOW);
  digitalWrite(DMROW4, HIGH);
  Dot5x7.wakeup();
}

/**************************** Measurement routines *************************************************/

void doMeasure(void)
{
  double sensval;
  int i, cnt = 0;
  if (samplecnt >= 0 && samplecnt < MAXSAMPLE) {
    sensor.Readout();
    sensval = sensor.GetTemp();
    while (sensval < MINTEMP || sensval > MAXTEMP) {
      rec_error(TEMP_OUTLIER_ERROR);
      if (errcnt <= MAXERRCNT) {
	start_ms5611();
	sensor.Readout();
	sensval = sensor.GetTemp();
	wdtcnt_reset();
      } else break;
    }
    if (refPress <= 1) temperature += sensval;
    sensval = sensor.GetPres();
    while (sensval < MINPRES || sensval > MAXPRES) {
      rec_error(PRESS_OUTLIER_ERROR);
      if (errcnt <= MAXERRCNT) {
	start_ms5611();
	sensor.Readout();
	sensval = sensor.GetPres();
	wdtcnt_reset();
      } else break;
    }
    if (errcnt > MAXERRCNT) return;
    sample[samplecnt++] = sensval;
  } else if (samplecnt == MAXSAMPLE) { // now compute average over non-outliers
    sensval = 0;
    sortArray(sample, MAXSAMPLE);
    for (i=THROWAWAY; i < MAXSAMPLE-THROWAWAY; i++) {
      sensval += sample[i];
      cnt++;
    }
    if (refPress <= 1)  { // compute average over all temperature measurements and compute cf
      refPress = sensval/cnt;
      temperature = temperature/MAXSAMPLE;
      cf = conversionFactor(START_HEIGHT_METERS, temperature); // conversion from Pascal to decimeters
    } else currPress = sensval/cnt;
    samplecnt = -1;
    DEBPR(F("!"));
    DEBPR(charcnt);
  }
}

// based on height and current temperature, compute actual rate of Pascal per decimeters
inline float conversionFactor(int height, float temp)
{
  return(10/(7.9+0.0008*height+0.03*temp/100));
}

int currentHeightDiff(void)
{
  float dm = (refPress-currPress)/cf;
  int up;
  DEBPR(F("decimeters="));
  DEBLN((int)dm);
  up = max(0,(int)((METERS_UP*10-dm)/CONV_MEASURE_FAC));
  DEBPR(F("up="));
  DEBLN(up);
  return up;
}
  
  
/**************************** Convert number routine and show stats **********************************************/

// scroll times
void scrollTimes(const __FlashStringHelper *mess, unsigned long int secs)
{
  Dot5x7.scrollLeftString_P(reinterpret_cast<PGM_P>(mess), SHOWMS, SCROLLMS, 1);
  convertTime(secs, numbuf);
  Dot5x7.scrollLeftString(numbuf, SHOWMS, SCROLLMS, 1);
  idleDelay(1000);
}

void scrollNum(const __FlashStringHelper *mess, int num, bool colon)
{
  Dot5x7.scrollLeftString_P(reinterpret_cast<PGM_P>(mess), SHOWMS, SCROLLMS, 1);
  if (colon) Dot5x7.scrollLeftString(F(":"), SHOWMS, SCROLLMS, 1);
  convertNum(num, numbuf);
  Dot5x7.scrollLeftString(numbuf, SHOWMS, SCROLLMS, 1);
  idleDelay(1000);
}



// place a string corresponding to a number into buf
void convertNum(int num, char buf[])
{
  int digval = 10000;
  byte i = 0;

  if (num < 0) {
    num = -num;
    buf[i++] = '-';
  }
  while (digval > 1 && (num/digval) == 0) digval = digval / 10;
  while (digval > 0) {
    buf[i++] = (num/digval) + '0';
    num = num%digval;
    digval = digval/10;
  }
  buf[i] = '\0';
}

// computer a string corresponding to time period given in seconds
void convertTime(unsigned long int period, char buf[])
{
  int i;
  convertNum(period/3600, buf);
  i = strlen(buf);
  buf[i++] = 'h';
  buf[i++] = ' ';
  convertNum((period%3600)/60, &buf[i]);
  i = strlen(buf);
  buf[i++] = 'm';
  buf[i++] = '\0';
}

/**************************** Powersave routines *************************************************/

// wait for 'msecs' millseconds and switch into idle mode as much as possible
void idleDelay(unsigned long msecs)
{
  unsigned long start = millis();

  while (millis() - start < msecs) {
    wdtcnt_reset();
    set_sleep_mode(SLEEP_MODE_IDLE);
    sleep_mode();
  }
}

// wait for 'msecs' milliseconds and do measurements in parallel, if necessary and possible
// This is the method, the DotMatrix class should use for delays. It also counts displayed
// chars by incrmenting the counter whenever we wait for SHOWMS milliseconds. 
void busyDelay(unsigned long msecs)
{
  unsigned long start = millis();
  unsigned long remaining;

  while (millis() - start < msecs) {
    wdtcnt_reset();
    remaining = msecs - (millis() - start);
    if (samplecnt >= 0 && samplecnt <= MAXSAMPLE) { // last number triggers computing the average
      if (remaining > 30 && errcnt <= MAXERRCNT && errcode < FATAL_ERROR) { // we can initiate a measurement
	doMeasure();
      } else idleDelay(remaining);
    } else idleDelay(remaining);
  }
  if (msecs == SHOWMS) // i.e. have shown a character
    charcnt++; // count this character!
}

// sleep for 'msecs' milliseconds (note: we have only a quarter seconds resolution!)
// put MS5611 and display to sleep!
void sleepDelay(unsigned long msecs)
{
  unsigned int wait = (msecs+249)/250;
  unsigned int start = getQuarterSeconds();

  pinMode(POWER, INPUT);
  Dot5x7.sleep();
  while (getQuarterSeconds() - start < wait) {
    wdtcnt_reset();
    if (bumped) disableVibIRQ(); // in order not to be woken up too often
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    sleep_mode();
  }
  Dot5x7.wakeup();
  enableVibIRQ();
}

void gosleep(bool wakeup_on_bump)
{
  bool repeat_bump = false;

  disableVibIRQ();   // disable IRQs
  pinMode(POWER, INPUT); // powerdown the MS5611
  Dot5x7.sleep(); // disable display
  digitalWrite(14, LOW); // if this pin (PB3) stayed high, then the MCU draws 0.45 mA in powerdown mode!!!
  storeStatistics(); // store the current statistics
  disable_wdt_counting(); // no more timing
#ifdef DEBUG
  DEBLN(F("Go to sleep ..."));
  DEBPR(F("episode="));
  DEBLN(episode);
  DEBPR(F("state="));
  DEBLN(state);
  DEBPR(F("quartersecs="));
  DEBLN(quartersecs);
  DEBPR(F("wakemillis="));
  DEBLN(wakemillis);
  DEBPR(F("accumillis="));
  DEBLN(accumillis);
  DEBPR(F("errcnt="));
  DEBLN(errcnt);
  DEBPR(F("errcode="));
  DEBLN(errcode);
  delay(5000); // allow print to finsh
#endif
  while (!repeat_bump) {
    if (wakeup_on_bump) enableVibIRQ(); // enable Vib IRQs
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    sleep_mode(); // sleep & wait for reset or IRQ
    disableVibIRQ();
    bumped = false;
    idleDelay(300);
    enableVibIRQ(); // enable Vib IRQs
    idleDelay(1000);
    repeat_bump = bumped;
  }
#ifdef DEBUG
  DEBLN(F("WDT-Restart"));
  delay(1000);
#endif
  wdt_enable(WDTO_15MS); // provoke a software reset
  while (true);
}

void storeStatistics()
{
  stat.totalsecs += getQuarterSeconds()/4;
  stat.wakesecs += (millis()/1000);
  stat.totalwakeups++;
  if (climbed) stat.totalclimbs++;
  if (summit_reached) stat.totalsummits++;
  EEPROM.put(EE_STAT, stat);
}

void clearStatistics()
{
  DEBLN(F("clearstat"));
  stat.totalsecs = 0;
  stat.wakesecs = 0;
  stat.totalwakeups = 0;
  stat.totalclimbs = 0;
  stat.totalsummits = 0;
  stat.totalstat = 0;
  for (byte i=0; i < ERROR_NUM; i++) stat.errcnt[i] = 0;
}
