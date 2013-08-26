// -*-c++-*-

/* A sketch that has the mission of sending people up for a couple of meters
   and then bringing them down again
   Uses the MS5611 altimeter/barometer

   Developed on an Arduino Pro Mini, deployed on a ATtiny84, meanwhile ATtiny167
   PCB is the Open-V3a board that fits together with a 3.6V/2400mA lithium
   battery into a preform tube (15cm)

   ATtiny Fusebits: Divide clock by 8, Brown-out disabled
   Arduino board: ATtiny 167 / 1MHz
*/


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
 */
#define Version "4.1"

#include <avr/wdt.h>
#include <avr/sleep.h>
#include <avr/eeprom.h>
#include <avr/pgmspace.h>

#define SCHOENBERG_SETTING 1
#define HOME_SETTING 2
#define DAGSTUHL_SETTING 3

#define SETTING DAGSTUHL_SETTING

#if SETTING==HOME_SETTING
#define MAXWAKEUP_NOBUMP 60 // if there is no bump, we go to sleep again 
#define MAXWAKEUP_TOTAL 120 // 2 minutes for wakeup & start climbing
#define MINQUIET 180 // 3 minutes no bump means we are back in the box
#define DEFAULT_METERS_UP 6 // meters you have to walk up
#define DEFAULT_EPSILON 2  // potential error
#define DEFAULT_SUMMIT_STR "OBEN" // message displayed on summit, max 30 chars
#define INTREFVOLTAGE 1120
#define OLD_I2C_WIRING // define if first board (= dragon) is used, depends of course!
#define ADDRESS 0xEC // depends, of course!
#elif SETTING==DAGSTUHL_SETTING
#define MAXWAKEUP_NOBUMP 120 // if there is no bump, we go to sleep again 
#define MAXWAKEUP_TOTAL 3600 // 60 minutes for wakeup & start climbing
#define MINQUIET 540 // 10 minutes no bump means we are back in the box
#define DEFAULT_METERS_UP 60 // meters you have to walk up
#define DEFAULT_EPSILON 5  // potential error
#define DEFAULT_SUMMIT_STR "CODE 132" // message displayed on summit, max 30 chars
#define ENGLISH // all messages in English
#define IMPERIAL // measures in feet
#define INTREFVOLTAGE 1112
#define ADDRESS 0xEE // 8-bit I2C address of MS5611 (CSB not connected to Vcc)
#elif SETTING==SCHOENBERG_SETTING
#define MAXWAKEUP_NOBUMP 120 // if there is no bump, we go to sleep again 
#define MAXWAKEUP_TOTAL 1800 // 30 minutes for wakeup & start climbing
#define MINQUIET 300 // 5 minutes no bump means we are back in the box
#define DEFAULT_METERS_UP 32 // meters you have to walk up
#define DEFAULT_EPSILON 7  // potential error
#define DEFAULT_SUMMIT_STR "CODE 4213" // message displayed on summit, max 30 chars
#define OLD_I2C_WIRING // define if first board (= dragon) is used!
#define INTREFVOLTAGE 1120
#define ADDRESS 0xEC // 8-bit I2C address of MS5611 (CSB connected to Vcc)
#else
#error "Undefined setting"
#endif


#define MAXSUMSTR 200 // should be less than 512-6
#define MAXOUTLIER 0.6 // max diff between extreme measurements corresponds to 5 meters

uint8_t emeters_up[1] EEMEM;
uint8_t eepsilon[1] EEMEM;
char esummit_str[MAXSUMSTR+1] EEMEM;
uint16_t eemagickey2[1] EEMEM;
uint8_t eelowvolterr[1] EEMEM;


// #define DEB_LCD
// #define DEB_TTY
// #define DEB_LED
// #define DEB_LEDRAM
// #define DEB_LEDBATT
// #define DEB_LEDHEIGHT
// #define DEB_LEDSTABLE

#ifdef IMPERIAL
#define CONVFAC 3.048 // 3.048 decimeters are one foot
#else
#define CONVFAC 10.0  // 10 decimeters are one meter
#endif

#define DISPLAY_ON_MSECS 1000 // msecs on
#define DISPLAY_OFF_MSECS 300 // msecs off
#define MAXERRCNT 20 // after that many measurement errors we give up
#define MAXRETRYCNT 5 // number of retries in reset and param command
#define LOWVOLT_WARN 280 // warning that battery is low
#define LOWVOLT_ERR 230 // with that we do not startup anymore!



/* error codes */
#define NO_ERROR 0
#define RESET_ERROR '1'
#define PARAM_ERROR '2'
#define CONV_ERROR '3'
#define TEMP_ERROR '4'
#define PRESS_ERROR '5'
#define NOBUMP_ERROR '6'
#define HEIGHT_ERROR '7'
#define WAKEUP_ERROR '8'
#define STATE_ERROR '9'
#define OUTLIER_ERROR 'A'
#define I2C_ERROR 'C'


/* On the PCB (or breadboard) put in the the following electronic components:

   IC1: Attiny84 with pin 1 away from prototyping area
   Display1: 7-segment with decimal point facing away from the Attiny
   C1: 100nF
   R1: 330 Ohm (if you use extra bright seven segment displays)
   R2: 3 MOhm
   R3: 0 Ohm, just a wire
   R4: 0 Ohm, just a wire
   R5: 0 Ohm, just a wire
   R9: 10 kOhm
   S1: Vibration switch
   The breakout board with the MS5611 on the prototype area
   row 2 to 8, pin 1
 
   Now connect:
   Pin5 of the first row in the prototype area (PA1 = D1) with SDA MS5611
   Pin4 of the first row (PA2 = D2) with SCL MS5611
   Pin5 of the 8th row (Vcc) with Vcc MS5611
   Pin2 of the 8throw (GND) with GND MS5611

   Note: On the MS5611 breakout board, you have to connect 
   - the I2C bridge (of SPI/I2C), 
   - the pullup resistor bridge, 
   - and the bottom I2C ADDR bridge - NO! We now use EE instead of EC!

   On breadboard connect:
   Attiny84-Pin Attiny84 ATtiny167-Pin ATtiny167 ProMini External
                         PB0           D8(20)            Button
   PA0          D0       PA3           D3(4)     D2      Vibration switch
   PA1          D1       PA0           D0(1)     D3      MS5611 SDA
   PA2          D2       PA1           D1(2)     D4      MS5611 SCL
                         PA6           D6(9)             MS5611 Vcc
							 MS5611 GND
   PA3          D3       PB3           D11(17)   D5      Disp. Pin1(g) 
   PA4          D4       PB4           D12(14)   D6      Disp. Pin2(f) + ISP CLK
   PA5          D5       PB5           D13(13)   D7      Disp. Pin4(e) + ISP MISO
   PA6          D6       PB6           D14(12)   D8      Disp. Pin5(d) + ISP MOSI
   PA7          D7       PA7           D7(10)    D9      Disp. Pin6(DP)
   PB2          D8       PA5           D5(8)     D10     Disp. Pin7(c)
   PB1          D9       PA4           D4(7)     D11     Disp. Pin9(b)
   PB0          D10      PA2           D2(3)     D12     Disp. Pin10(a)
                         PB2           D10(18)   D1      Serial TX
                                                         Disp. Pin8 or 3 to R1
				                 D13     LCD   SCLK
				                 D14     LCD   DIN
				                 D15     LCD   D/C
				                 D16     LCD   RST
				                 D17     LCD   CS
				                         LCD   Vcc
						         LCD   GND
*/

#if defined(DEB_LCD)
//#include <Adafruit_GFX.h>
//#include <Adafruit_PCD8544.h>
//Adafruit_PCD8544 lcd = Adafruit_PCD8544(13,14,15,17,16); 
#endif

// differences between Atmega and Attiny
#if defined(__AVR_ATmega328P__)
  #define SEGAPIN 12
  #define SEGBPIN 11
  #define SEGCPIN 10
  #define SEGDPPIN 9
  #define SEGDPIN  8
  #define SEGEPIN  7
  #define SEGFPIN  6
  #define SEGGPIN  5
  #define T1_vect TIMER1_OVF_vect
  #define PCINT_vect PCINT2_vect
  #define SDA_PORT PORTD
  #define SDA_PIN 3
  #define SCL_PORT PORTD
  #define SCL_PIN 4
#elif defined(__AVR_ATtiny84__)
  #define SEGAPIN 10
  #define SEGBPIN  9
  #define SEGCPIN  8
  #define SEGDPPIN 7
  #define SEGDPIN  6
  #define SEGEPIN  5
  #define SEGFPIN  4
  #define SEGGPIN  3
  #define T1_vect TIM1_OVF_vect
  #define PCINT_vect PCINT0_vect
  // I2C pins - now conforms to description above!
  #define SDA_PORT PORTA
  #define SCL_PORT PORTA
  #ifdef OLD_I2C_WIRING // as on the dragon board!
    #define SDA_PIN 2
    #define SCL_PIN 1
  #else                // as described above and on the new boards
    #define SDA_PIN 1
    #define SCL_PIN 2
  #endif
#elif defined(__AVR_ATtiny167__)
  #define SEGAPIN  2
  #define SEGBPIN  4
  #define SEGCPIN  5
  #define SEGDPPIN 7
  #define SEGDPIN  14
  #define SEGEPIN  13
  #define SEGFPIN  12
  #define SEGGPIN  11
  #define T1_vect TIMER1_OVF_vect
  #define PCINT_vect PCINT0_vect
  #define PCINTBUTTON_vect PCINT1_vect
  #define WDTCSR WDTCR
  #define SDA_PORT PORTA
  #define SCL_PORT PORTA
  #define SDA_PIN 0
  #define SCL_PIN 1
  #define SDA_ARDUINO_PIN 0
  #define SCL_ARDUINO_PIN 1
  #define MS_POWER 6
  #define BUTTON 8
  #include <TXOnlySerial.h>
  #ifdef DEB_TTY
    Serial TXOnlySerial(10);
  #endif
#else
  #error "Unsupported MCU"
#endif


#include <SoftI2CMaster.h>

#if  defined(DEB_TTY) 
#define DEBTTY_PRINT(str) Serial.print(str)
#define DEBTTY_PRINTLN(str) Serial.println(str)
#define DEBTTY_INIT() Serial.begin(9600)
#else
#define DEBTTY_PRINT(str)
#define DEBTTY_PRINTLN(str)
#define DEBTTY_INIT()
#endif

#if  defined(DEB_LCD) 
#define DEBLCD_PRINT(str) lcd.print(str)
#define DEBLCD_PRINTLN(str) lcd.println(str)
#define DEBLCD_HOME() lcd.setCursor(0,0)
#define DEBLCD_DISPLAY() lcd.display()
#define DEBLCD_CLEAR() lcd.clearDisplay()
#define DBLCD_INIT()   lcd.begin()
#else
#define DEBLCD_PRINT(str)
#define DEBLCD_PRINTLN(str)
#define DEBLCD_HOME()
#define DEBLCD_DISPLAY()
#define DEBLCD_CLEAR()
#define DEBLCD_INIT()
#endif

// error value for measurements
#define ERRORVAL -9999.0


// LCD segment definitions. 
// These will need to be changed depending on the 
// wiring of your output port to the segements.
#define SEGa (1<<SEGAPIN)
#define SEGb (1<<SEGBPIN)
#define SEGc (1<<SEGCPIN)
#define SEGd (1<<SEGDPIN)
#define SEGe (1<<SEGEPIN)
#define SEGf (1<<SEGFPIN)
#define SEGg (1<<SEGGPIN)
#define SEGdp (1<<SEGDPPIN)


const uint8_t lcd_seg[] =
  { SEGAPIN, SEGBPIN, SEGCPIN, SEGDPIN, SEGEPIN, SEGFPIN, SEGGPIN, SEGDPPIN };


// LCD Character Generator 
// Change these defines as needed to make new characters.

const uint16_t ascii_gen[] = {
  0, // " "
  0, // "!"
  SEGf+SEGb, // """
  SEGa+SEGb+SEGg+SEGf, // "#" - display degree symbol 
  0, // "$"
  0, // "%"
  0, // "&"
  SEGb, // "'"
  SEGa+SEGd+SEGe+SEGf, // "("
  SEGa+SEGb+SEGc+SEGd, // ")"
  0, // "*"
  0, // "+"
  SEGc, // ","
  SEGg, // "-"
  SEGdp, // "."
  0, // "/"
  SEGa+SEGb+SEGc+SEGd+SEGe+SEGf,   // Displays "0"
  SEGb+SEGc,           // Displays "1"
  SEGa+SEGb+SEGd+SEGe+SEGg,     // Displays "2"
  SEGa+SEGb+SEGc+SEGd+SEGg,     // Displays "3"
  SEGb+SEGc+SEGf+SEGg,       // Displays "4"
  SEGa+SEGc+SEGd+SEGf+SEGg,     // Displays "5"
  SEGa+SEGc+SEGd+SEGe+SEGf+SEGg,   // Displays "6"
  SEGa+SEGb+SEGc,         // Displays "7"
  SEGa+SEGb+SEGc+SEGd+SEGe+SEGf+SEGg, // Displays "8"
  SEGa+SEGb+SEGc+SEGd+SEGf+SEGg,   // Displays "9"
  0, // ":"
  0, // ";"
  SEGd+SEGe+SEGg, // "<"
  SEGg+SEGd, // "="
  SEGg+SEGc+SEGd, // ">"
  SEGa+SEGb+SEGg+SEGe, // "?"
  0, // "@"
  SEGa+SEGb+SEGc+SEGe+SEGf+SEGg, // "A"
  SEGc+SEGd+SEGe+SEGf+SEGg,   // "b"
  SEGg+SEGe+SEGd,     // "c"
  SEGb+SEGc+SEGd+SEGe+SEGg,   // "d"
  SEGa+SEGd+SEGe+SEGf+SEGg,   //  "E"
  SEGa+SEGe+SEGf+SEGg,     //  "F"   
  SEGa+SEGc+SEGd+SEGe+SEGf,   //  "G"
  SEGc+SEGe+SEGf+SEGg,     //  "h"
  SEGf+SEGe,         //  "I"
  SEGb+SEGc+SEGd+SEGe,     //  "J"
  SEGa+SEGc+SEGe+SEGf+SEGg,   //  "k"
  SEGd+SEGe+SEGf,       //  "L"
  SEGg+SEGc+SEGe,       //  "M"
  SEGc+SEGe+SEGf+SEGb+SEGa,       //  "n"
  SEGc+SEGd+SEGe+SEGg,     //  "o"
  SEGa+SEGb+SEGe+SEGf+SEGg,   //  "P"
  SEGa+SEGb+SEGd+SEGf+SEGg,   //  "q"
  SEGe+SEGg,         //  "r"
  SEGc+SEGf+SEGg,   //  "S"
  SEGd+SEGe+SEGf+SEGg,     //  "t"
  SEGb+SEGc+SEGd+SEGe+SEGf,   //  "U"
  SEGc+SEGd+SEGe,       //  "v"
  SEGb+SEGd+SEGf,       //  "W"
  SEGb+SEGc+SEGe+SEGf+SEGg,   //  "X"
  SEGb+SEGc+SEGd+SEGf+SEGg,   //  "Y"
  SEGa+SEGb+SEGd+SEGe+SEGg,    //  "Z"
};
#define LASTCHAR 'Z'

/* messages */
const char reset_str[] PROGMEM = "RESET";
#ifdef ENGLISH
const char lowbatt_str[] PROGMEM = "  BATT LO. ";
const char go_str[] PROGMEM = "GO";
#ifdef IMPERIAL
const char up_str[] PROGMEM ="UP";
const char meter_str[] PROGMEM ="FEET";
#else
const char up_str[] PROGMEM ="UP";
const char meter_str[] PROGMEM ="METER";
#endif
const char bye_str[] PROGMEM = "BYE.";
const char error_str[] PROGMEM = "  ERROR";
const char sleep_str[] PROGMEM = "SP";
#else
const char lowbatt_str[] PROGMEM = "  BATT LO. ";
const char go_str[] PROGMEM = "";
const char up_str[] PROGMEM ="HOCH LAUFEN";
const char meter_str[] PROGMEM ="METER";
const char bye_str[] PROGMEM = "BYE.";
const char error_str[] PROGMEM = "  FEHLER";
const char sleep_str[] PROGMEM = "SP";
#endif


/* states */

#define NO_STATE 0
#define SLEEP_STATE 1
#define WAKEUP_STATE 2
#define CLIMB_STATE 3
#define SUMMIT_STATE 4
#define BATT_LOW_STATE 5
#define ERROR_STATE 6
#define DEEPSLEEP_STATE 7
#define RESET_STATE 8
#define LAST_STATE 8


#if (defined(DEB_TTY) || defined(DEB_LCD))
char statestr[14][12] = { "UNDEF", "SLEEP", "WAKEUP", 
			  "CLIMB", "SUMMIT", "BATT_LOW", "ERROR", 
			  "DEEP_SLEEP" };
#endif


/* magic keys - to find out whether reset is a startup */
#define MKEY1 0x7109
#define MKEY2 0xFD1E

/***** super global variables (surviving resets) *****/
// The state var
volatile uint8_t state __attribute__ ((section (".noinit")));
// value of start variable last time
volatile uint8_t laststate __attribute__ ((section (".noinit")));
// This counter is advanced by the wdt interrupt - started after wakeup
volatile uint16_t seconds __attribute__ ((section (".noinit"))); 
// Keeping track when last bump happened (using seconds)
uint16_t lastbump __attribute__ ((section (".noinit"))); 
// last time the reset button was pressed
uint16_t lastpress __attribute__ ((section (".noinit"))); 
// last time, the state was changed
uint16_t lastchange __attribute__ ((section (".noinit"))); 
// last time, we measured the pressure
volatile uint16_t lastmeasure __attribute__ ((section (".noinit"))); 
// reference pressure for 0 level
volatile float refPress __attribute__ ((section (".noinit"))); 
// measurement converted to height above zero level
volatile int height __attribute__ ((section (".noinit")));
// last height when making measurement

volatile int lastheight __attribute__ ((section (".noinit")));

volatile uint8_t sleeplevel __attribute__ ((section (".noinit")));
volatile uint8_t resetlevel __attribute__ ((section (".noinit")));
uint16_t magickey1 __attribute__ ((section (".noinit"))); 





/***** global vars ****/
char summit_str[31] = DEFAULT_SUMMIT_STR; // will be initialized in setup!
int errcnt = 0;
char errcode = '\0';
int decimeters_up; // will be initialized from EEPROM
int epsilon; // will be initialized from EEPROM
int volt;
uint8_t pressed = true;
volatile uint8_t bumped = false;
uint8_t lbumped = false;


/* interrupt routine to drive display */
/* variables for display driver */
int csegix = 0; // current segment index
long cmask = 0;
volatile unsigned char dispchar = ' ';


// correction coefficents of MS5611
uint16_t C[7];

// MS5611 variables
// SoftI2cMaster baro(SDA_PIN,SCL_PIN);

ISR(T1_vect)
{
  TCNT1 = 0xFFFF-40; // 156 for 8MHz
  digitalWrite(lcd_seg[csegix],LOW);
  if (++csegix == 8) {
    csegix = 0;
  }
  if (1<<lcd_seg[csegix]&ascii_gen[(0x7F&dispchar)-' ']) {
    digitalWrite(lcd_seg[csegix],HIGH);
  }
}

ISR(WDT_vect)
{
  WDTCSR |= (1<<WDIE); // re-enable watchdog interrupt
  seconds++;
}

// vibration switch
ISR(PCINT_vect)
{
  bumped = true;
}

// button (if not directly connected to reset)
#ifdef PCINTBUTTON_vect
ISR(PCINTBUTTON_vect)
{
  wdt_enable(WDTO_15MS);
  WDTCSR &= ~(1<<WDIE); // disable WDT interrupt, leads to soft reset
  while(1);
}
#endif

void startTimerOne(void) {
  TCCR1A = 0;
  TCCR1B = (1<<CS11|1<<CS10); //  prescaler = 64
  TCCR1C = 0;
  TIMSK1 = (1<<TOIE1);
  TCNT1 = 0xFFFF-200;
}

void stopTimerOne(void) {
  TIMSK1 = 0;
}

void idleDelay(unsigned int msecs)
{
  unsigned long start;

  // millis() is very inaccurate, so we use delay() for everything <= 50 msec
  if (msecs <= 50) delay(msecs);
  else { 
    start = millis();
    while (millis() - start < msecs) {
      set_sleep_mode(SLEEP_MODE_IDLE);
      sleep_mode();
    }
  }
}


void displayChar(char c) 
{
  dispchar = c;
  idleDelay(DISPLAY_ON_MSECS);
  dispchar = ' ';
  idleDelay(DISPLAY_OFF_MSECS);
}

void displayPString(const char *mess)
{
  char c;
  while (c = pgm_read_byte(mess++)) {
    displayChar(c);
  }
}

void displaySummitString(void) {
  char c;
  byte i = 0;
  while (c = (char)eeprom_read_byte((uint8_t*)(&esummit_str[i++]))) {
    displayChar(c);
  }
}

void displayNum(int num)
{
  int digval = 10000;

  if (num < 0) {
    num = -num;
    displayChar('-');
  }
  while (digval > 1 && (num/digval) == 0) digval = digval / 10;
  while (digval > 0) {
    displayChar((num/digval) + '0');
    num = num%digval;
    digval = digval/10;
  }
}


float getAltiVal(byte code)
{
  float ret = 0;
  if (!i2c_start(ADDRESS | I2C_WRITE)) return ERRORVAL;
  if (!i2c_write(code)) return ERRORVAL;
  i2c_stop();
  idleDelay(15); // at least 10, 15 is on the safe side
  // start read sequence
  if (!i2c_start(ADDRESS | I2C_WRITE)) return ERRORVAL;
  if (!i2c_write(0x00)) return ERRORVAL; // request ADC vals
  i2c_stop();
  if (!i2c_start(ADDRESS | I2C_READ)) return ERRORVAL;
  ret = i2c_read(false) * 65536.0 + i2c_read(false) * 256.0 + i2c_read(true);
  i2c_stop();
  return ret;
}

bool resetAlti(void)
{
  bool fail = true;
  if (i2c_start(ADDRESS | I2C_WRITE)) 
    if (i2c_write(0x1E)) {
      idleDelay(6);
      fail = false;
    }
  i2c_stop();
  if (fail) {
    errcnt++;
    errcode = RESET_ERROR;
    DEBTTY_PRINTLN(F("Reset command unsuccessfull"));
  }
  return (!(fail));
}

void paramAlti(void)
{
  bool fail = true;
  int retrycnt = 0;
  DEBTTY_PRINTLN("");
  DEBTTY_PRINTLN(F("PROM COEFFICIENTS"));
  while (fail && retrycnt++ < MAXRETRYCNT && errcnt < MAXERRCNT) {
    fail = false;
    for (int i=0; i<6 && !fail ; i++) {
      if (!i2c_start(ADDRESS | I2C_WRITE)) fail = true;
      if (!fail && !i2c_write(0xA2 + i*2)) fail = true;
      i2c_stop();
      if (!fail && !i2c_start(ADDRESS | I2C_READ)) fail = true;
      C[i+1] = i2c_read(false) << 8 | i2c_read(true);
      i2c_stop();
      DEBTTY_PRINTLN(C[i+1]);
      if (fail) {
	DEBTTY_PRINTLN(F("Prom command unsuccessfull"));
	errcnt++;
	recoverFromI2cProblem();
	errcode = PARAM_ERROR;
	idleDelay(10);
      }
    }
    DEBTTY_PRINTLN("");
  }
  if (fail) {
    errcnt = MAXERRCNT;
    DEBTTY_PRINTLN(F("*** Fatal error in prom command"));
  }
}


float measurePress(int count)
{
  float D1;
  float D2;
  float dT;
  float TEMP, P;
  float OFF; 
  float SENS; 
  float acc = 0;
  float minpress = 5000.0;
  float maxpress = 0;

  if (count <= 0) count = 1;
  while (errcnt < MAXERRCNT) {
    acc = 0;
    maxpress = 0;
    minpress = 5000.0;
    
    for (int i = 0; i < count && errcnt < MAXERRCNT; i++) {
      D1 = getAltiVal(0x48); // Pressure raw
      D2 = getAltiVal(0x58);// Temperature raw
      
      if (D1 != ERRORVAL && D2 != ERRORVAL) {
	dT   = D2 - (C[5] * 256.0);
	OFF  = C[2] * 65536.0 + ((dT * C[4])/128.0);
	SENS = (C[1] * 32768.0) + ((dT * C[3])/256.0);
	
	TEMP = dT * C[6] / 8388608.0 + 2000.0;
	
	if(TEMP < 2000.0) // if temperature lower than 20 Celsius 
	  {
	    int32_t T1    = 0;
	    int64_t OFF1  = 0;
	    int64_t SENS1 = 0;
	    
	    T1    = pow(dT, 2) / 2147483648;
	    OFF1  = 5 * pow((TEMP - 2000), 2) / 2;
	    SENS1 = 5 * pow((TEMP - 2000), 2) / 4;
	    
	    if(TEMP < -1500) // if temperature lower than -15 Celsius 
	      {
		OFF1  = OFF1 + 7 * pow((TEMP + 1500), 2); 
		SENS1 = SENS1 + 11 * pow((TEMP + 1500), 2) / 2;
	      }
	    
	    TEMP -= T1;
	    OFF -= OFF1; 
	    SENS -= SENS1;
	  }
	if (TEMP < -7000.0 || TEMP > 12000.0) { // almost impossible temperatures
	  errcnt++;
	  errcode = TEMP_ERROR;
	  if (errcnt < MAXERRCNT) { // reset the chip again
	    resetAlti();
	    i--; // redo last measurement
	    continue;
	  } else {
	    DEBTTY_PRINTLN(F("*** Fatal abort because too many measurement errors"));
	      return(0);
	  }
	}
	P = (D1 * SENS / 2097152 - OFF) / 32768 / 100.0;

	if (P < 600.0 || P > 1200.0) { // almost impossible pressures up to 4000 meters elevation
	  errcnt++;
	  errcode = PRESS_ERROR;
	   if (errcnt < MAXERRCNT) { // reset the chip again
	    resetAlti();
	    i--; // redo last measurement
	    continue;
	  } else {
	     DEBTTY_PRINTLN(F("*** Fatal abort because too many measurement errors"));
	      return(0);
	  }
	}
	acc  += P;
	if (minpress > P) minpress = P;
	if (maxpress < P) maxpress = P;
      } else {
	errcnt++;
	errcode = CONV_ERROR;
	DEBTTY_PRINTLN(F("Measure command unsuccessful"));
	  if (errcnt < MAXERRCNT) { // reset the chip again
	    recoverFromI2cProblem();
	    i--; // redo last measurement
	    errcode = CONV_ERROR;
	  } else {
	    DEBTTY_PRINTLN(F("*** Fatal abort because too many measurement errors"));
	      return(0);
	  }
      }
    }
    if (maxpress-minpress > MAXOUTLIER) { // if more than 0.6 hPa resp. 5 meters diff
      errcnt++;
      errcode = OUTLIER_ERROR;
    } else break;
  }
  return (acc/count);
}


float measurePressWithRecovery(int count)
{
  float result;
  bool cont = true;
  int retrycnt = 4;

  while (retrycnt-- && cont) {
    result = measurePress(count);
    if (errcnt >= MAXERRCNT && retrycnt) {
      errcnt = 0;
      errcode = NO_ERROR;
      resetAlti();
      idleDelay(20);
      paramAlti();
      idleDelay(20);
    } else cont = false;
  }
  return result;
}


int readVoltage(void)
{
  int result, reading;
  
  ADCSRA |= (1<<ADEN); // switch on ADC  
  idleDelay(20);

#if defined(__AVR_ATmega328P__)
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
#elif defined(__AVR_ATtiny84__)
  ADMUX = _BV(MUX5) | _BV(MUX0);
#elif defined(__AVR_ATtiny167__)
  ADMUX = _BV(MUX3) | _BV(MUX2);
#else
  #error "Unsupported MCU"
#endif

  idleDelay(20); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Convert
  while (bit_is_set(ADCSRA,ADSC));
  ADCSRA |= _BV(ADSC); // Convert again
  while (bit_is_set(ADCSRA,ADSC));
  reading = ADC;
  result = (((INTREFVOLTAGE * 1023L) / ADC) + 5L) / 10L;

  DEBTTY_PRINT(F("Raw voltage: "));
  DEBTTY_PRINTLN(reading);
  DEBTTY_PRINT(F("Voltage:  "));
  DEBTTY_PRINTLN(result);

  ADCSRA &= ~(_BV(ADEN)); // switch off ADC
  return result;
}

#if defined(DEB_LCD) || defined(DEB_TTY) || defined(DEB_LEDRAM) 
// function to return free mem
int freeRam(void)
{
  extern unsigned int __heap_start;
  extern void *__brkval;

  int free_memory;
  int stack_here;

  if (__brkval == 0)
    free_memory = (int) &stack_here - (int) &__heap_start;
  else
    free_memory = (int) &stack_here - (int) __brkval; 

  return (free_memory);
}
#endif


// This guards against reset loops caused by resets
// is useless under Arduino's bootloader
void wdt_init(void) __attribute__((naked)) __attribute__((section(".init3"))) __attribute__((used));
void wdt_init(void)
{
  MCUSR = 0;
  wdt_disable();
  return;
} 


void enablePinChangeIRQ(void) 
{
#if defined(__AVR_ATmega328P__)
  PCICR |= (1<<PCIE2); // allow for PCINT on PCINT16-23
  PCMSK2 |= (1<<PCINT18); // on pin PD2 = PCINT18 = dig. pin 2
#elif defined(__AVR_ATtiny84__)
  GIMSK |= (1<<PCIE0);  // allow for PCINT on PA0-PA7
  PCMSK0 |= (1<<PCINT0); // on pin PA0 = ADC0 = analog0 = dig. pin 0
#elif defined(__AVR_ATtiny167__)
  PCICR |= (1<<PCIE0) | _BV(PCIE1); // allow interrupts on both ports
  PCMSK0 |= (1<<PCINT3); // vibration switch
  PCMSK1 |= (1<<PCINT8); // button
#else
  error "MCU not supported"
#endif
}

void disableVibIRQ(void) 
{
#if defined(__AVR_ATmega328P__)
  PCICR &= ~(1<<PCIE2); // disable PCINT on PCINT16-23
#elif defined(__AVR_ATtiny84__)
  GIMSK &= ~(1<<PCIE0);  // disable PCINT on PA0-PA7
#elif defined(__AVR_ATtiny167__)
  PCICR &= ~(1<<PCIE0);
#else
  #error "MCU not supported"
#endif
}

void initIO(void) {
  int i;
  for (i=0;i<8;i++) {
    pinMode(lcd_seg[i],OUTPUT);
    digitalWrite(lcd_seg[i],LOW);
  }
}

void IOoff(void) {
  int i;
  for (i=0;i<8;i++) {
    digitalWrite(lcd_seg[i],LOW);
  }
  stopTimerOne();
}

void setupIO(void)
{
  initIO();
  startTimerOne();
  enablePinChangeIRQ();
  wdt_enable(WDTO_1S);
  WDTCSR |= (1<<WDIE); // enable WDT interrupt instead of reset
}

void setSeconds(uint16_t &var)
{
  cli();
  var = seconds;
  sei();
}

uint16_t getSeconds(void)
{
  uint16_t res;
  cli();
  res = seconds;
  sei();
  return res;
}

void gosleep(void)
{
  wdt_disable(); // no more seconds counting
  IOoff(); // no more IO
#ifdef MS_POWER
  // powerdown the MS5611
  pinMode(SDA_ARDUINO_PIN, INPUT);
  pinMode(SCL_ARDUINO_PIN, INPUT);
  pinMode(MS_POWER, INPUT);
#endif
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_mode(); // sleep & wait for reset
}

// power-cycle or just toggle the clock-line
void recoverFromI2cProblem(void)
{
  byte retry = 5;
  while (retry--) {
#ifdef MS_POWER
    // power-cycle the chip
    pinMode(SDA_ARDUINO_PIN, INPUT);
    pinMode(SCL_ARDUINO_PIN, INPUT);
    pinMode(MS_POWER, INPUT);
    idleDelay(30);
    pinMode(MS_POWER, OUTPUT);
    digitalWrite(MS_POWER, HIGH);
#else
    // try to recover by toggling the clock line
    i2c_write(0x00);
    i2c_write(0x00);
    i2c_stop();
#endif
    if (i2c_init()) 
      if (resetAlti()) 
	return;
    errcnt++;
    errcnt = I2C_ERROR;
  }
}

void setup() {
  DEBTTY_INIT();
  DEBLCD_INIT();
  DEBTTY_PRINTLN(F("Initializing ..."));
#ifdef MS_POWER
  delay(20);
  pinMode(MS_POWER, OUTPUT);
  digitalWrite(MS_POWER, HIGH); // power-up the MS5611
  delay(10);
#endif
  if (!i2c_init()) {
    recoverFromI2cProblem();
  }
  if (magickey1 == MKEY1 && 
      eeprom_read_word(&eemagickey2[0]) == MKEY2) {
    DEBTTY_PRINTLN(F("Reset pressed"));
    if (sleeplevel == 1) sleeplevel = 2;
    if (sleeplevel == 3) sleeplevel = 4;
  } else {
    DEBTTY_PRINTLN(F("Startup..."));
    DEBTTY_PRINT(F("Free ram:"));
    DEBTTY_PRINTLN(freeRam());
    sleeplevel = 0;
    resetlevel = 0;
    pressed = false; // startup reset
    bumped = false;
    height = 0;
    lastheight = 0;
    state = SLEEP_STATE;
    laststate = NO_STATE;
    lastpress = 0;
    lastbump = 0;
    lastchange = 0;
    lastmeasure = 0;
    seconds = 0; // before wdt irq is enabled
    magickey1 = MKEY1;
    if (eeprom_read_word(&eemagickey2[0]) != MKEY2) 
      eeprom_write_word(&eemagickey2[0],MKEY2);
    if (eeprom_read_byte(&eelowvolterr[0]) != 0xFF) 
      eeprom_write_byte(&eelowvolterr[0],0xFF);
    refPress = 0.0;
  }
  pinMode(lcd_seg[7],OUTPUT);
  digitalWrite(lcd_seg[7],HIGH); // creates approx 3 mA load
  idleDelay(30);
  volt = readVoltage();
  if (volt < LOWVOLT_ERR) 
    if (readVoltage()  < LOWVOLT_ERR) {
      if (eeprom_read_byte(&eelowvolterr[0]) != 0)
	eeprom_write_byte(&eelowvolterr[0],0);
    } else {
      if (eeprom_read_byte(&eelowvolterr[0]) == 0)
	eeprom_write_byte(&eelowvolterr[0],0xFF);
    }
  digitalWrite(lcd_seg[7],LOW); 
  DEBTTY_PRINT(F("Voltage: "));
  DEBTTY_PRINTLN(volt);

  setupIO();
  if (errcode != I2C_ERROR) {
    resetAlti();
    paramAlti();
  }

  if (eeprom_read_byte(&emeters_up[0]) == 0xFF) {
    dispchar = '.';
    eeprom_write_byte(&emeters_up[0],(uint8_t)DEFAULT_METERS_UP);
    eeprom_write_byte(&eepsilon[0],(uint8_t)DEFAULT_EPSILON);
    byte i = 0;
    do {
      eeprom_write_byte((uint8_t*)&esummit_str[i],(uint8_t)summit_str[i]);
      if (i == MAXSUMSTR) break;
    } while (summit_str[i++]);
    eeprom_write_byte((uint8_t*)&esummit_str[MAXSUMSTR],'\0');
    idleDelay(4000);
    dispchar = ' ';
  }

#ifdef DEB_LED
  digitalWrite(lcd_seg[7],HIGH); 
  idleDelay(500);
  digitalWrite(lcd_seg[7],LOW); 
#endif
#ifdef DEB_LEDRAM
  if (!pressed) displayNum(freeRam());
#endif
#ifdef DEB_LEDBATT
  displayNum(volt);
#endif


#ifdef DEB_LEDHEIGHT
  int frac;
  dispchar = '.';
  refPress = measurePress(25);
  dispchar = ' ';
  displayNum((int)refPress);
  displayChar('.');
  frac = (int)(refPress*10)%10;
  if (frac < 0) frac = -frac;
  displayNum((int)(refPress*10)%10);
#endif

  // initialize parameters from EEPROM
  decimeters_up =  (byte)eeprom_read_byte(&emeters_up[0])*10;
  epsilon =  (byte)eeprom_read_byte(&eepsilon[0])*10;

  // display summit string if last_state == NO_STATE, i.e. fresh start
  if (laststate == NO_STATE) {
    displaySummitString();
    delay(1000);
    displayNum(volt);
  }
}

void loop()
{
  float currPress;
  uint16_t nextmeasure;

#ifdef DEB_LED
    digitalWrite(lcd_seg[7],HIGH); 
    idleDelay(100);
    digitalWrite(lcd_seg[7],LOW); 
    
#endif


  if (pressed) {
    setSeconds(lastpress);
  }

  if (bumped) {
    setSeconds(lastbump);
  }
  
  if ((eeprom_read_byte(&eelowvolterr[0]) == 0) && state > WAKEUP_STATE)
    state = BATT_LOW_STATE;
  if (errcnt >= MAXERRCNT) state = ERROR_STATE;
  if (state > LAST_STATE || state < NO_STATE) state = NO_STATE;
  if (sleeplevel == 5) state = DEEPSLEEP_STATE;
  if (resetlevel >= 5) state = RESET_STATE;

  nextmeasure = getSeconds();
  if ((state == CLIMB_STATE ||
      state == SUMMIT_STATE) && 
      (nextmeasure != lastmeasure || pressed)) {
    lastmeasure = nextmeasure;
    if (nextmeasure%3 == 0 || pressed) { 
      dispchar = '.';
      currPress = measurePressWithRecovery(5);
      dispchar = ' ';
      idleDelay(300);
      lastheight = height;
      height = (int)((refPress-currPress)/0.012);
#if 1
      if ((height < -(5*epsilon) || height > decimeters_up+5*epsilon) &&
	  state == CLIMB_STATE) {
#ifdef DEB_LEDHEIGHT
	displayNum((int)height);
	displayChar(' ');
#endif
        errcnt = MAXERRCNT;
	errcode = HEIGHT_ERROR;
      }
#endif
    }
  }
  if (errcode && errcnt >= MAXERRCNT) state = ERROR_STATE;

  //DEBUG
  //  displayNum(state);
  //  displayNum(sleeplevel);

  DEBLCD_CLEAR();
  DEBLCD_HOME();
  if (laststate != state) {
    DEBTTY_PRINT(F("New state: "));
    DEBTTY_PRINTLN(statestr[state]);
    laststate = state;
    setSeconds(lastchange);
  }
  DEBLCD_PRINTLN(statestr[state]);
  DEBLCD_PRINT(F("Volt:   "));
  DEBLCD_PRINTLN(volt);
  DEBLCD_PRINT(F("Height: "));
  DEBLCD_PRINTLN(height);
  DEBLCD_PRINT(F("Time:   "));
  DEBLCD_PRINTLN(getSeconds());
  DEBLCD_PRINT(F("P/B: "));
  DEBLCD_PRINT(getSeconds()-lastpress);
  DEBLCD_PRINT(F("/"));
  DEBLCD_PRINTLN(getSeconds()-lastbump);
  DEBLCD_PRINT(F("RefP:   "));
  DEBLCD_PRINTLN(refPress);
  DEBLCD_DISPLAY();

  switch(state) {

  case RESET_STATE:
    resetlevel = 0;
    displayPString(reset_str);
    state = WAKEUP_STATE;
    pressed = true;
    break;
    
  case SLEEP_STATE: // sleeping and waiting for a bump or reset to wake up
    if (!pressed) {
      gosleep();
    }
    if (bumped || pressed)  {
      cli();
      seconds = 0;
      lastbump = 0;
      lastpress = 0;
      lastchange = 0;
      sei();
      setupIO();
      dispchar = '.';
      idleDelay(100);
      dispchar = ' ';
      idleDelay(100);
      state = WAKEUP_STATE;
    } 
    break;

  case WAKEUP_STATE: // somebody was moving us 
    if (bumped) {
      	dispchar = '.';
	idleDelay(60);
      	dispchar = ' ';
	idleDelay(60);
    }
    if (pressed) {
      if (eeprom_read_byte(&eelowvolterr[0]) == 0) state = BATT_LOW_STATE;
      else {
	dispchar = '.';
	refPress = measurePressWithRecovery(20);
	currPress = refPress;
	dispchar = ' ';
	idleDelay(200);
	state = CLIMB_STATE;
	if (volt < LOWVOLT_WARN) displayPString(lowbatt_str);
	if (!errcode || errcnt < MAXERRCNT) {
	  resetlevel++;
	  displayPString(go_str);
	  displayChar(' ');
	  if (sleeplevel == 0) sleeplevel = 1;
	  displayNum(int(decimeters_up/CONVFAC));
	  if (sleeplevel == 1) sleeplevel = 0;
	  displayChar(' ');
	  resetlevel = 0;
	  displayPString(meter_str);
	  displayChar(' ');
	  displayPString(up_str);
	}
      }
    } else {
      if (getSeconds() > MAXWAKEUP_TOTAL) state = SLEEP_STATE;
      else if  (getSeconds() - lastbump > MAXWAKEUP_NOBUMP) state = SLEEP_STATE;
    }
    break;

  case CLIMB_STATE: // steadily climbing
#ifndef DEB_LEDSTABLE
    if (decimeters_up-height <= epsilon) {
      if (pressed) displaySummitString();
      state = SUMMIT_STATE;
    } else if (pressed) {
      if (decimeters_up-height >= 0) {
	resetlevel++;
	displayPString(go_str);
	displayChar(' ');
	resetlevel = 0;
	if (sleeplevel == 0) sleeplevel = 1;
	displayNum(int((decimeters_up-height)/CONVFAC));
	displayChar(' ');
	if (sleeplevel == 1) sleeplevel = 0;
	if (sleeplevel == 2) sleeplevel = 3;
	displayPString(meter_str);
	displayChar(' ');
	if (sleeplevel == 3) sleeplevel = 0;
	if (sleeplevel == 4) sleeplevel = 5;
	displayPString(up_str);
	if (sleeplevel == 5) sleeplevel = 0;
      } else {
	state = SUMMIT_STATE;
      }
    } else if (getSeconds() - lastbump > MINQUIET) {
      displayPString(bye_str);
      state = SLEEP_STATE;
    }
    break;
#else
    if (height < 0) {
      displayChar('-');
      height = -height;
    }
    displayNum(height);
    break;
#endif

  case SUMMIT_STATE: // reached the summit
    if (pressed) {
      displaySummitString();
      if (volt < LOWVOLT_WARN) displayPString(lowbatt_str);
    }
    if (height < decimeters_up-epsilon*2) 
      state = CLIMB_STATE;
    break;

  case BATT_LOW_STATE: // if battery is definitely too low
    if (!pressed || lastchange >= lastpress)
      for (int i=0; i<3; i++) displayPString(lowbatt_str);
    state = SLEEP_STATE;
    displayPString(bye_str);
    break;

  case ERROR_STATE: // display error code
    if (!pressed || lastchange >= lastpress) {
      for (int i=0; i<4; i++) {
	displayPString(error_str);
	displayChar(' ');
	displayChar(errcode);
      }
    }
    errcode = NO_ERROR;
    errcnt = 0;
    state = SLEEP_STATE; 
    displayPString(bye_str);
    break;

  case DEEPSLEEP_STATE: // sleep for transport
    displayPString(sleep_str);
    disableVibIRQ();
    state = SLEEP_STATE;
    sleeplevel = 0;
    gosleep();
    errcode = WAKEUP_ERROR;
    errcnt = MAXERRCNT;
    break;

  default: // somehow, the state var got a wrong value
    errcode = STATE_ERROR;
    errcnt = MAXERRCNT;
    break;

  }
  lbumped = bumped;
  pressed = false;
  bumped = false;
  if (laststate == state) {
    IOoff();
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    sleep_mode();
    setupIO();
  }
}

