// -*-c++-*-

/* A sketch that has the mission of sending people up for a couple of meters
   and then bringing them down again
   Uses the MS5611 altimeter/barometer

   Developed on an Arduino Pro Mini, deployed on a Attiny84.
   PCB is the Open-V2 board that fits together with a 3.6V/2400mA lithium
   battery into a preform tube (15cm)

   ATtiny Fusebits: Divide clock by 8, Brown-out disabled
   Arduino board: ATtiny 84 / 1MHz
*/

#define Version "2.8"
// #define OLD_I2C_WIRING // define if first board (= dragon) is used!

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
 *
 */
#include <avr/wdt.h>
#include <avr/sleep.h>
#include <avr/eeprom.h>
#include <avr/pgmspace.h>


#define DEFAULT_METERS_UP 70 // meters. you have to walk up
#define DEFAULT_EPSILON 5  // potential error
#define DEFAULT_SUMMIT_STR "CODE 132" // message displayed on summit, max 30 chars

#define MAXSUMSTR 200 // should be less than 512-6

uint8_t emeters_up[1] EEMEM;
uint8_t eepsilon[1] EEMEM;
char esummit_str[MAXSUMSTR+1] EEMEM;
uint16_t eemagickey2[1] EEMEM;
uint8_t eelowvolterr[1] EEMEM;

#define ENGLISH // all messages in English
#define IMPERIAL // measures in feet

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

#define MAXCHANGE 40 // if the height change is > 4 meters, we do a second measurement!
#define DISPLAY_ON_MSECS 1000 // msecs on
#define DISPLAY_OFF_MSECS 300 // msecs off
#define MAXERRCNT 10 // after that many measurement errors we give up
#define MAXRETRYCNT 5 // number of retries in reset and param command
#define LOWVOLT_WARN 2.8 // warning that battery is low
#define LOWVOLT_ERR 2.3 // with that we do not startup anymore!
#define VOLTSPLIT 4.5
#define VOLTHIGHCOEFF 0.17
#define VOLTLOWCOEFF 0.05

#define MAXWAKEUP_NOBUMP 120 // if there is no bump, we go to sleep again 
#define MAXWAKEUP_TOTAL 480 // 8 minutes for wakeup & start climbing
#define MINQUIET 300 // 5 minutes no bump means we are back in the box

/* error codes */
#define NO_ERROR 0
#define RESET_ERROR 1
#define PARAM_ERROR 2
#define CONV_ERROR 3
#define TEMP_ERROR 4
#define PRESS_ERROR 5
#define NOBUMP_ERROR 6
#define HEIGHT_ERROR 7
#define WAKEUP_ERROR 8
#define STATE_ERROR 9


#if (__AVR_ARCH__  == 5) // means ATMEGA 
#define ATMEGA
#endif

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
   - and the bottom I2C ADDR bridge!

   On breadboard connect:
   Attiny-Pin   Arduino(Attiny)   Arduino(ProMini)     External
   PA0          D0                D2                   Vibration switch
   PA1          D1                D3                   MS5611 SDA
   PA2          D2                D4                   MS5611 SCL
                                                       MS5611 Vcc
						       MS5611 GND
   PA3          D3                D5                   Disp. Pin1(g) 
   PA4          D4                D6                   Disp. Pin2(f) + ISP CLK
   PA5          D5                D7                   Disp. Pin4(e) + ISP MISO
   PA6          D6                D8                   Disp. Pin5(d) + ISP MOSI
   PA7          D7                D9                   Disp. Pin6(DP)
   PB2          D8                D10                  Disp. Pin7(c)
   PB1          D9                D11                  Disp. Pin9(b)
   PB0          D10               D12                  Disp. Pin10(a)
                                                       Disp. Pin8 or 3 to R1
                                  D13                  LCD   SCLK
				  D14                  LCD   DIN
				  D15                  LCD   D/C
				  D16                  LCD   RST
				  D17                  LCD   CS
				                       LCD   Vcc
						       LCD   GND
*/

#if defined(ATMEGA) && defined(DEB_LCD)
#include <Adafruit_GFX.h>
#include <Adafruit_PCD8544.h>
Adafruit_PCD8544 lcd = Adafruit_PCD8544(13,14,15,17,16); 
#endif

// differences between Atmega and Attiny
#ifdef ATMEGA
#define VOLTOFFSET -0.06
#define POFF 2
#define T1_vect TIMER1_OVF_vect
#define PCINT_vect PCINT2_vect
#define SDA_PORT PORTD
#define SDA_PIN 3
#define SCL_PORT PORTD
#define SCL_PIN 4
#else
#define VOLTOFFSET -0.08
#define POFF 0
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
#endif

#include <SoftI2CMaster.h>

#if  defined(DEB_TTY) && defined(ATMEGA)
#define DEBTTY_PRINT(str) \
  Serial.print(str);
#define DEBTTY_PRINTLN(str) \
  Serial.println(str);
#else
#define DEBTTY_PRINT(str)
#define DEBTTY_PRINTLN(str)
#endif

#if  defined(DEB_LCD) && defined(ATMEGA)
#define DEBLCD_PRINT(str) \
  lcd.print(str);
#define DEBLCD_PRINTLN(str) \
  lcd.println(str);
#define DEBLCD_HOME() \
  lcd.setCursor(0,0);
#define DEBLCD_DISPLAY() \
  lcd.display();
#define DEBLCD_CLEAR() \
  lcd.clearDisplay();
#else
#define DEBLCD_PRINT(str)
#define DEBLCD_PRINTLN(str)
#define DEBLCD_HOME()
#define DEBLCD_DISPLAY()
#define DEBLCD_CLEAR()
#endif

// 8-bit I2C address of MS5611 (CSB connected to Vcc)
#define ADDRESS 0xEC

// error value for measurements
#define ERRORVAL -9999.0

// LCD segment definitions. 
// These will need to be changed depending on the 
// wiring of your output port to the segements.
#define SEGa (1<<(10+POFF))
#define SEGb (1<<(9+POFF))
#define SEGc (1<<(8+POFF))
#define SEGd (1<<(6+POFF))
#define SEGe (1<<(5+POFF))
#define SEGf (1<<(4+POFF))
#define SEGg (1<<(3+POFF))
#define SEGdp (1<<(7+POFF))

const uint8_t lcd_seg[] =
  { (10+POFF), (9+POFF), (8+POFF), (6+POFF), (5+POFF), (4+POFF), (3+POFF), (7+POFF) };


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
  SEGa+SEGb,
  SEGb+SEGg,
  SEGg+SEGe,
  SEGe+SEGd,
  SEGd+SEGc,
  SEGc+SEGg,
  SEGg+SEGf,
  SEGf+SEGa,
};
#define LASTCHAR 'Z'

const char circle[] = { LASTCHAR+1, LASTCHAR+2, LASTCHAR+3, LASTCHAR+4, 
			LASTCHAR+5, LASTCHAR+6, LASTCHAR+7, LASTCHAR+8 };

/* messages */
#ifdef ENGLISH
const char lowbatt_str[] PROGMEM = "  BATT LO. ";
const char go_str[] PROGMEM = "GO";
#ifdef IMPERIAL
const char up_str[] PROGMEM ="UP";
const char meter_str[] PROGMEM ="FEET";
#else
const char up_str[] PROGMEM ="UP";
const char meter_str[] PROGMEM ="M";
#endif
const char bye_str[] PROGMEM = "BYE.";
const char error_str[] PROGMEM = "  ERROR";
const char sleep_str[] PROGMEM = "SP";
#else
const char lowbatt_str[] PROGMEM = "  BATT LO. ";
const char go_str[] PROGMEM = "";
const char up_str[] PROGMEM ="HOCH LAUFEN";
const char meter_str[] PROGMEM ="M";
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
#define LAST_STATE 7


#if defined(ATMEGA) && (defined(DEB_TTY) || defined(DEB_LCD))
char statestr[14][12] = { "UNDEF", "SLEEP", "WAKEUP", 
			  "CLIMB", "SUMMIT", "BATT_LOW", "ERROR", 
			  "DEEP_SLEEP" };
#endif


/* magic keys - to find out whether reset is a startup */
#define MKEY1 0x7109
#define MKEY2 0xFD1E

/***** super global variables (surviving resets) *****/
// The state var
uint8_t state __attribute__ ((section (".noinit")));
// value of start variable last time
uint8_t laststate __attribute__ ((section (".noinit")));
// This counter is advanced by the wdt interrupt - started after wakeup
volatile uint16_t seconds __attribute__ ((section (".noinit"))); 
// Keeping track when last bump happened (using secods)
uint16_t lastbump __attribute__ ((section (".noinit"))); 
// last time the reset button was pressed
uint16_t lastpress __attribute__ ((section (".noinit"))); 
// last time, the state was changed
uint16_t lastchange __attribute__ ((section (".noinit"))); 
// last time, we measured the pressure
uint16_t lastmeasure __attribute__ ((section (".noinit"))); 
// reference pressure for 0 level
float refPress __attribute__ ((section (".noinit"))); 
// measurement converted to height above zero level
int height __attribute__ ((section (".noinit")));
// last height when making measurement
int lastheight __attribute__ ((section (".noinit")));

uint8_t sleeplevel __attribute__ ((section (".noinit")));

uint16_t magickey1 __attribute__ ((section (".noinit"))); 





/***** global vars ****/
char summit_str[31] = DEFAULT_SUMMIT_STR; // will be initialized in setup!
int errcnt = 0;
int errcode = 0;
int decimeters_up; // will be initialized from EEPROM
int epsilon; // will be initialized from EEPROM
float volt;
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

ISR(PCINT_vect)
{
  bumped = true;
}

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

void resetAlti(void)
{
  int retrycnt = 0;
  bool fail = true;
  while (fail && retrycnt++ < MAXRETRYCNT && errcnt < MAXERRCNT) {
    i2c_start(0 | I2C_WRITE);
    i2c_stop();
    idleDelay(5);
    if (i2c_start(ADDRESS | I2C_WRITE)) 
      if (i2c_write(0x1E)) {
	i2c_stop();
	idleDelay(6);
	fail = false;
      }
    if (fail) {
      errcnt++;
      errcode = RESET_ERROR;
      DEBTTY_PRINTLN(F("Reset command unsuccessfull"))
	idleDelay(10);
    }
  }
  if (fail) {
    errcnt = MAXERRCNT;
    DEBTTY_PRINTLN(F("*** Fatal error in reset command exec"))
  }
}

void paramAlti(void)
{
  bool fail = true;
  int retrycnt = 0;
  DEBTTY_PRINTLN("")
  DEBTTY_PRINTLN(F("PROM COEFFICIENTS"))
  while (fail && retrycnt++ < MAXRETRYCNT && errcnt < MAXERRCNT) {
    fail = false;
    for (int i=0; i<6 && !fail ; i++) {
      if (!i2c_start(ADDRESS | I2C_WRITE)) fail = true;
      if (!fail && !i2c_write(0xA2 + i*2)) fail = true;
      i2c_stop();
      if (!fail && !i2c_start(ADDRESS | I2C_READ)) fail = true;
      C[i+1] = i2c_read(false) << 8 | i2c_read(true);
      i2c_stop();
      DEBTTY_PRINTLN(C[i+1])
      if (fail) {
	DEBTTY_PRINTLN(F("Prom command unsuccessfull"))
	errcode = PARAM_ERROR;
	errcnt++;
	resetAlti(); // try another reset
	idleDelay(10);
      }
    }
    DEBTTY_PRINTLN("")
  }
  if (fail) {
    errcnt = MAXERRCNT;
    DEBTTY_PRINTLN(F("*** Fatal error in prom command"))
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

  if (count <= 0) count = 1;
  for (int i = 0; i < count && errcnt < MAXERRCNT; i++) {
    D1 = getAltiVal(0x48); // Pressure raw
    D2 = getAltiVal(0x58);// Temperature raw

    /*
    DEBTTY_PRINT(F("RAW PRESS: "))
    DEBTTY_PRINTLN(D1)
    DEBTTY_PRINT(F("RAW TEMP: "))
    DEBTTY_PRINTLN(D2)
    */

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
      }
      P = (D1 * SENS / 2097152 - OFF) / 32768 / 100.0;
      /*
      DEBTTY_PRINT(F("TEMP="))
      DEBTTY_PRINTLN(TEMP)
      DEBTTY_PRINT(F("P="))
      DEBTTY_PRINTLN(P)
      */
      if (P < 600.0 || P > 1200.0) { // almost impossible pressures up to 2000 meters elevation
	errcnt++;
	errcode = PRESS_ERROR;
      }
      acc  += P;
    } else {
      errcnt++;
      errcode = CONV_ERROR;
      DEBTTY_PRINTLN(F("Measure command unsuccessful"))
      if (errcnt < MAXERRCNT) { // reset the chip again
	resetAlti();
	i--; // redo last measurement
      } else {
	DEBTTY_PRINTLN(F("*** Fatal abort because too many measurement errors"))
	return(0);
      }
    }
  }
  return (acc/count);
}

float measurePressWithRecovery(int count)
{
  float result;
  bool cont = true;
  int retrycnt = 3;

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


float readVoltage(void)
{
  int reading;
  float result;

  ADCSRA |= (1<<ADEN); // switch on ADC  
  idleDelay(20);

#ifdef ATMEGA
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
#else
  ADMUX = _BV(MUX5) | _BV(MUX0);
#endif

  idleDelay(20); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Convert
  while (bit_is_set(ADCSRA,ADSC));
  reading = ADCL;
  reading |= ADCH<<8;
  result = 1126.4 / reading; // Back-calculate AVcc in mV
  // correction:
  if (result > VOLTSPLIT) result = result - (result - VOLTSPLIT)*VOLTHIGHCOEFF;
  else result = result + (VOLTSPLIT - result)*VOLTLOWCOEFF;
  result += VOLTOFFSET;

  DEBTTY_PRINT(F("Raw voltage: "))
  DEBTTY_PRINTLN(reading)
  DEBTTY_PRINT(F("Voltage:  "))
  DEBTTY_PRINTLN(result)

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
void wdt_init(void) __attribute__((naked)) __attribute__((section(".init3")));
void wdt_init(void)
{
  MCUSR = 0;
  wdt_disable();
  return;
} 


void enablePinChangeIRQ(void) 
{
#ifdef ATMEGA
  PCICR |= (1<<PCIE2); // allow for PCINT on PCINT16-23
  PCMSK2 |= (1<<PCINT18); // on pin PD2 = PCINT18 = dig. pin 2
#else
  GIMSK |= (1<<PCIE0);  // allow for PCINT on PA0-PA7
  PCMSK0 |= (1<<PCINT0); // on pin PA0 = ADC0 = analog0 = dig. pin 0
#endif
}

void disablePinChangeIRQ(void) 
{
#ifdef ATMEGA
  PCICR &= ~(1<<PCIE2); // disable PCINT on PCINT16-23
#else
  GIMSK &= ~(1<<PCIE0);  // disable PCINT on PA0-PA7
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
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_mode(); // sleep & wait for reset
}

void setup() {
#if defined(ATMEGA) && defined(DEB_TTY)
  Serial.begin(19200); 
#endif
#if defined(ATMEGA) && defined(DEB_LCD)
  lcd.begin();
#endif
  i2c_init();
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
    if (readVoltage()  < LOWVOLT_ERR && eeprom_read_byte(&eelowvolterr[0]) != 0)
	eeprom_write_byte(&eelowvolterr[0],0);
  digitalWrite(lcd_seg[7],LOW); 
  DEBTTY_PRINT(F("Voltage: "));
  DEBTTY_PRINTLN(volt);

  setupIO();

  resetAlti();
  paramAlti();


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
  displayNum((int)(volt*100));
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
  if (laststate == NO_STATE) displaySummitString();
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
      if (abs(height-lastheight) > MAXCHANGE) {
	currPress = measurePressWithRecovery(10);
	height = (int)((refPress-currPress)/0.012);
      }
#if 1
      if ((height < -(3*epsilon) || height > decimeters_up+3*epsilon) &&
	  state == CLIMB_STATE) {
#ifdef DEB_LEDHEIGHT
	displayNum((int)height);
	displayChar(' ');
#endif
	state = ERROR_STATE;
	errcode = HEIGHT_ERROR;
      }
#endif
    }
  }


  //DEBUG
  //  displayNum(state);
  //  displayNum(sleeplevel);

  DEBLCD_CLEAR()
  DEBLCD_HOME()
  if (laststate != state) {
    DEBTTY_PRINT(F("New state: "))
    DEBTTY_PRINTLN(statestr[state])
    laststate = state;
    setSeconds(lastchange);
  }
  DEBLCD_PRINTLN(statestr[state])
  DEBLCD_PRINT(F("Volt:   "))
  DEBLCD_PRINTLN(volt)
  DEBLCD_PRINT(F("Height: "))
  DEBLCD_PRINTLN(height)
  DEBLCD_PRINT(F("Time:   "))
  DEBLCD_PRINTLN(getSeconds())
  DEBLCD_PRINT(F("P/B: "))
  DEBLCD_PRINT(getSeconds()-lastpress)
  DEBLCD_PRINT(F("/"))
  DEBLCD_PRINTLN(getSeconds()-lastbump)
  DEBLCD_PRINT(F("RefP:   "))
  DEBLCD_PRINTLN(refPress)
  DEBLCD_DISPLAY()

  switch(state) {

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
	displayPString(go_str);
	displayChar(' ');	
	if (sleeplevel == 0) sleeplevel = 1;
	displayNum(int(decimeters_up/CONVFAC));
	if (sleeplevel == 1) sleeplevel = 0;
	displayChar(' ');
	displayPString(meter_str);
	displayChar(' ');
	displayPString(up_str);
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
	displayPString(go_str);
	displayChar(' ');	
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
	displayNum(errcode);
      }
    }
    errcode = NO_ERROR;
    errcnt = 0;
    state = SLEEP_STATE; 
    displayPString(bye_str);
    break;

  case DEEPSLEEP_STATE: // sleep for transport
    displayPString(sleep_str);
    disablePinChangeIRQ();
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

