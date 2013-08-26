/* A sketch that has the mission of sending people up for a couple of meters
   and then bringing them down again
   Uses the MS5611 altimeter/barometer

   Developed on an Arduino Pro Mini, deployed on a Attiny84.
   PCB is the Open-V2 board that fits together with a 3.6V/2400mA lithium
   battery into a preform tube (15cm)
*/
// #define ENGLISH // all messages in English

// #define DEB_LCD
// #define DEB_TTY
// #define DEB_LED
// #define DEB_LEDRAM
// #define DEB_LEDBATT

#define DISPLAY_ON_MSECS 1000 // msecs on
#define DISPLAY_OFF_MSECS 300 // msecs off
#define COORDSHOW 4 // show coords this number of times
#define MAXERRCNT 10 // after that many measurement errors we give up
#define MAXRETRYCNT 5 // number of retries in reset and param command
#define LOWVOLT_WARN 2.8 // warning that battery is low
#define LOWVOLT_ERR 2.2 // with that we do not startup anymore!
#define VOLTSPLIT 4.5
#define VOLTHIGHCOEFF 0.17
#define VOLTLOWCOEFF 0.05

#define METER_UP 6
#define EPSILON 1
#define EPSILON2 3
#define MAXTIME 1800 // 1/2 hrs maxtime
#define MAXWAKEUP_NOBUMP 60 // if there is no bump, we go to sleep again 
#define MAXWAKEUP_TOTAL 360 // 6 minutes for wakeup & start climbing
#define WAKEUP_ASK 120 // 1 minute, then we ask
#define MAXEPSILON2 120 // 2 minutes above epsilon2 measn we are there
#define MINQUIET 90 // 1.5 minutes no bump means we are back in the box
#define MAXCLIMB 900 // 15 minutes to climb to top
#define MAXDISPLAY_SUMMIT_TIME 40 
#define MAXREQUEST_DISPLAY_TIME 45

/* error codes */
#define NO_ERROR 0
#define RESET_ERROR 1
#define PARAM_ERROR 2
#define CONV_ERROR 3
#define TEMP_ERROR 4
#define PRESS_ERROR 5
#define NOBUMP_ERROR 6
#define TOOLONG_ERROR 7
#define STATE_ERROR 8


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
 
   Now connect:
   Pin5 of the first row in the prototype area (PA1 = D1) with SDA MS5611
   Pin4 of the first row (PA2 = D2) with SCL MS5611
   Pin5 of the 8th row (Vcc) with Vcc MS5611
   Pin2 of the 8throw (GND) with GND MS5611

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
#define  __PROG_TYPES_COMPAT__ 1
#include <avr/wdt.h>
#include <avr/sleep.h>
#include <avr/eeprom.h>
#include <avr/pgmspace.h>
#include <I2cMaster.h> // mod. to include the software only version for Attiny

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
#else
#define VOLTOFFSET -0.08
#define POFF 0
#define T1_vect TIM1_OVF_vect
#define PCINT_vect PCINT0_vect
#endif

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

// I2C pins
#define SDA_PIN POFF+2
#define SCL_PIN POFF+1

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
  0, // "#"
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
prog_char summit_str[] PROGMEM = "GOTO N123 E456.";
prog_char lowbatt_str[] PROGMEM = "  BATT LO. ";
prog_char emptybatt_str[] PROGMEM = "BATT EMPTY. ";
prog_char pressme_str[] PROGMEM = "PRESS BUTTON.";
prog_char up_str[] PROGMEM ="UP ";
prog_char meter_str[] PROGMEM =" M.  ";
prog_char down_str[] PROGMEM = "DOWNHILL.";
prog_char bye_str[] PROGMEM = "BYE.";
prog_char error_str[] PROGMEM = "  ERROR";
#else
prog_char summit_str[] PROGMEM = "CODE 456.";
prog_char lowbatt_str[] PROGMEM = "  BATT LO. ";
prog_char emptybatt_str[] PROGMEM = " BATT LEER. ";
prog_char pressme_str[] PROGMEM = "DRUECKEN.";
prog_char up_str[] PROGMEM ="HOCH ";
prog_char meter_str[] PROGMEM =" M. ";
prog_char down_str[] PROGMEM = "RUNTER.";
prog_char bye_str[] PROGMEM = "BYE.";
prog_char error_str[] PROGMEM = "  FEHLER";
#endif



/* states */

#define NO_STATE 0
#define SLEEP_STATE 1
#define WAKEUP_STATE 2
#define PRESS_ME_STATE 3
#define REQUEST_UP_STATE 4
#define CLIMB_STATE 5
#define SUMMIT_STATE 6
#define DESCEND_STATE 7
#define BATT_LOW_STATE 8
#define ERROR_STATE 9
#define LAST_STATE 9


#if defined(ATMEGA) && (defined(DEB_TTY) || defined(DEB_LCD))
char statestr[14][12] = { "UNDEF", "SLEEP", "WAKEUP", "PRESS ME",
		      "REQUEST UP", "CLIMB", "SUMMIT", "DESCEND", 
		      "BATT LOW", "ERROR" };
#endif


/* magic keys - to find out whether reset is a startup */
#define MKEY1 0x7109
#define MKEY2 0xFD1E
#define MKEY3 0x5231

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
// last time, we passed epsilon2
uint16_t lastepsilon2 __attribute__ ((section (".noinit"))); 
// reference pressure for 0 level
float refPress __attribute__ ((section (".noinit"))); 
// actual summit height
float summitheight __attribute__ ((section (".noinit"))); 

uint16_t magickey1 __attribute__ ((section (".noinit"))); 
uint16_t magickey2 __attribute__ ((section (".noinit"))); 

uint16_t eemagickey3[1] EEMEM;
uint8_t eelowvolterr[1] EEMEM;

/***** global vars ****/
int errcnt = 0;
int errcode = 0;
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
SoftI2cMaster baro(SDA_PIN,SCL_PIN);

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

void displayChar(char c) 
{
  dispchar = c;
  delay(DISPLAY_ON_MSECS);
  dispchar = ' ';
  delay(DISPLAY_OFF_MSECS);
}

void displayPString(prog_char *mess)
{
  char c;
  while (c = pgm_read_byte(mess++)) {
    displayChar(c);
  }
  displayChar(' ');
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
  if (!baro.start(ADDRESS | I2C_WRITE)) return ERRORVAL;
  if (!baro.write(code)) return ERRORVAL;
  baro.stop();
  delay(10);
  // start read sequence
  if (!baro.start(ADDRESS | I2C_WRITE)) return ERRORVAL;
  if (!baro.write(0x00)) return ERRORVAL; // request ADC vals
  baro.stop();
  if (!baro.start(ADDRESS | I2C_READ)) return ERRORVAL;
  ret = baro.read(false) * 65536.0 + baro.read(false) * 256.0 + baro.read(true);
  baro.stop();
  return ret;
}

void resetAlti(void)
{
  int retrycnt = 0;
  bool fail = true;
  while (fail && retrycnt++ < MAXRETRYCNT && errcnt < MAXERRCNT) {
    baro.start(0 | I2C_WRITE);
    baro.stop();
    delay(5);
    if (baro.start(ADDRESS | I2C_WRITE)) 
      if (baro.write(0x1E)) {
	baro.stop();
	delay(3);
	fail = false;
      }
    if (fail) {
      errcnt++;
      errcode = RESET_ERROR;
      DEBTTY_PRINTLN(F("Reset command unsuccessfull"))
      delay(5);
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
      if (!baro.start(ADDRESS | I2C_WRITE)) fail = true;
      if (!fail && !baro.write(0xA2 + i*2)) fail = true;
      baro.stop();
      if (!fail && !baro.start(ADDRESS | I2C_READ)) fail = true;
      C[i+1] = baro.read(false) << 8 | baro.read(true);
      baro.stop();
      DEBTTY_PRINTLN(C[i+1])
      if (fail) {
	DEBTTY_PRINTLN(F("Prom command unsuccessfull"))
	errcode = PARAM_ERROR;
	errcnt++;
	resetAlti(); // try another reset
	delay(5);
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


float readVoltage(void)
{
  int reading;
  float result;

#ifdef ATMEGA
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
#else
  ADMUX = _BV(MUX5) | _BV(MUX0);
#endif

  delay(20); // Wait for Vref to settle
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
// is useless under Arduinos bootloader
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
  ADCSRA &= ~(1<<ADEN); // switch off ADC
  stopTimerOne();
}

void setupIO(void)
{
  initIO();
  startTimerOne();
  enablePinChangeIRQ();
  wdt_enable(WDTO_1S);
  WDTCSR |= (1<<WDIE); // enable WDT interrupt instead of reset
  ADCSRA |= (1<<ADEN); // switch on ADC  
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

void setup() {
#if defined(ATMEGA) && defined(DEB_TTY)
  Serial.begin(19200); 
#endif
#if defined(ATMEGA) && defined(DEB_LCD)
  lcd.begin();
#endif

  if (magickey1 == MKEY1 && 
      magickey2 == MKEY2 && 
      eeprom_read_word(&eemagickey3[0]) == MKEY3) {
    DEBTTY_PRINTLN(F("Reset pressed"));
  } else {
    DEBTTY_PRINTLN(F("Startup..."));
    DEBTTY_PRINT(F("Free ram:"));
    DEBTTY_PRINTLN(freeRam());
    pressed = false; // startup reset
    bumped = false;
    state = SLEEP_STATE;
    laststate = NO_STATE;
    lastpress = 0;
    lastbump = 0;
    lastchange = 0;
    lastepsilon2 = 0;
    seconds = 0; // before wdt irq is enabled
    magickey1 = MKEY1;
    magickey2 = MKEY2;
    if (eeprom_read_word(&eemagickey3[0]) != MKEY3) 
      eeprom_write_word(&eemagickey3[0],MKEY3);
    if (eeprom_read_byte(&eelowvolterr[0]) != 0xFF) 
      eeprom_write_byte(&eelowvolterr[0],0xFF);
  }
  pinMode(lcd_seg[7],OUTPUT);
  digitalWrite(lcd_seg[7],HIGH); // creates approx 3 mA load
  delay(50);
  volt = readVoltage();
  if (volt < LOWVOLT_ERR) 
    if (readVoltage()  < LOWVOLT_ERR) 
      if (eeprom_read_byte(&eelowvolterr[0]) != 0)
	eeprom_write_byte(&eelowvolterr[0],0);
  digitalWrite(lcd_seg[7],LOW); 
  DEBTTY_PRINT(F("Voltage: "));
  DEBTTY_PRINTLN(volt);

  setupIO();
  resetAlti();
  paramAlti();
#ifdef DEB_LED
  digitalWrite(lcd_seg[7],HIGH); 
  delay(500);
  digitalWrite(lcd_seg[7],LOW); 
#endif
#ifdef DEB_LEDRAM
  if (!pressed) displayNum(freeRam());
#endif
#ifdef DEB_LEDBATT
  displayNum((int)(volt*100));
#endif


}

void loop()
{
  float currPress = refPress;
  float height;

#ifdef DEB_LED
    digitalWrite(lcd_seg[7],HIGH); 
    delay(100);
    digitalWrite(lcd_seg[7],LOW); 
#endif

  if (pressed) {
    setSeconds(lastpress);
  }

  if (bumped) {
    setSeconds(lastbump);
  }
  
  if (state >= REQUEST_UP_STATE && state <= DESCEND_STATE) 
    currPress = measurePress(5);
  height = (refPress-currPress)/0.12;

  if ((eeprom_read_byte(&eelowvolterr[0]) == 0) && state > WAKEUP_STATE)
    state = BATT_LOW_STATE;
  if (getSeconds() > MAXTIME) {
    errcode = TOOLONG_ERROR;
    errcnt = MAXERRCNT;
  }
  if (errcnt >= MAXERRCNT) state = ERROR_STATE;
  if (state > LAST_STATE || state < NO_STATE) state = NO_STATE;
    
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
      wdt_disable(); // no more seconds counting
      IOoff(); // no more IO
      set_sleep_mode(SLEEP_MODE_PWR_DOWN);
      sleep_mode(); // sleep & wait for bump or reset
    }
    if (bumped || pressed) {
      cli();
      seconds = 0;
      lastbump = 0;
      lastpress = 0;
      lastchange = 0;
      lastepsilon2 = 0;
      summitheight = METER_UP;
      sei();
      state = WAKEUP_STATE;
      setupIO();
      refPress = measurePress(30);
    } 
    break;

  case WAKEUP_STATE: // somebody was moving us 
    if (bumped) {
      digitalWrite(lcd_seg[7],HIGH); 
      delay(200);
      digitalWrite(lcd_seg[7],LOW); 
      delay(20);
      refPress = measurePress(30);
    }
    if (pressed) {
      if (eeprom_read_byte(&eelowvolterr[0]) == 0) state = BATT_LOW_STATE;
      else {
	state = REQUEST_UP_STATE;
	if (volt < LOWVOLT_WARN) displayPString(lowbatt_str);
      }
    } else {
      if (getSeconds() > MAXWAKEUP_TOTAL) state = SLEEP_STATE;
      else if (getSeconds() - lastbump > MAXWAKEUP_NOBUMP) state = SLEEP_STATE;
      else if (getSeconds() - lastchange > WAKEUP_ASK) state = PRESS_ME_STATE;
    }
    break;

  case PRESS_ME_STATE: // if wakeup is too long
    if (pressed) state = REQUEST_UP_STATE;
    for (int i=0; i < 3; i++) {
      displayPString(pressme_str);
    }
    state = SLEEP_STATE;
    break;

  case REQUEST_UP_STATE: // ask for getting up
    if (!pressed && getSeconds() - lastchange < MAXREQUEST_DISPLAY_TIME) {
      displayPString(up_str);
      displayNum(METER_UP);
      displayPString(meter_str);
    } else state = CLIMB_STATE;
    break;

  case CLIMB_STATE: // steadily climbing
    if (lastepsilon2 == 0 && height >= METER_UP-EPSILON2)
      setSeconds(lastepsilon2);
    if (height >= METER_UP-EPSILON) {
      state = SUMMIT_STATE;
      summitheight = height;
    } else if (height >= METER_UP-EPSILON2 && 
	       lastepsilon2 > 0 && 
	       getSeconds() - lastepsilon2 > MAXEPSILON2) {
      state = SUMMIT_STATE;
      summitheight = height;
    } else if (pressed) {
      if ((int)METER_UP-height > 0) {
	displayPString(up_str);
	displayNum(METER_UP-height);
	displayPString(meter_str);
      } else {
	state = SUMMIT_STATE;
      }
    } else if (height <= EPSILON2 && getSeconds() > MAXCLIMB ||
	       (getSeconds() - lastbump > MINQUIET*2 &&
		getSeconds() - lastpress > MINQUIET*2)) {
      displayPString(bye_str);
      state = SLEEP_STATE;
    }
    break;

  case SUMMIT_STATE: // reached the summit
    if ((pressed || (getSeconds() - lastchange < MAXDISPLAY_SUMMIT_TIME)) && 
	(height >= summitheight - EPSILON2)) {
      displayPString(summit_str);
      if (volt < LOWVOLT_WARN) displayPString(lowbatt_str);
    }
    if (height < summitheight-EPSILON2) 
      state = DESCEND_STATE;
    break;

  case DESCEND_STATE: // steadily going down
    if (height >= summitheight - EPSILON) state = SUMMIT_STATE;
    else if (pressed) {
      if (height - EPSILON <= 0) displayPString(bye_str);
      else displayPString(down_str);
    } else if (getSeconds() - lastbump > MINQUIET && 
	     getSeconds() - lastpress > MINQUIET) {
      if (height - EPSILON2 <= 0) state = SLEEP_STATE;
    }
    break;

  case BATT_LOW_STATE: // if battery is definitely too low
    if (!pressed || lastchange >= lastpress)
      for (int i=0; i<3; i++) displayPString(emptybatt_str);
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

