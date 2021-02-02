// -*- c++ -*-
// Sketch to test measure height using an MS5611 

#define USEDOTMATRIX 
#include <stdio.h>
#include <PETPreformBoard.h>
#include <SoftI2CMaster.h>
#include <DotMatrix5x7.h>
#include <MyMS5611.h>
#include <ArduinoSort.h>

#define ONTIME 500
#define OFFTIME 100
#define HEIGHT 230
#define MAXSAMPLE 100
#define THROWAWAY 25
#define FPS 50

MyMS5611 sensor;
float temp;
float refPress;
float sample[MAXSAMPLE];
float cf;
char str[30];

void setup()
{
#if defined(__AVR_ATtiny1634__)
  Serial.begin(2400);
  UCSR0B&=~(1<<RXEN0); // disable RX
#else
  Serial.begin(57600);
#endif
  pinMode(POWER, OUTPUT);
  digitalWrite(POWER, HIGH);
  delay(2);
  Serial.println(F("\nmeasureHeight setup..."));
#ifdef USEDOTMATRIX
  Dot5x7.begin(DMCOL1, DMCOL2, DMCOL3, DMCOL4, DMCOL5,                  // column pins
	       DMROW1, DMROW2, DMROW3, DMROW4, DMROW5, DMROW6, DMROW7,  // row pins
	       LOW,                   // value when row pin is active (default value)
	       HIGH);                 // value when column pin is active (default value)
  Dot5x7.setFramesPerSecond(FPS);      // display 50 frames per second (default value)
#endif
  if(sensor.connect()>0) {
    Serial.println(F("Error connecting..."));
    while (1);
  }
  sensor.ReadProm();
  Serial.print(F("PROM-Id: 0x"));
  Serial.println(sensor.Read_C(0), HEX);
  if (sensor.Calc_CRC4() == sensor.Read_CRC4()) {
    Serial.println(F("CRC OK"));
  } else {
    Serial.println(F("CRC failed"));
    while (1);
  }
  Dot5x7.setBlinkFrames(ONTIME/FPS,ONTIME/FPS);
  Dot5x7.show('!');
  temp = measure(false); // in 1/100 degree C
  Dot5x7.setBlinkFrames(0,0);
  sprintf(str, "Temp: %i °C", (long)(temp/100));
  Serial.println(str);
  refPress = measure(true); // in Pascal
  sprintf(str, "refPress: %i Pa", (long)refPress);
  Serial.println(str);
  cf = convFactor(HEIGHT,temp/100);
  sprintf(str, "convFactor: %i", (long)(cf*100));
  Serial.println(str);
}

void loop()
{
  float press;
  int dm;
  int fac;
  Dot5x7.setBlinkFrames(ONTIME/FPS,ONTIME/FPS);
  Dot5x7.show('.');
  press = measure(true);
  Dot5x7.setBlinkFrames(0,0);
  dm = (refPress-press)/cf;
  sprintf(str, "Press: %i Pa", (long)press);
  sprintf(str, "Press diff: %i Pa", (long)(refPress-press));
  Serial.println(str);
  sprintf(str, "Height: %d dm", dm);
  Serial.println(str);
  if (dm < 0) {
    dm = -dm;
    Dot5x7.show('-');
    delay(ONTIME);
    Dot5x7.clear();
    delay(OFFTIME);
  }
  fac = 10;
  while (fac <= dm) fac *= 10;
  while (fac > 1) {
    dm = dm%fac;
    fac = fac/10;
    Dot5x7.show((dm/fac)+'0');
    delay(ONTIME);
    Dot5x7.clear();
    delay(OFFTIME);
  }
}

float measure(bool mpress) {
  int cnt = 0;
  int i = 0;
  float avg = 0;

  Serial.print(F("\nMeasuring "));
  if (mpress) Serial.println(F(" pressure ..."));
  else Serial.println(F(" temperature ..."));
  while (cnt < MAXSAMPLE) {
    sensor.Readout();
    sample[cnt] = (mpress ? sensor.GetPres() : sensor.GetTemp());
    if (((sample[cnt] < 85000.0 || sample[cnt] > 110000.0) && mpress) ||
	((sample[cnt] < -5000.0 || sample[cnt] > 8000.0) && !mpress)) {
      Serial.print(F("Outlier: "));
      Serial.print((int)(sample[cnt]/100));
      if (mpress) Serial.println(F("mBar"));
      Serial.println(F("°C"));
    } else {
#if 1
      cnt++;
#else
      avg += sample[cnt];
      i++;
      if (i >= 200) break;
#endif
    }
  }
#if 1
  sortArray(sample, MAXSAMPLE);
  sprintf(str, "Large span: %i", (long)sample[MAXSAMPLE-1]-sample[0]);
  Serial.println(str);
  avg = 0;
  for (i=0; i < MAXSAMPLE; i++) avg += sample[i];
  avg = avg / MAXSAMPLE;
  sprintf(str,"Large avg:  %i", (long)avg);
  Serial.println(str);
  sprintf(str, "Small span: %i", (long)(sample[MAXSAMPLE-1-THROWAWAY]-sample[THROWAWAY]));
  Serial.println(str);
  avg = 0;
  cnt = 0;
  for (i=THROWAWAY; i < MAXSAMPLE-THROWAWAY; i++) {
    avg += sample[i];
    cnt++;
  }
  avg = avg / cnt;
  sprintf(str, "Small avg:  %i", (long)avg);
  Serial.println(str);
#endif
  return avg;
}

float convFactor(int height, float temp)
{
  return(10/(7.9+0.0008*height+0.03*temp));
}
