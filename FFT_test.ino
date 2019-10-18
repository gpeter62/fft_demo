/*

ESP 32 FFT Audio Spectrum Analyser
20Hz-20kHz   1/3 octave steps   25x10 LED pixel
1024 bins
  
        Copyright (C) 2018 Peter Gautier

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.


Screen refresh about 200msec

PixelPin = 10, 250 programmable LEDs connected
*/
int programMode = 1;    //1=Spectrum, 2=GameOfLife 9=input test

//*********************************** OTA *******************************************
#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>

const char* ssid = "farm32";
const char* password = "birka12345";
//***********************************************************************************
#include <Wire.h>
//#include "SSD1306.h"  // https://github.com/squix78/esp8266-oled-ssd1306
//SSD1306 display(0x3c, 21,22);  // 0.96" OLED display object definition (address, SDA, SCL) Connect OLED SDA , SCL pins to ESP SDA, SCL pins
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include <SPI.h>
#include <SD.h>
//***********************************************************************************
#define CHANNEL A0
#define POT1  A3
#define POT2  A6
#define POT3  A7
#define BUT1  A4
#define BUT2  A5
#define DIP1  26    //OTA bekapcsolva
#define DIP2  25    //input POT + SWITCH test!
//***********************************************************************************
//define pins of TFT screen
#define TFT_CS     12 //15
#define TFT_RST    14 //4 
#define TFT_DC     13 //12
#define TFT_SCLK   22 //14
#define TFT_MOSI   21 //13
//define pins of SD card slot
#define SD_CS    5
#define SD_MOSI  23
#define SD_MISO  19
#define SD_SCK   18
Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST);
//*************************************************************************************
#include <NeoPixelBus.h>
const uint16_t PixelCount = 250; // this example assumes 4 pixels, making it smaller will cause a failure
const uint8_t PixelPin = 10;  // make sure to set this to the correct pin, ignored for Esp8266

#define colorSaturation 50

// three element pixels, in different order and speeds
NeoPixelBus<NeoGrbFeature, NeoEsp32I2s1800KbpsMethod> strip(PixelCount, PixelPin);

RgbColor red(colorSaturation, 0, 0);
RgbColor red2(colorSaturation/10, 0, 0);
RgbColor green(0, colorSaturation, 0);
RgbColor green2(0, colorSaturation/10, 0);
RgbColor blue(0, 0, colorSaturation);
RgbColor blue2(0, 0, colorSaturation/10);
RgbColor white(colorSaturation/2,colorSaturation,colorSaturation/2);
RgbColor black(0);
RgbColor orange(2*colorSaturation, colorSaturation, 0);
RgbColor yellow(colorSaturation/2, colorSaturation, 0);
//************************************************************************************************
#include "arduinoFFT.h"
arduinoFFT FFT = arduinoFFT(); /* Create FFT object */

const uint16_t samples = 1024; //This value MUST ALWAYS be a power of 2
const double samplingFrequency = 44000; //Hz, must be less than 10000 due to ADC
unsigned int sampling_period_us;
unsigned long microseconds;
double bin_size;
double BIN_MAX = 240000;
int offset = 1757;    //middle DC level reading
#define BAND_MAX 25   //0..BAND_MAX - 1 are the values
#define BAND_LENGTH 1

// 1/3 ocatave frequency scale   (lower 8 values are missing)
int limit[] = {50,100,125,160,200,250,315,400,500,630,800,
              1000,1250,1600,2000,2500,3150,4000,5000,6300,8000,
              10000,12500,16000,20000};
int bin_band[samples];

#define XMAX 25
#define YMAX 10

double LEDlimit[YMAX] = {
  0.100,    // 0  -20dB
  0.126,    // 1  -18dB
  0.178,    // 2  -15dB   
  0.251,    // 3  -12dB
  0.355,    // 4  - 9dB
  0.501,    // 5  - 6dB
  0.708,    // 6  - 3dB
  0.891,    // 7  - 1dB
  1,        // 8    0dB
  1.414     // 9   +3dB and up
};

/*
These are the input and output vectors
Input vectors receive computed results from FFT
*/
byte peak[128];
double vReal[samples];
double vImag[samples];

double volume = 0; //-10..+10

//--------- GAME OF LIFE -----------------
int gen = 0;
int maxGen = 300;
unsigned long delaytime=300;

#define XMAX  25
#define YMAX  10

byte gameBoard[XMAX][YMAX];
byte newGameBoard[XMAX][YMAX];
//-----------------------------------------

void PotButTest() {
  while (true) {
    Serial.print("Audio:"); Serial.println(analogRead(CHANNEL));
    Serial.print("POT1 :"); Serial.println(analogRead(POT1));
    Serial.print("POT2 :"); Serial.println(analogRead(POT2));
    Serial.print("POT3 :"); Serial.println(analogRead(POT3));
    Serial.print("BUT1 :"); Serial.println(digitalRead(BUT1));
    Serial.print("BUT2 :"); Serial.println(digitalRead(BUT2)); 
    Serial.print("DIP1 :"); Serial.println(digitalRead(DIP1));
    Serial.print("DIP2 :"); Serial.println(digitalRead(DIP2));
    Serial.println("-------------------------------");
    delay(2000);   
  }
}

void setup() {
  delay(100);
  Serial.begin(115200);
  delay(100);
  if (digitalRead(DIP2) == LOW) startOTA();
  gen = 0;
  resetMap();
  pinMode(BUT1,INPUT_PULLUP);
  pinMode(BUT2,INPUT_PULLUP);
  pinMode(DIP1,INPUT_PULLUP);
  pinMode(DIP2,INPUT_PULLUP);
  delay(100);
  if (digitalRead(DIP2) == LOW) PotButTest();
  sampling_period_us = round(1000000*(1.0/samplingFrequency));
  bin_size = (1.0 * samplingFrequency) / samples;
  strip.Begin();
  strip.Show();
  delay(200);

  delay(500);
  //Wire.begin(21,22); // SDA, SCL
  //delay(10);
  //display.init();
  //display.setFont(ArialMT_Plain_10);
  //display.flipScreenVertically(); // Adjust to suit or remove
  
  tft.initR(INITR_BLACKTAB);//initialise tft screen
  Serial.print("Initializing SD card...");
  //if (!SD.begin(SD_CS, SD_MOSI, SD_MISO, SD_SCK)) {//initialise SD card
  if (!SD.begin(SD_CS)) {//initialise SD card
    Serial.println("SD card failed!");
    return;
  }
  Serial.println("SD card initialised!");  
  tft.fillScreen(ST7735_BLACK); // Clear display

  
  for (int i=0;i<BAND_MAX;i++) { peak[i] = 0; }   //clear peak levels

  int actband = 0;  //Calculate bands
  for (int i=1; i < samples/2; i++) {
    if (i*bin_size > (limit[actband] + (limit[actband+1]-limit[actband])/2)) actband ++;
    if (actband > BAND_MAX-1) actband = BAND_MAX-1;   
    bin_band[i] = actband;
    Serial.print(i); Serial.print(" "); Serial.print(i*bin_size); Serial.print(" -> "); Serial.println(actband);
  }
}

long minta;

int amin=4096; int amax = 0;

void loop() {
minta = millis();
if (digitalRead(DIP2) == LOW) ArduinoOTA.handle();

//--------------------- READING MUSIC --------------------------------------
  portDISABLE_INTERRUPTS();
  microseconds = micros();
  for(int i=0; i<samples; i++) {
      vReal[i] = analogRead(CHANNEL)-offset;
      vImag[i] = 0;
      while(micros() - microseconds  < sampling_period_us){ } //empty loop
      microseconds += sampling_period_us;
  }
  portENABLE_INTERRUPTS();
//---------------------------------------------------------------
  
  FFT.Windowing(vReal, samples, FFT_WIN_TYP_HAMMING, FFT_FORWARD);  /* Weigh data */
  FFT.Compute(vReal, vImag, samples, FFT_FORWARD); /* Compute FFT */
  FFT.ComplexToMagnitude(vReal, vImag, samples); /* Compute magnitudes */
  
  //PrintVector();
  
//  double x = FFT.MajorPeak(vReal, samples, samplingFrequency);
//  Serial.println(x, 6); //Print out what frequency is the most dominant.

  volume = map(analogRead(POT1),0,4095,-10,10);

if (programMode == 1) {
  for (int s=0; s<XMAX*YMAX; s++) strip.SetPixelColor(s,black);
  showOldPeaks();
  double sum =0;
  for (int i = 1; i < (samples/2); i++){ // 0 = DC level!
    if (vReal[i] > 1000) {
        if (vReal[i] > sum) sum = vReal[i];       //maximum among bins
        //sum += vReal[i];   //sum of bins
        }
    if (sum > amax) amax = sum;
    if (sum < amin) amin = sum;
    if (bin_band[i] != bin_band[i+1]) {
      displayBand(bin_band[i],sum / BIN_MAX * volume);
      sum = 0;
      }
   }
}

  if (programMode == 2) {
    amax = 0;
    for (int i = 1; i < 5; i++) // 0 = DC level!
        if (vReal[i] > amax) amax = vReal[i];
        
    //Serial.print(vReal[2]/BIN_MAX); Serial.print("/"); Serial.println(LEDlimit[6]);
    if (amax/BIN_MAX * volume > LEDlimit[4]) {
      gen++;  
      nextLifeStep();
    } 
  }
                           
  strip.Show();

                         
//while (Serial.available()==0) {delay(100); }    Serial.read();
//Serial.println(millis()-minta);
while( millis()-minta <50) {delay(1);} 

}

void  showOldPeaks() {
  static long lastDecay=0;
  for (byte band = 0; band < BAND_MAX; band++) {
        if (peak[band] > 0) {
            setPixel(band,peak[band],blue);
            //if (peak[band]>0) { for (int j=0;j<peak[band];j++)  setPixel(band,j,green2); }  
        }
        if (peak[band] == 0)  setPixel(band,0,blue2);   
  }
  if (millis()-lastDecay > 400) {
       //display.clear();
       //display.drawString(10, 10, String(amin));
       //display.drawString(10, 20, String(amax));
       //display.display(); 
       amin=4096; amax = 0;

      lastDecay = millis();
      for (byte band = 0; band < BAND_MAX; band++) 
             if (peak[band] > 0) peak[band]--; 
  } // Decay the peak
}

void setPixel(int x,int y, RgbColor color) {

  //Serial.print("X:"); Serial.print(x); Serial.print(" Y:"); Serial.print(y);
  if (y%2 ==0) {
    strip.SetPixelColor(x + XMAX*y, color); 
    //Serial.print("  APOS:"); Serial.println(x + XMAX*y);
    }
  else { 
    strip.SetPixelColor(XMAX*(y+1)-x-1, color);
    //Serial.print("  BPOS:"); Serial.println(XMAX*(y+1)-x-1);
    }
}

void displayBand(int band, double level){
  int dsize = -1;
    
  for (int k=0; k<YMAX; k++) if (level >= LEDlimit[k]) dsize = k;  //find the upper LED position
  if (dsize<0) return;   //level is below the last limit
  
  if (dsize >= YMAX) dsize = YMAX;  //safety only
  
  //Serial.print("band:"); Serial.print(band); Serial.print("  level:"); Serial.println(level); Serial.print("  dsize:"); Serial.println(dsize);
 //Serial.print("band:"); Serial.print(band); Serial.print("  dsize:"); Serial.println(dsize);
  for (int s = 0; s <= dsize; s++){
    RgbColor color1 = green;
    if (s==YMAX-1) color1 = red;
    if (s==YMAX-2) color1 = orange;
    if (s==YMAX-3) color1 = yellow;
    setPixel(band,s,color1); 
   //Serial.print(band); Serial.print(" - "); Serial.println(s);
  }
 
  if (dsize > peak[band]) {peak[band] = dsize;}  //peak level monitoring
}

void PrintVector() {
  Serial.println("Computed magnitudes:");
  for (uint16_t i = 0; i < samples/2; i++)  {
    Serial.print(i); Serial.print(": ");
    Serial.print(i * bin_size, 0);
    Serial.print("Hz ");
    Serial.println(vReal[i],0);
  }
  Serial.println("----------------------------");
}
