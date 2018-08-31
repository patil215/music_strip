#include <FastLED.h>
#include <arduinoFFT.h>
 
#define SAMPLES 128             // Must be a power of 2
#define SAMPLING_FREQUENCY 1000 // Hz, must be less than 10000 due to ADC
#define LED_PIN 6
#define NUM_LEDS 160

arduinoFFT FFT = arduinoFFT();
 
unsigned int sampling_period_us;
unsigned long microseconds;
 
double vReal[SAMPLES];
double vImag[SAMPLES];

CRGB leds[NUM_LEDS];

const int sampleWindow = 40; // Sample window width in mS (50 mS = 20Hz)
unsigned int sample;
 
void setup() {
    Serial.begin(115200);
    FastLED.addLeds<NEOPIXEL, LED_PIN>(leds, NUM_LEDS);
 
    sampling_period_us = round(50 * (1.0/SAMPLING_FREQUENCY));
}

double getVoltage() {
   unsigned long startMillis = millis();  // Start of sample window
   unsigned int peakToPeak = 0;   // peak-to-peak level
 
   unsigned int signalMax = 0;
   unsigned int signalMin = 1024;
 
   // collect data for 50 mS
   while (millis() - startMillis < sampleWindow) {
      sample = analogRead(0);
      if (sample < 1024) {   // toss out spurious readings 
         if (sample > signalMax) {
            signalMax = sample;  // save just the max levels
         } else if (sample < signalMin) {
            signalMin = sample;  // save just the min levels
         }
      }
   }
   peakToPeak = signalMax - signalMin;  // max - min = peak-peak amplitude
   double volts = (peakToPeak * 5.0) / 1024;  // convert to volts

   return volts;
}

int prevDistance = 0;

int baseHue = 0;
void loop() {
    for(int i=0; i < SAMPLES; i++) {
        microseconds = micros();    //Overflows after around 70 minutes!
     
        vReal[i] = analogRead(0);
        vImag[i] = 0;
     
        while(micros() < (microseconds + sampling_period_us)){}
    }
 
    /* FFT */
    FFT.Windowing(vReal, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
    FFT.Compute(vReal, vImag, SAMPLES, FFT_FORWARD);
    FFT.ComplexToMagnitude(vReal, vImag, SAMPLES);
    double peak = FFT.MajorPeak(vReal, SAMPLES, SAMPLING_FREQUENCY); // Most dominant frequency

    baseHue++;
    if (baseHue > 1000) {
      baseHue = 0;
    }
    
    float h = (float((int(peak / 10) + baseHue) % 360));
    float s = 1.0f;
    float v = 1.0f;
    float r;
    float g;
    float b;

    HSVtoRGB(r, g, b, h, s, v);
    double volts = getVoltage();
    double multiplier = volts / 2.4;
    int distance = int(multiplier * 80);
    int limit = 5;
    if (abs(prevDistance - distance) > limit) {
      if (prevDistance < distance) {
        distance = prevDistance + limit;
      } else {
        distance = prevDistance - limit;
      }
    }
    prevDistance = distance;
    colorWipe(int(r * 255), int(g * 255), int(b * 255), 1.0, distance);
}

int binFromFrequency(int frequency) {
  return int(double(frequency) * SAMPLES / SAMPLING_FREQUENCY);
}

void colorWipe(uint32_t r, uint32_t g, uint32_t b, double mult, double distance) {
  for(int i = 0; i < NUM_LEDS; i++) {
    leds[i].r = 0;
    leds[i].g = 0;
    leds[i].b = 0;
  }
  
  int start = 80;
  
  for (int i = 0; i < distance; i++) {
    float multiplier = 1 - (float(i) / float(distance));
    leds[start - i].r = int(float(r) * multiplier);
    leds[start - i].g = int(float(g) * multiplier);
    leds[start - i].b = int(float(b) * multiplier);
    leds[start + i].r = int(float(r) * multiplier);
    leds[start + i].g = int(float(g) * multiplier);
    leds[start + i].b = int(float(b) * multiplier);
    //delay(int(80 / distance));
    //FastLED.show();
  }
  FastLED.show();
}

void HSVtoRGB(float& fR, float& fG, float& fB, float& fH, float& fS, float& fV) {
  float fC = fV * fS; // Chroma
  float fHPrime = fmod(fH / 60.0, 6);
  float fX = fC * (1 - fabs(fmod(fHPrime, 2) - 1));
  float fM = fV - fC;
  
  if(0 <= fHPrime && fHPrime < 1) {
    fR = fC;
    fG = fX;
    fB = 0;
  } else if(1 <= fHPrime && fHPrime < 2) {
    fR = fX;
    fG = fC;
    fB = 0;
  } else if(2 <= fHPrime && fHPrime < 3) {
    fR = 0;
    fG = fC;
    fB = fX;
  } else if(3 <= fHPrime && fHPrime < 4) {
    fR = 0;
    fG = fX;
    fB = fC;
  } else if(4 <= fHPrime && fHPrime < 5) {
    fR = fX;
    fG = 0;
    fB = fC;
  } else if(5 <= fHPrime && fHPrime < 6) {
    fR = fC;
    fG = 0;
    fB = fX;
  } else {
    fR = 0;
    fG = 0;
    fB = 0;
  }
  
  fR += fM;
  fG += fM;
  fB += fM;
}

