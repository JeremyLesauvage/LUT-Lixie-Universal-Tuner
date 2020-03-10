/*
  ////////////////////////////////////////////////////////
  //            LIXIE UNIVERSAL TUNER                   //
  //  music tuner with chromatic, Bb, Eb and F mode     //
  //       Range from C0=32.7Hz to A5=1864.66Hz         //
  //    good accuracy on the midrange (300Hz-1865Hz)    //
  //                                                    //
  //             Abo Akademi 2019-2020                  //
  //  Jérémy Lesauvage                                  //
  //  Andreas Salminen                                  //
  //                                                    //
  //The frequency detection algorithm is based on       //
  //Amanda Ghassaei solution. All other parts of the    //
  //program are made by the Lixie Universal Tuner team. //
  ////////////////////////////////////////////////////////
*/

#include "arduinoFFT.h"
#include <FastLED.h>
#include <LiquidCrystal.h>

#define SAMPLES 64                //SAMPLES-pt FFT. Must be a base 2 number. Max 128 for Arduino Uno.
#define SAMPLING_FREQUENCY  4000  //Shannon theoreme, must be at least 2 times the highest expected frequency. max 9615 for arduino uno
#define NUM_LEDS            6     //The total number of LED
#define LED_PIN             7     //The data wire of the led strip is connected to arduino digital pin 7
#define modeBtn             6     //The push button change mode or reference is connected to arduino digital pin 6
#define toggleButton        8     // The on-off button select what to change with the pushbutton (mode or reference)

arduinoFFT    FFT = arduinoFFT();
CRGB          leds[NUM_LEDS];
LiquidCrystal lcd(12, 11, 5, 4, 3, 2);

//Variable for controlling the LED
int   LEDR   = 5;
int   LEDYH2 = 4;
int   LEDYH  = 3;
int   LEDG   = 2;
int   LEDYL  = 1;
int   LEDYL2 = 0;

//Variable for the FFT
unsigned int    samplingPeriod;
unsigned long   microSeconds;
double          vReal[SAMPLES]; //create vector of size SAMPLES to hold real values
double          vImag[SAMPLES]; //create vector of size SAMPLES to hold imaginary values

//variables for filtering out some of the scattered frequencies
int             samples = 1;
float           latestFrequencies[1]; //same as samples
volatile float  largestFreq = 0;
volatile float  largestFreqPre;
volatile int    latestFreqArrayIndex = 0;

//variables for frequency to notes conversion
int reference = 440; //using to calculate octave 0 in order to facilitate change between reference 432/434/436/438/442/444Hz
float c0;  
float c0d; 
float d0;  
float d0d; 
float e0;  
float f0;  
float f0d; 
float g0;  
float g0d; 
float a0;  
float a0d; 
float b0;  

volatile float fbase = c0;
volatile float frmin;
volatile float nnmin;
volatile float errorPercent = 0.0;
volatile float prevNote;
volatile float nextNote;
volatile float detectedNote;
volatile float noteDisplayed;

//Variable for switching mode and reference
int mode = 1;                           // variable to choose between modes: 1=C, 2=Bb, 3=Eb, 4=F
int modeLast = mode;                    // Used to prevent one button press from changing several modes
int referenceSelection = 5;             // Variable to choose between references: 1=432Hz, 2=434,..., 5=440,...,8=446
int referenceLast = referenceSelection; // Used to prevent one button press from changing several references



//////////////////////////////////////////////
//                                          //
//                SETUP                     //      
//                                          //
//////////////////////////////////////////////

void setup() 
{
  samplingPeriod = round(1000000*(1.0/SAMPLING_FREQUENCY)); //Period in microseconds for FFT
  Serial.begin(115200);
  lcd.begin(16, 2);
  FastLED.addLeds<WS2812, LED_PIN, GRB>(leds, NUM_LEDS);
  pinMode(toggleButton, INPUT);
  pinMode(modeBtn, INPUT);
  digitalWrite(modeBtn, HIGH);
}



//////////////////////////////////////////////
//                                          //
//                LOOP                      //      
//                                          //
//////////////////////////////////////////////

void loop() 
{
  /*mode or reference selection*/
  //the on-off button let the user select between change mode or change reference
  if(digitalRead(toggleButton) == LOW)//On-off button is LOW
  {
    if (digitalRead(modeBtn) == LOW) //The pushButton change mode
    {
      if (modeLast == mode) 
      {
        mode += 1;
        if (mode > 4) 
        {
          mode = 1;
        }
      }
    } else 
    {
      modeLast = mode;
    }
  }
  else //On-off button is HIGH
  {
    if (digitalRead(modeBtn) == LOW) //The pushButton Change reference
    {
      if (referenceLast == referenceSelection) 
      {
        referenceSelection += 1;
        if (referenceSelection > 7) 
        {
          referenceSelection = 1;
        }
      }
    } else 
    {
      referenceLast = referenceSelection;
    }
    switch (referenceSelection) 
    {
      case 1:
        reference = 432;
        break;
        
        case 2:
        reference = 434;
        break;
        
        case 3:
        reference = 436;
        break;
        
        case 4:
        reference = 438;
        break;
        
        case 5:
        reference = 440;
        break;
        
        case 6:
        reference = 442;
        break;
        
        case 7:
        reference = 444;
        break;
    }
  }

  /*calculate the octave zero after having the reference value*/
  c0  = reference * pow(2.0, -45.0/12); //from reference A, c0  is  45 semiton down
  c0d = reference * pow(2.0, -44.0/12); //from reference A, c0d is  44 semiton down
  d0  = reference * pow(2.0, -43.0/12); //from reference A, d0  is  43 semiton down
  d0d = reference * pow(2.0, -42.0/12); //from reference A, d0d is  42 semiton down
  e0  = reference * pow(2.0, -41.0/12); //from reference A, e0  is  41 semiton down
  f0  = reference * pow(2.0, -40.0/12); //from reference A, f0  is  40 semiton down
  f0d = reference * pow(2.0, -39.0/12); //from reference A, f0d is  39 semiton down
  g0  = reference * pow(2.0, -38.0/12); //from reference A, go  is  38 semiton down
  g0d = reference * pow(2.0, -37.0/12); //from reference A, g0d is  37 semiton down
  a0  = reference * pow(2.0, -36.0/12); //from reference A, a0  is  36 semiton down
  a0d = reference * pow(2.0, -35.0/12); //from reference A, a0d is  35 semiton down
  b0  = reference * pow(2.0, -34.0/12); //from reference A, b0  is  34 semiton down
    
  /*filtering noise*/
  float voltage = (analogRead(0) * (5.0 / 1023.0)); 
  //voltage max = 2.5 = VDD/2
  //voltage when no noise 2.5/2 = 1.25
  float noiseWindow = 0.10;    //INCREASE TO IMPROVE FILTERING, default 0.10 not good after 0.5
  Serial.println(voltage);

  lightLED(LEDR   ,  255,  255,  255);
  lightLED(LEDYH2 ,  255,  255,  255);
  lightLED(LEDYH  ,  255,  255,  255);
  lightLED(LEDG   ,  255,  255,  255);
  lightLED(LEDYL  ,  255,  255,  255);
  lightLED(LEDYL2 ,  255,  255,  255);
    
  if(voltage >1.25 - noiseWindow && voltage < 1.25+noiseWindow){
    //turnOffAllLED();
    updateLCD("  ", mode,  0, reference);
    delay(50);
  }
  else{
  
  /*Sample SAMPLES times*/
  for(int i=0; i<SAMPLES; i++)
  {
    microSeconds = micros();    //Returns the number of microseconds since the Arduino board began running the current script.    
    vReal[i] = analogRead(0);   //Reads the value from analog pin 0 (A0), quantize it and save it as a real term.
    vImag[i] = 0;               //Imaginary part must be zeroed in case of looping to avoid wrong calculations and overflows
    /*remaining wait time between samples if necessary*/
    while(micros() < (microSeconds + samplingPeriod)){/*do nothing*/}
  }
  
  /*Perform FFT on samples*/
  FFT.Windowing(vReal, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);  //Weigh data 
  // windows type and test
  // FFT_WIN_TYP_RECTANGLE                                          bad
  // FFT_WIN_TYP_HAMMING                                            default, good after 220, generated 440 found 436
  // FFT_WIN_TYP_HANN                                               good after 233, 440 -> 437-438 
  // FFT_WIN_TYP_TRIANGLE                                           bad
  // FFT_WIN_TYP_NUTTALL                                            good after 277, 440 -> 438-439
  // FFT_WIN_TYP_BLACKMAN                                           good after 220, 440 -> 437-438
  // FFT_WIN_TYP_BLACKMAN_NUTTALL                                   good after 246, 440 -> 438
  // FFT_WIN_TYP_BLACKMAN_HARRIS                                    good after 277, 440 -> 438-439
  // FFT_WIN_TYP_FLT_TOP                                            bad 
  // FFT_WIN_TYP_WELCH                                              bad
  FFT.Compute(vReal, vImag, SAMPLES, FFT_FORWARD);                //Compute FFT 
  FFT.ComplexToMagnitude(vReal, vImag, SAMPLES);                  //Compute magnitudes
  
  /*Find peak frequency*/
  float freq = FFT.MajorPeak(vReal, SAMPLES, SAMPLING_FREQUENCY);
 
  /*calibrate the microphone*/
  freq = calibrateMic(freq);
 
  latestFrequencies[latestFreqArrayIndex] = freq;
  latestFreqArrayIndex += 1;

  // Finding the largest value of frequencies from an array NOTE: largestFreq is used as the note played
  if (latestFreqArrayIndex == samples) 
  {
    largestFreq = latestFrequencies[0];
    for (int i = 1; i < samples; i++) 
    {
      if (largestFreq < latestFrequencies[i])
        largestFreq = latestFrequencies[i];
    }
    largestFreqPre = largestFreq; //to save the value of largestFreq (it will be overwritten when comparing frequency to notes)
    frmin = abs(largestFreq - fbase);
    
    /*replace the input freq by a known freq in the table*/
    for (int i = 0; i < 108; i++)                 //108=12*9 meanning we can cover from 20Hz up to octave 9 19331Hz (the full range of the microphone)
    {               
      nnmin = fbase * pow(2, 1.0 / 12);           //formula to determine the +1/2ton next frequence (c0*2^(1/12)) 
      if (abs(largestFreq - nnmin) < frmin)       //if the input freq is not exact it give the nearest freq
      {     
        frmin = abs(largestFreq - nnmin);
        fbase = nnmin;
      }
      else                                        //if the input freq is exact to one of the table it's go next
      { 
        largestFreq = fbase;
        break;
      }
    }

    /*Identifying the note
     * search from octave 0 to octave 5 (max freq of octave 5: B=1975.53Hz) since our limit is SAMPLING_FREQUENCY/2=4000/2
     * E  F  F#  G  G#  |  A  A#  B  C  C#  D  D#  E  F  F#  G  G#  |  A  A#     
     * 1  2  3   4  5      6  7   8  9  10  11 12  13 14 15  16 17     18 19
    */
    for (int i = 0; i < 6; i++) 
    {
      // A
      if (round(largestFreq) == round(a0 * pow(2.0, i))) 
      {
        prevNote = g0d * pow(2.0, i);
        nextNote = a0d * pow(2.0, i);
        //Serial.println("It's a A !");
        detectedNote = a0 * pow(2.0, i);
        errorPercent = calculatePercentage(largestFreqPre, detectedNote, prevNote, nextNote); 
        noteAccuracyLEDS (errorPercent, largestFreqPre, detectedNote);
        noteDisplayed = 6;
      }
      // A#
      if (round(largestFreq) == round(a0d * pow(2.0, i))) 
      {
        prevNote = a0 * pow(2.0, i);
        nextNote = b0 * pow(2.0, i);
        //Serial.println("It's a A# !");
        detectedNote = a0d * pow(2.0, i);
        errorPercent = calculatePercentage(largestFreqPre, detectedNote, prevNote, nextNote);
        noteAccuracyLEDS (errorPercent, largestFreqPre, detectedNote);
        noteDisplayed = 7;
      }
      // B
      if (round(largestFreq) == round(b0 * pow(2.0, i))) 
      {
        prevNote = a0d * pow(2.0, i);
        nextNote = c0 * pow(2.0, i + 1); //B is the last note in the list
        //Serial.println("It's a B !");
        detectedNote = b0 * pow(2.0, i);
        errorPercent = calculatePercentage(largestFreqPre, detectedNote, prevNote, nextNote);
        noteAccuracyLEDS (errorPercent, largestFreqPre, detectedNote);
        noteDisplayed = 8;
      }
      // C
      if (round(largestFreq) == round(c0 * pow(2.0, i))) 
      {
        if (i > 0)
          prevNote = b0 * pow(2.0, i - 1); //C is the first note in the list
        else {              // TODO: handle error when largestFreqPre (input freq) < c0
          prevNote = 1;
        }
        nextNote = c0d * pow(2.0, i);
        //Serial.println("It's a C !");
        detectedNote = c0 * pow(2.0, i);
        errorPercent = calculatePercentage(largestFreqPre, detectedNote, prevNote, nextNote);
        noteAccuracyLEDS (errorPercent, largestFreqPre, detectedNote);
        noteDisplayed = 9;
      }
      // C#
      if (round(largestFreq) == round(c0d * pow(2.0, i))) 
      {
        prevNote = c0 * pow(2.0, i);
        nextNote = d0 * pow(2.0, i);
        //Serial.println("It's a C# !");
        detectedNote = c0d * pow(2.0, i);
        errorPercent = calculatePercentage(largestFreqPre, detectedNote, prevNote, nextNote);
        noteAccuracyLEDS (errorPercent, largestFreqPre, detectedNote);
        noteDisplayed = 10;
      }
      // D
      if (round(largestFreq) == round(d0 * pow(2.0, i))) 
      {
        prevNote = c0d * pow(2.0, i);
        nextNote = d0d * pow(2.0, i);
        //Serial.println("It's a D !");
        detectedNote = d0 * pow(2.0, i);
        errorPercent = calculatePercentage(largestFreqPre, detectedNote, prevNote, nextNote);
        noteAccuracyLEDS (errorPercent, largestFreqPre, detectedNote);
        noteDisplayed = 11;
      }
      // D#
      if (round(largestFreq) == round(d0d * pow(2.0, i))) 
      {
        prevNote = d0 * pow(2.0, i);
        nextNote = e0 * pow(2.0, i);
        //Serial.println("It's a D# !");
        detectedNote = d0d * pow(2.0, i);
        errorPercent = calculatePercentage(largestFreqPre, detectedNote, prevNote, nextNote);
        noteAccuracyLEDS (errorPercent, largestFreqPre, detectedNote);
        noteDisplayed = 12;
      }
      // E
      if (round(largestFreq) == round(e0 * pow(2.0, i))) 
      {
        prevNote = d0d * pow(2.0, i);
        nextNote = f0 * pow(2.0, i);
        //Serial.println("It's a E !");
        detectedNote = e0 * pow(2.0, i);
        errorPercent = calculatePercentage(largestFreqPre, detectedNote, prevNote, nextNote);
        noteAccuracyLEDS (errorPercent, largestFreqPre, detectedNote);
        noteDisplayed = 13;
      }
      // F
      if (round(largestFreq) == round(f0 * pow(2.0, i))) 
      {
        prevNote = e0 * pow(2.0, i);
        nextNote = f0d * pow(2.0, i);
        //Serial.println("It's a F !");
        detectedNote = f0 * pow(2.0, i);
        errorPercent = calculatePercentage(largestFreqPre, detectedNote, prevNote, nextNote);
        noteAccuracyLEDS (errorPercent, largestFreqPre, detectedNote);
        noteDisplayed = 14;
      }
      // F#
      if (round(largestFreq) == round(f0d * pow(2.0, i))) 
      {
        prevNote = f0 * pow(2.0, i);
        nextNote = g0 * pow(2.0, i);
        //Serial.println("It's a F# !");
        detectedNote = f0d * pow(2.0, i);
        errorPercent = calculatePercentage(largestFreqPre, detectedNote, prevNote, nextNote);
        noteAccuracyLEDS (errorPercent, largestFreqPre, detectedNote);
        noteDisplayed = 15;
      }
      // G
      if (round(largestFreq) == round(g0 * pow(2.0, i))) 
      {
        prevNote = f0d * pow(2.0, i);
        nextNote = g0d * pow(2.0, i);
        //Serial.println("It's a G !");
        detectedNote = g0 * pow(2.0, i);
        errorPercent = calculatePercentage(largestFreqPre, detectedNote, prevNote, nextNote);
        noteAccuracyLEDS (errorPercent, largestFreqPre, detectedNote);
        noteDisplayed = 16;
      }
      // G#
      if (round(largestFreq) == round(g0d * pow(2.0, i))) 
      {
        prevNote = g0 * pow(2.0, i);
        nextNote = a0 * pow(2.0, i);
        //Serial.println("It's a G# !");
        detectedNote = g0d * pow(2.0, i);
        errorPercent = calculatePercentage(largestFreqPre, detectedNote, prevNote, nextNote);
        noteAccuracyLEDS (errorPercent, largestFreqPre, detectedNote);
        noteDisplayed = 17;
      }
    }

    switch (mode) {
      case 1:
        break;
      case 2://Bb
        noteDisplayed += 2;
        break;
      case 3: //Eb
        noteDisplayed -= 3;
        break;
      case 4: //F
        noteDisplayed -= 5;
        break;
    }
    //Serial.println("Mode: " + mode);

    /*
     * E  F  F#  G  G#  |  A  A#  B  C  C#  D  D#  E  F  F#  G  G#  |  A  A#     
     * 1  2  3   4  5      6  7   8  9  10  11 12  13 14 15  16 17     18 19
    */
    if (noteDisplayed == 6 || noteDisplayed == 18)
      updateLCD("A ", mode, latestFrequencies[0], reference);

    if (noteDisplayed == 7 || noteDisplayed == 19)
      updateLCD("A#", mode, latestFrequencies[0], reference);
    
    if (noteDisplayed == 8) 
      updateLCD("B ", mode, latestFrequencies[0], reference);
    
    if (noteDisplayed == 9) 
      updateLCD("C ", mode, latestFrequencies[0], reference);

    if (noteDisplayed == 10) 
      updateLCD("C#", mode, latestFrequencies[0], reference);

    if (noteDisplayed == 11)
      updateLCD("D ", mode, latestFrequencies[0], reference);

    if (noteDisplayed == 12)
      updateLCD("D#", mode, latestFrequencies[0], reference);
    
    if (noteDisplayed == 13 || noteDisplayed == 1) 
      updateLCD("E ", mode, latestFrequencies[0], reference);
    
    if (noteDisplayed == 14 || noteDisplayed == 2) 
      updateLCD("F ", mode, latestFrequencies[0], reference);

    if (noteDisplayed == 15 || noteDisplayed == 3) 
      updateLCD("F#", mode, latestFrequencies[0], reference);

    if (noteDisplayed == 16 || noteDisplayed == 4) 
      updateLCD("G ", mode, latestFrequencies[0], reference);

    if (noteDisplayed == 17 || noteDisplayed == 5)
      updateLCD("G#", mode, latestFrequencies[0], reference);
      

    /*counter to 0 and reset array of frequencies*/
    latestFreqArrayIndex = 0;
    for (byte i = 0; i < samples; i++) 
    {
      latestFrequencies[i] = 0;
    }
    fbase = c0;
    frmin = 0;
    nnmin = 0;
    
  }
  delay(150); //REMOVE OR LOWER DELAY TO IMPROVE RAPIDITY
 }
 
}



//////////////////////////////////////////////
//                                          //
//                FUNCTION                  //      
//                                          //
//////////////////////////////////////////////


// Function to calculate error percentage of note played
// Return the error percentage
float calculatePercentage(float largestFreqPre, float detectedNote, float prevNote, float nextNote) {
  float errorPercent = 0;
  if (largestFreqPre > detectedNote) 
    errorPercent = 100 * abs(largestFreqPre - detectedNote) / ((nextNote - detectedNote) / 2);
  else
    errorPercent = 100 * abs(largestFreqPre - detectedNote) / ((detectedNote - prevNote) / 2);
  //Serial.println("Error: " + errorPercent + "%" );
  return errorPercent;
}

//function to light up a specific LED with a given RGB code
void lightLED(int led, int red, int green, int blue) 
{
  leds[led] = CRGB(red, green, blue);
  FastLED.show();
}

//function to turn off a specific LED
void turnOffLED(int led) 
{
  leds[led] = CRGB(0, 0, 0);
  FastLED.show();
}

//function to turn off all the led
void turnOffAllLED() 
{
  for (int i = 0; i < NUM_LEDS; i++) {
    leds[i] = CRGB(0, 0, 0);
  }
  FastLED.show();
}

void turnOffAllOtherAccuracyLEDS (String led) 
{
  if (led != "ledG")
    turnOffLED(LEDG);
  if (led != "ledYH")
    turnOffLED(LEDYH);
  if (led != "ledYH2")
    turnOffLED(LEDYH2);
  if (led != "ledYL")
    turnOffLED(LEDYL);
  if (led != "ledYL2")
    turnOffLED(LEDYL2);
}


// function to light green, yellow and red LEDs(accuracy LED) based on error percentages.
// needs error percentage to determine which LEDs. Needs frequency from sensor and the detected note to determine higher or lower
void noteAccuracyLEDS (float errorPercent, float largestFreqPre, float detectedNote) {
  int margin1 = 20; 
  int margin2 = 60;

  if (errorPercent < margin1)                                   // note played is good
  { 
    //turnOffAllOtherAccuracyLEDS ("ledG");
    lightLED(LEDG, 0, 255, 0); 
  }
  else 
  {
    if (largestFreqPre > detectedNote) {                        // note played is too high
      if (errorPercent >= margin1 && errorPercent < margin2) {  // low error, yellow high on
        //turnOffAllOtherAccuracyLEDS ("ledYH");
        lightLED(LEDYH, 255, 165, 0);
      }
      else {                                                    // HIGH error, yellow high 2 on
        //turnOffAllOtherAccuracyLEDS ("ledYH2");
        lightLED(LEDYH2, 255, 69, 0); 
      }
    }
    else {                                                      // note played is too low
      if (errorPercent >= margin1 && errorPercent < margin2) {  // low error, yellow low on
        //turnOffAllOtherAccuracyLEDS ("ledYL");
        lightLED(LEDYL, 255, 165, 0); 
      }
      else {                                                    // HIGH error, yellow low 2 on
        //turnOffAllOtherAccuracyLEDS ("ledYL2");
        lightLED(LEDYL2, 255, 69, 0); 
      }
    }
  }
}



// writes to LCD screen. LCD screen dimensions: 16x2. 
void updateLCD(String note, int mode, float frequency, int reference) 
{
  String modeStr;
  if (mode == 1)
    modeStr = "C "; //a space after C is added to lign up with note with two char
  if (mode == 2)
    modeStr = "Bb";
  if (mode == 3)
    modeStr = "Eb";
  if (mode == 4)
    modeStr = "F ";

  lcd.clear();
  lcd.setCursor(0, 0);
  if((int)frequency < 1000) //if the frequency has only 3digit add a space before to lign up with a four digit frequency
    lcd.print("  " + note + "      " + (int)frequency + "Hz");
  else
    lcd.print("  " + note + "     " + (int)frequency + "Hz");
  lcd.setCursor(0, 1);
  lcd.print("Mode:" + modeStr + "    A:" + reference);
}


/*look the excel file for more detail on how the factors were found*/
float calibrateMic(float freq){
  float newFreq = 0;
  if (freq >= 1450) 
    newFreq = freq*0.957784038; //1396 1864.66
    
  if (freq >= 1000  && freq < 1450) 
    newFreq = freq*0.960661305; //987 1318

  if (freq >= 890 && freq < 1000)
    newFreq = freq*0.961374974; //880 932

  if (freq >= 785 && freq < 890)
    newFreq = freq*0.955404936; //783 830
    
  if (freq >= 710 && freq < 785)
    newFreq = freq*0.961506047; //698 739

  if (freq >= 630 && freq < 710)
    newFreq = freq*0.960088514; //622 659
    
  if (freq >= 560 && freq < 630)
    newFreq = freq*0.957948365; //554 587
    
  if (freq >= 500 && freq < 560)
    newFreq = freq*0.958857211; //493 523  

  if (freq >= 460 && freq <500) 
    newFreq = freq*0.951354615; //466
    
  if (freq >= 440 && freq < 460)
    newFreq = freq*0.967032967; //440

  if (freq >= 410 && freq < 440)
    newFreq = freq*0.951431844; //415
    
  if (freq >= 387 && freq < 410)
    newFreq = freq*0.965517241; //392

  if (freq >= 369 && freq < 387)
    newFreq = freq*0.963505515; //369

  if (freq >= 345 && freq < 369)
    newFreq = freq*0.948968458; //349
    
  if (freq >= 310 && freq < 345)
    newFreq = freq*0.967841116; // 311 329
    
  if (freq >= 277 && freq < 310)
    newFreq = freq*0.949792181; // 277 293

  if (freq >= 260 && freq < 277)
    newFreq = freq*0.976192883; // 261
    
  if (freq >= 250 && freq < 260)
    newFreq = freq*0.96459417; // 246 
  
  if (freq >= 225 && freq < 250)
    newFreq = freq*0.945825433; // 220 233
  
  if (freq >= 188 && freq < 225)
    newFreq = freq*0.972781294; // 184 207
  
  if (freq >=  160&& freq < 188)
    newFreq = freq*0.938897573; // 155 174
  
  return newFreq;
}
