/*
 *  SoundSensor
 *  by Scott Kildall
 *  www.kildall.com
 *  
 *  Sonaqua: EC Water Sensor to play sounds
 *  
 *  Designed for Arduino Uno / Metro Mini, but will work on other platforms
 *  
 *  Adapted from this Hackaday project
 *  https://hackaday.io/project/7008-fly-wars-a-hackers-solution-to-world-hunger/log/24646-three-dollar-ec-ppm-meter-arduino
 *  
 */

#include "Adafruit_LEDBackpack.h"
#include "MSTimer.h"
#include <Adafruit_NeoPixel.h>

//-- leave uncommented to see serial debug messages
//#define SERIAL_DEBUG

//-- Conditional defines for testing hardware
//#define SPEAKER_ONLY            // just play ascending tones, to make sure speaker works, tests speakers
//  #define SPEAKER_POT             // plays speaker according to potentiometer, tests pot

//-- Conditional to modulate a range on the pot
//#define POT_RANGE_TEST


//-- PINS
#define speakerPin (6)
#define waterLEDPin (10)
#define SwitchPin (8)
#define neoPixelPin (11)


#define PotPin (A0)
#define ECPower (A2)
#define ECPin (A1)   

#define LEDPin (9)

//-- NeoPixel colors
int r;
int b;
int g;

unsigned int toneValue;

//-- If raw EC is above this, we won't play the sounds
#define EC_SILENT_THRESHOLD (970)
#define DELAY_TIME (80)
#define MIN_TONE (50)   // lowest possible tone for Arduinos
#define MIN_TONE_THRESHOLD (60)   // Under this and we drop it down a bit more

Adafruit_7segment matrix = Adafruit_7segment();
MSTimer displayTimer = MSTimer();

//-- if this is set to TRUE, then we look at a pin for a digital input (from another Arduino), which acts like a swetch
//-- OR this can be just a switch to activate
boolean bUseSwitch = false;
boolean bLightsOn = false;

// Instatiate the NeoPixel from the library
#define numPixels (16)

Adafruit_NeoPixel strip = Adafruit_NeoPixel(numPixels, neoPixelPin, NEO_GRB + NEO_KHZ800);

void setup() {
  
   //-- pin inputs / outputs
  pinMode(ECPin,INPUT);
  pinMode(ECPower,OUTPUT);                // set pin for sourcing current
  pinMode(speakerPin, OUTPUT); 
  pinMode(LEDPin,OUTPUT);                // set pin for sourcing current
  pinMode(waterLEDPin,OUTPUT);
  
  pinMode(SwitchPin, INPUT);


  // Flash LED
  for(int i = 0; i < 6; i++ ) {
    digitalWrite(LEDPin,HIGH);
    delay(100);
    digitalWrite(LEDPin,LOW);
    delay(100);
  }
  digitalWrite(LEDPin,HIGH);
  
  matrix.begin(0x70);
  matrix.print(9999, DEC);
  matrix.writeDisplay();
  
#ifdef SERIAL_DEBUG
  //-- no serial support for the Digispark
  Serial.begin(115200);
  Serial.println("startup");
#endif

 
  //-- speaker ground is always low
  //digitalWrite(speakerGndPin,LOW);

  // every 1000ms we will update the display, for readability
  displayTimer.setTimer(1000);
  displayTimer.start();
   
   matrix.begin(0x70);
 
  matrix.print(7777, DEC);
  matrix.writeDisplay();

  setNeoPixelColors();

  strip.begin();  // initialize the strip
  lightsOn();
  delay(1000);
  lightsOff();
  
  digitalWrite(waterLEDPin, LOW);
}

//-- rawEC == 0 -> max conductivity; rawEC == 1023, no conductivity
void loop() { 
  boolean bSwitchOn = digitalRead(SwitchPin);
  if( bUseSwitch == false )
    bSwitchOn = true;
    
  // Get inputs: EC and Pot Value
  unsigned int rawEC = getEC(); 
  int potValue = analogRead(PotPin);;

  if( displayTimer.isExpired() ) {
    //-- quick test
   // matrix.print(bSwitchOn, DEC);
    matrix.print(rawEC, DEC);
    matrix.writeDisplay();
    displayTimer.start();
  }



#ifdef SPEAKER_ONLY
  speakerOnlyTest();
  return;
#endif

#ifdef SPEAKER_POT
  speakerPotTest(potValue);
  return;
#endif

#ifdef POT_RANGE_TEST
  potRangeTest(potValue);
  return;
#endif

  // Only activate if we below the EC Threshold
  if( rawEC > EC_SILENT_THRESHOLD ) { 
    noTone(speakerPin);
    digitalWrite(waterLEDPin, LOW);
    
    lightsOff();
    delay(DELAY_TIME);

    #ifdef SERIAL_DEBUG       
      Serial.println("-----");
    #endif
    
    return;
  }

  //-- clean this up
  boolean bPlaySound = true;
  if( bUseSwitch == true )
    bPlaySound = digitalRead(SwitchPin);
    
  
#ifdef SERIAL_DEBUG       
    Serial.print("rawEC = ");
    Serial.println(rawEC);
#endif

#ifdef SERIAL_DEBUG       
    Serial.print("Pot value = ");
    Serial.println(potValue);
#endif

  toneValue = getToneValueFromEC(rawEC);
 
  if (toneValue < 0 )
    toneValue = MIN_TONE;

    // Handle less-than-zero, will overflow to large 8-bit (65535) number
    if( toneValue > 50000 )
      toneValue = MIN_TONE;

//-- specific modulations for Currents 2018
    //toneValue += random(4);

  // this corrects for the random sampling noise that we get in normal cases
  if( toneValue == MIN_TONE ) {
    toneValue += random(2);
  }
  
  // polluted water glitch
  
  if( rawEC < 300 ) {
    if( random(rawEC-20) < 5 )
      toneValue += random(60);
  }
  
//--

#ifdef SERIAL_DEBUG       
     Serial.print("Tone value = ");
     Serial.println(toneValue);
#endif 

    //-- emit some sort of tone based on EC
    if( bSwitchOn ) {
      lightsOn();
      tone(speakerPin, toneValue );
      digitalWrite(waterLEDPin, HIGH);
    }
    else {
      noTone(speakerPin);
      digitalWrite(waterLEDPin, LOW);
       lightsOff();
    }
     
    delay(DELAY_TIME);    
}

//-- Sample EC using Analog pins, returns 0-1023
unsigned int getEC(){
  unsigned int raw;
 
  digitalWrite(ECPower,HIGH);

  // This is not a mistake, First reading will be low beause of charged a capacitor
  raw= analogRead(ECPin);
  raw= analogRead(ECPin);   
  
  digitalWrite(ECPower,LOW);
  

 return raw;
}

// expand the range of the tone value by doubling the rawEC and doing some various math to it
// this works for a sampling range where minumim EC < 400
unsigned int getToneValueFromEC(unsigned int rawEC) {
  long toneValue = ((long)rawEC - 100);

  if( toneValue  < MIN_TONE_THRESHOLD ) {
    toneValue = MIN_TONE;
    
  }
  else {
    toneValue = toneValue/2;
    
   if( toneValue  < MIN_TONE_THRESHOLD )
      toneValue = MIN_TONE;
  }

  return (unsigned int)toneValue;
}

//-- depending on the type of fluid we are using, we have different neoPixelColors, default is pure white
void setNeoPixelColors() { 
  r = 128; g = 255; b = 255;
}

//-- turn on the neopixel ring when we are activating the circuit
void lightsOn() {
  if( bLightsOn )
    return;
    
  // set the colors for the strip
   for( int i = 0; i < numPixels; i++ )
       strip.setPixelColor(i, r,g,b);
   
   // show all pixels  
   strip.show();
   bLightsOn = true;
}


void lightsOff() {
  if( !bLightsOn )
    return;
    
  strip.clear();  // Initialize all pixels to 'off
  strip.show();
  bLightsOn = false;
}


void speakerOnlyTest() {
  for( int i = 100; i < 600; i += 10 ) {
    tone( speakerPin, i );
    delay(100);
  }
  
  return;
}

void speakerPotTest(int potValue) {
#ifdef SERIAL_DEBUG  
  Serial.println(potValue);
#endif

  tone( speakerPin, 100 + (potValue/4) );
  delay(100);
}


void potRangeTest(int potValue) {
#ifdef SERIAL_DEBUG  
  Serial.println(potValue);
#endif

// these are hard-coded ranges
    int val = map(potValue, 0, 1023, 93, 873);
    
  lightsOn();

  unsigned int toneValue = getToneValueFromEC(val);

  // val will show rage
  matrix.print(val, DEC);
  matrix.writeDisplay();

  toneValue += random(4);

  // polluted water glitch
  if( val < 300 ) {
    if( random(val-20) < 5 )
      toneValue += random(60);
  }

  tone(speakerPin, toneValue );
  delay(DELAY_TIME);
}

