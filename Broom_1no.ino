/* 

  Broom_ino uses one 3-axis ADXL335 accelerometer mounted on the flat surface of the broom to read data from Y and X.
  One LDR among the brushes facing ground.
  One piezo-sensor on the end of the stick on the bottom (among the brushes).
  
  Knob for fundamental frequency.
  
  FM synthesis parameters
  
  I'm using Mozzi sonification library.

  Analog input, audio and control oscillators, phase modulation
  and smoothing a control signal at audio rate to avoid clicks.

  //no distance sensor
 
  
  The circuit:
     Audio output on digital pin 9 (on a Uno or similar), or 
     check the README or http://sensorium.github.com/Mozzi/

     Potentiometer connected to analog pin 0.
       Center pin of the potentiometer goes to the analog pin.
       Side pins of the potentiometer go to +5V and ground
       
     Piezo on analog pin 1:
     + connection of the piezo attached to the analog pin
     - connection of the piezo attached to ground
     1-megohm resistor between the analog pin and ground
     
     YAccelerometer connected to analog pin 2.
     
     ZAccelerometer connected to analog pin 3.
      
   
     Light dependent resistor (LDR) and 5.1k resistor on analog pin 4:
       LDR from analog pin to +5V
       5.1k resistor from analog pin to ground
     
     Pushbutton on digital pin 4
     
     Pushbutton on digital pin 8
     
  from mozzi (
  Mozzi help/discussion/announcements:
  https://groups.google.com/forum/#!forum/mozzi-users

  Tim Barrass 2013.
  This example code is in the public domain.
  )
  
  Alessia Milo 2013.
  
*/

#include <MozziGuts.h>
#include <Oscil.h> // oscillator 
#include <tables/cos2048_int8.h> // table for Oscils to play
#include <Smooth.h>
#include <AutoMap.h> // maps unpredictable inputs to a range
 
// desired carrier frequency max and min, for AutoMap
const int MIN_CARRIER_FREQ = 22;
const int MAX_CARRIER_FREQ = 440;

// desired intensity max and min, for AutoMap, note they're inverted for reverse dynamics
const int MIN_INTENSITY = 700;
const int MAX_INTENSITY = 10;

// desired mod speed max and min, for AutoMap, note they're inverted for reverse dynamics
const int MIN_MOD_SPEED = 100000;
const int MAX_MOD_SPEED = 1;

AutoMap kMapCarrierFreq(0,1023,MIN_CARRIER_FREQ,MAX_CARRIER_FREQ);
AutoMap kMapIntensity(0,1023,MIN_INTENSITY,MAX_INTENSITY);
AutoMap kMapModSpeed(0,1023,MIN_MOD_SPEED,MAX_MOD_SPEED);

const int KNOB_PIN = 0; // set the input for the knob to analog pin 0
const int PIEZO_PIN=1; // set the analog input for piezo reading on pin 1

const int Y_PIN=2; // set the analog input for mod rate to pin 2
const int Z_PIN=3; // set the analog input for fm_intensity to pin 3


const int LDR_PIN=5; // set the analog input for ldr to pin 2


const int BUTTON_PIN_PLAY = 4;  // set the digital input pin for the button play
const int BUTTON_PIN_FN = 8;  // set the digital input pin for the button fn

Oscil<COS2048_NUM_CELLS, AUDIO_RATE> aCarrier(COS2048_DATA);
Oscil<COS2048_NUM_CELLS, AUDIO_RATE> aModulator(COS2048_DATA);
Oscil<COS2048_NUM_CELLS, CONTROL_RATE> kIntensityMod(COS2048_DATA);

int mod_ratio = 5; // brightness (harmonics) - default values
long fm_intensity; // carries control info from updateControl to updateAudio

char button_state; // variable for reading the pushbutton status  
char fnstatus; // variable for reading the pushbutton status
int ldr_spread; // variable for ldr_pitchbending


// smoothing for intensity to remove clicks on transitions
float smoothness = 0.95f;
Smooth <long> aSmoothIntensity(smoothness);


void setup(){
  Serial.begin(115200); // set up the Serial output so we can look at the light level
  startMozzi(); // :))
}


void updateControl(){
  
  // read the knob
  int knob_value = mozziAnalogRead(KNOB_PIN); // value is 0-1023
  
    // map the knob (the one I had gave that range)
  int knob_mapped= map (knob_value, 28, 979, 0, 1023);
  
  // read the piezo
  int piezo_value = mozziAnalogRead(PIEZO_PIN); 
  
   // read the ldr
  int ldr_value = mozziAnalogRead(LDR_PIN); // value is 0-1023
  
   // map the ldr to spread coefficient
  int ldr_spread = map (ldr_value, 0, 400, -12, 12);
   
  //map piezo for variable harmonics
  int mod_ratio = map (piezo_value, 0, 1023, 1, 24);
    
  // map the knob to carrier frequency
  int carrier_freq = kMapCarrierFreq(knob_mapped);
  
  float pitchBend = (carrier_freq/1200 * ldr_spread *100);
  
  //calculate the modulation frequency to stay in ratio, the number is from testing.
  int mod_freq = (carrier_freq * mod_ratio/3)+ pitchBend ;
  
  // set the FM oscillator frequencies
  aCarrier.setFreq(carrier_freq); 
  aModulator.setFreq(mod_freq);
  
  // read the y_axis from accelerometer on the width Analog input pin
  int y_value= mozziAnalogRead(Y_PIN); // value is oscillating around 400
  // print the value to the Serial monitor for debugging
  int carr = map( y_value, 370, 450, 0, 1023);
  Serial.print("y = "); 
  Serial.print(y_value);
  Serial.print("\t"); // prints a tab

  int Y_calibrated = kMapIntensity(carr);
  Serial.print("Y_calibrated = ");
  Serial.print(Y_calibrated);
  Serial.print("\t"); // prints a tab
  
 // calculate the fm_intensity
  fm_intensity = ((long)Y_calibrated * (kIntensityMod.next()+128))>>8; // shift back to range after 8 bit multiply
  Serial.print("fm_intensity = ");
  Serial.print(fm_intensity);
  Serial.print("\t"); // prints a tab
  
  // read the z from the accelerometer on the speed Analog input pin
  int z_value= mozziAnalogRead(Z_PIN); // value is 0-1023
  Serial.print("Z = "); 
  Serial.print(z_value);
  Serial.print("\t"); // prints a tab
  int intens = map( z_value, 270, 400, 0, 1023); 
  
  // use a float here for low frequencies
  float mod_speed = (float)kMapModSpeed(intens)/1000;
  Serial.print("   mod_speed = ");
  Serial.print(mod_speed);
  kIntensityMod.setFreq(mod_speed);
  
  Serial.println(); // finally, print a carraige return for the next line of debugging info
  
  // read the state of the pushbutton value:
  button_state = digitalRead(BUTTON_PIN_PLAY);
  fnstatus = digitalRead(BUTTON_PIN_FN);
  
  
}


int updateAudio(){
  
  //if button play pressed
  
  if (button_state==HIGH){
  long modulation = aSmoothIntensity.next(fm_intensity) * aModulator.next();
  
  //if button fn released
  
   if (fnstatus==LOW){
  return aCarrier.phMod(modulation);
    }
    
  }
}


void loop(){
  audioHook();
}





