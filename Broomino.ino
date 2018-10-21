/* 

  Broom_ino uses one 3-axis ADXL335 accelerometer mounted on the flat surface of the broom to read data from Y and X.
  One LDR among the brushes facing ground.
  One piezo-sensor on the end of the stick on the bottom (among the brushes).
  
  Knob for fundamental frequency.
  
  FM synthesis parameters
  
  I'm using Mozzi sonification library.

  Analog input, audio and control oscillators, phase modulation
  and smoothing a control signal at audio rate to avoid clicks.

  //distance sensor
 
  
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
 
 // range of the readings from the distance sensor in cm
int maximumRange = 400; // Maximum range needed
int minimumRange = 1; // Minimum range needed
long duration, distance; // Duration used to calculate distance

float pitch;
boolean p = false;
 
 //HC-SR04
#define echoPin 12 // Echo Pin
#define trigPin 13 // Trigger Pin
#define LEDPin 2 //  LED
 
// desired carrier frequency max and min, for AutoMap
const int MIN_CARRIER_FREQ = 22;
const int MAX_CARRIER_FREQ = 440;

// desired intensity max and min, for AutoMap, note they're inverted for reverse dynamics
const int MIN_INTENSITY = 700;
const int MAX_INTENSITY = 10;

// desired mod speed max and min, for AutoMap, note they're inverted for reverse dynamics
const int MIN_MOD_SPEED = 100000;
const int MAX_MOD_SPEED = 1;
/*
AutoMap kMapCarrierFreq(0,1023,MIN_CARRIER_FREQ,MAX_CARRIER_FREQ);
AutoMap kMapIntensity(0,1023,MIN_INTENSITY,MAX_INTENSITY);
AutoMap kMapModSpeed(0,1023,MIN_MOD_SPEED,MAX_MOD_SPEED);
*/
//const int kMapCarrierFreq(500); initial pitch
//const int kMapIntensity(300);
//const int kMapModSpeed(500);

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
// use #define for CONTROL_RATE, not a constant
#define CONTROL_RATE 64 // powers of 2 please

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
  
  
 pinMode(trigPin, OUTPUT);
 pinMode(echoPin, INPUT);
 pinMode(LEDPin, OUTPUT); // Use LED indicator (if required)
 
  startMozzi(CONTROL_RATE); // :))
  //startMozzi(); // :))
}


void updateControl(){
  
  // ---------------------------------------------
     //read the distance
  /* The following trigPin/echoPin cycle is used to determine the
 distance of the nearest object by bouncing soundwaves off of it. */ 
 digitalWrite(trigPin, LOW); 
 delayMicroseconds(2); 

 digitalWrite(trigPin, HIGH);
 delayMicroseconds(10); 
 
 digitalWrite(trigPin, LOW);
 duration = pulseIn(echoPin, HIGH);
 
 //Calculate the distance (in cm) based on the speed of sound.
 distance = duration/58.2;
 
 Serial.println(distance);
   float pitch= 34300/ distance;
   //aSin.setFreq(pitch); // set the frequency
  //-------------------------------------------------- 
  //Serial.print("pitch = ");
  //Serial.print(pitch);
  //Serial.println();
  // read the knob
  //int knob_value = mozziAnalogRead(KNOB_PIN); // value is 0-1023
  
    // map the knob (the one I had gave that range)
  //int knob_mapped= map (knob_value, 28, 979, 0, 1023);
  
  // read the piezo
  int piezo_value = mozziAnalogRead(PIEZO_PIN); 
  
   // read the ldr
  int ldr_value = mozziAnalogRead(LDR_PIN); // value is 0-1023
  
   // map the ldr to spread coefficient
  int ldr_spread = map (ldr_value, 0, 400, -2, 2);
  //int ldr_spread = map (ldr_value, 0, 400, -12, 12);
   
  //map piezo for variable harmonics
  int mod_ratio = map (piezo_value, 0, 1023, 1, 24);
   
  // map the knob to carrier frequency
  //int carrier_freq = kMapCarrierFreq(knob_mapped);
  //int carrier_freq = kMapCarrierFreq(pitch); for dynamic autorange mapping
  //int carrier_freq = pitch;
  
  
   // read the y_axis from accelerometer on the width Analog input pin
  int y_value= mozziAnalogRead(Y_PIN); // value is oscillating around 400
  // print the value to the Serial monitor for debugging
  //int carr = map( y_value, 370, 450, 0, 1023);
  int carrY = map( y_value, 370, 450, 220, 880);
    //Serial.print("y = "); 
  //Serial.print(y_value);
  //Serial.print("\t"); // prints a tab
 
 
  float Yspeed = (float)carrY;
  
  
 //int Y_calibrated =carr;
  //int Y_calibrated = kMapIntensity(carr);
  //Serial.print("Y_calibrated = ");
  //Serial.print(Y_calibrated);
  //Serial.print("\t"); // prints a tab
    // use a float here for low frequencies
    
      
  // read the z from the accelerometer on the speed Analog input pin
  int z_value= mozziAnalogRead(Z_PIN); // value is 0-1023
  //Serial.print("Z = "); 
  //Serial.print(z_value);
  //Serial.print("\t"); // prints a tab
 // int intens = map( z_value, 270, 400, 0, 1023);       //the intensity of the frequency modulation is dependent on z reading
 float calculator = (float)pitch/12;
  int intensZ = map( z_value, 270, 400, 1, 100000);       //the intensity of the frequency modulation is dependent on z reading
   
  
  //float pitchBend = (carrier_freq/1200 * ldr_spread *100);
    float pitchBend = 0;
  
  //calculate the modulation frequency to stay in ratio, the number is from testing.
  //int mod_freq = (carrier_freq * mod_ratio/3)+ pitchBend ;
  //int mod_freq = carrier_freq ;
  int mod_freq = pitch ;
   //float mod_freq = (float)intensZ*calculator;    // the modulation speed is dependent on the intensity reading 
   //float mod_freq = (float)intensZ + calculator;    // the modulation speed is dependent on the intensity reading 
  
  // set the FM oscillator frequencies
  //aCarrier.setFreq(carrier_freq); 
  //aCarrier.setFreq(pitch); 
  aCarrier.setFreq(440); 
  //aModulator.setFreq(intensZ);
  aModulator.setFreq(880);
   
 
  

   
 // calculate the fm_intensity
 // fm_intensity = ((long)Y_calibrated * (kIntensityMod.next()+128))>>8; // shift back to range after 8 bit multiply
  //fm_intensity = ((long)Yspeed * (kIntensityMod.next()+128))>>8; // shift back to range after 8 bit multiply 
fm_intensity = ((long)Yspeed * (kIntensityMod.next()+128))>>8; // shift back to range after 8 bit multiply  
  //Serial.print("fm_intensity = ");
  //Serial.print(fm_intensity);
 // Serial.print("\t"); // prints a tab

  
 
 
  float mod_speed = (float)Yspeed;
  //float mod_speed = (float)kMapModSpeed(intens)/1000;
  //Serial.print("   mod_speed = ");
  //Serial.print(mod_speed);
  //kIntensityMod.setFreq(mod_speed);   //set the new speed of the intensity modulation
   kIntensityMod.setFreq(100);   //set the new speed of the intensity modulation
  
  //Serial.println(); // finally, print a carraige return for the next line of debugging info
  
  // read the state of the pushbutton value:
  button_state = digitalRead(BUTTON_PIN_PLAY);
  //fnstatus = digitalRead(BUTTON_PIN_FN);
  
  
}


int updateAudio(){
  
  //if button play pressed
  
  if (button_state==HIGH){
  //long modulation = aSmoothIntensity.next(fm_intensity) * aModulator.next();    //fm intensity  0-1 * 1/12
   long modulation = aSmoothIntensity.next(fm_intensity) * aModulator.next();    //fm intensity  0-1 * 1/12
   //long modulation = aSmoothIntensity.next(1) * 0.001;    //fm intensity  0-1 * 1/12
  //return aSin.next();
  //if button fn released
  
   //if (fnstatus==LOW){
 return aCarrier.phMod(modulation);
 //return aCarrier.phMod(1);
    //}
    
  }
}


void loop(){
  audioHook();
}





