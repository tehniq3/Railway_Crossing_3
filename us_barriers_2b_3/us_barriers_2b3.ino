// 
// inital changes by MKO
// niq_ro changed few lines
// ver.2 - added bidirectional trigger
// ver.2.1 - removed sounds at opening
// ver.2.b.3 - adding fade lights 

#define GATE_DELAY  2000 // [ms] delay time before gate closes once train detected on Sensor 1


int UP = 2000;  //2000;     // MKO Upper limit of gate travel; adjust these two values to get the travel required
int DOWN = 900; //900;   // MKO lower limit of gate travel
int GateSpeed = 1; // MKO set to an integer between say 1 and 8; lower = slower;
                   //   to set speed that gate moves from up to down and back
int angle = 0;     // MKO variable to track angle of gate opening (thougb it has nothing to do with degrees of travel) 

int trains = 0;
int trains0 = 7;

//#define BLINK_SPEED  700 // [ms] smaller number is faster blinking
#define SERVO_PIN      2  //MKO SERVO_PIN was 3 in JS original sketch. Change to D2
#define LED0_PIN       A0  // 3rd led 
#define LED1_PIN       A1  // blink led 1
#define LED2_PIN       A2  // blink led 2
#define LED3_PIN       5  // fade led 1
#define LED4_PIN       6  // fade led 2
#define SENSOR1_PIN    7  //MKO SENSOR1_PIN was 10 in original sketch. Change to D5 
#define SENSOR2_PIN    8  //MKO SENSOR2_PIN was 11 in original sketch. Change to D8
#define SENSOR3_PIN    9  //niq_ro added 3rd sensor
#define SENSOR4_PIN    10 //niq_ro added 4th sensor

#define speakerPin 11 // Can be either 3 or 11, two PWM outputs connected to Timer 2; using 11 in this sketch

byte state = 1;  // 1;
byte transition = 0; //0;
byte led1 = 0;            // MKO  Define inital values for these 3 variables
byte led2 = 0;            // MKO                                 |      
byte blink_enabled = 0;   // MKO                                 _
byte sound_enabled = 0;   // niq_ro

unsigned long time_to_blink;
unsigned long time_to_close_gate = millis();  //MKO set intial millis times
unsigned long time_to_fade;  
unsigned long time_to_stop = 200;
unsigned long blink_speed; // [ms] smaller number is faster blinking
unsigned long fade_delay = 2; // [ms] smaller number is faster fade
int brightness = 0;    // niq_ro: how bright the LED is
int fadeAmount = 1;    // niq_ro: how many points to fade the LED by


void blinkLights() 
{
    if(millis() > time_to_blink) 
    {
      time_to_blink = millis() + (unsigned long)blink_speed;;
      led1 = !led1;
      led2 = !led1;
    
      digitalWrite(LED1_PIN, led1);
      digitalWrite(LED2_PIN, led2);
      digitalWrite(LED0_PIN, (led1+led2)%2); 
      }
}

void noLights() 
{
      digitalWrite(LED0_PIN, LOW);
      digitalWrite(LED1_PIN, LOW);
      digitalWrite(LED2_PIN, LOW); 
      digitalWrite(LED3_PIN, LOW);
      digitalWrite(LED4_PIN, LOW);       
}

void fadeLights() 
{
  analogWrite(LED3_PIN, brightness);
  analogWrite(LED4_PIN, 255-brightness);
  
if(millis() > time_to_fade) 
  {
  time_to_fade = millis() + fade_delay;
  brightness = brightness + fadeAmount; // change the brightness for next time through the loop:

  // reverse the direction of the fading at the ends of the fade:
  if (brightness <= 0) 
  {
    brightness = 0;
    fadeAmount = -fadeAmount;
    time_to_fade = time_to_fade + time_to_stop;
  }
  if (brightness >= 255) 
  {
    brightness = 255;
    fadeAmount = -fadeAmount;
    time_to_fade = time_to_fade + time_to_stop;
  }
  }
}

void closeGate() 
{
      while (angle < (UP-DOWN)) {
        angle += GateSpeed;  // Use 5 as the servo PWM increment/decrement
        digitalWrite(SERVO_PIN, HIGH);
        delayMicroseconds(angle+DOWN);    //position
        digitalWrite(SERVO_PIN, LOW);
        delayMicroseconds(20000-angle);   //balance of 20000 cycle
        blinkLights();
        /*
        Serial.print("angle = ");
        Serial.println(angle);
        Serial.print("angle + DOWN = ");
        Serial.println(angle+DOWN);    
        Serial.print("state = ");
        Serial.println(state);
        Serial.print("transition = ");
        Serial.println(transition); 
        */
        fadeLights();
      }
        digitalWrite(SERVO_PIN, LOW); 
}


void openGate() 
{
      while ( angle > 0) {
        angle -= GateSpeed;
        digitalWrite(SERVO_PIN, HIGH);
        delayMicroseconds(angle+DOWN);    //position
        digitalWrite(SERVO_PIN, LOW);
        delayMicroseconds(20000-angle);   //balance of 20000 cycle
        blinkLights();
        /*
        Serial.print("angle = ");
        Serial.println(angle);
        Serial.print("angle + DOWN = ");
        Serial.println(angle+DOWN); 
        Serial.print("state = ");
        Serial.println(state);
        Serial.print("transition = ");
        Serial.println(transition);
        */
        fadeLights();
      }
        digitalWrite(SERVO_PIN, LOW); 
}


// MKO Bell setup
#include <stdint.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include "crossing_bell_8KHz_mono.h"  // This is file in the same folder as this sketch that contains the sound data (limited to 1-2 secs total sound due to limited Uno memory)
                                      // Sounds must be in C code which can bet obtained via editing in AUdacity and conversion from WAV files using various tools 
                                      //  (e.g. http://colinjs.com/software.htm#t_WAVToCode)
#define SAMPLE_RATE 8000   // 8 KHz sample rate; compromised between quality and limited PROGMEM space on Uno/Nano/etc.


// Use a cheap "earbud" headphone connected to pin 11 and ground for a modest sound level but still reasonable in scale. 
// Use an simple LM386 audio amp and cheap 8 ohm speaker to get a louder sound using the same speaker pin


volatile uint16_t sample;
byte lastSample;
// MKO END Bell setup




// MKO The following code get the sound data and, via user-controlled variable may zero-out the data for the "no sound" case
// This code called at 8000 Hz to get the next sound sample.
ISR(TIMER1_COMPA_vect) {
    if (sample >= sounddata_length) {
        if (sample == sounddata_length + lastSample) {sample = 0;}
        else {OCR2B = sounddata_length + lastSample - sample;  }
    }
    else {OCR2A = (sound_enabled) * pgm_read_byte(&sounddata_data[sample]);} // <- first variable after the "+" sign needs to be 0 or 1 as dicated by the LED state


    ++sample;   }


void startPlayback()
{
    pinMode(speakerPin, OUTPUT);


    // Set up Timer 2 to do pulse width modulation on the speaker pin.


    // Use internal clock (datasheet p.160)
    ASSR &= ~(_BV(EXCLK) | _BV(AS2));


    // Set fast PWM mode  (p.157)
    TCCR2A |= _BV(WGM21) | _BV(WGM20);
    TCCR2B &= ~_BV(WGM22);


        // Do non-inverting PWM on pin OC2A (p.155)
        // On the Arduino this is pin 11.
        TCCR2A = (TCCR2A | _BV(COM2A1)) & ~_BV(COM2A0); TCCR2A &= ~(_BV(COM2B1) | _BV(COM2B0));
        // No prescaler (p.158)
        TCCR2B = (TCCR2B & ~(_BV(CS12) | _BV(CS11))) | _BV(CS10);


        // Set initial pulse width to the first sample.
        OCR2A = pgm_read_byte(&sounddata_data[0]);
   
    // Set up Timer 1 to send a sample every interrupt.
    cli();


    // Set CTC mode (Clear Timer on Compare Match) (p.133)
    // Have to set OCR1A after, otherwise it gets reset to 0!
    TCCR1B = (TCCR1B & ~_BV(WGM13)) | _BV(WGM12);  TCCR1A = TCCR1A & ~(_BV(WGM11) | _BV(WGM10));


    // No prescaler (p.134)
    TCCR1B = (TCCR1B & ~(_BV(CS12) | _BV(CS11))) | _BV(CS10);


    // Set the compare register (OCR1A).
    // OCR1A is a 16-bit register, so we have to do this with
    // interrupts disabled to be safe.
    OCR1A = F_CPU / SAMPLE_RATE;    // 16e6 / 8000 = 2000


    // Enable interrupt when TCNT1 == OCR1A (p.136)
    TIMSK1 |= _BV(OCIE1A);


    lastSample = pgm_read_byte(&sounddata_data[sounddata_length-1]); sample = 0; sei();
}
// MKO  END Bell timer code setup


void setup() {
  pinMode(SENSOR1_PIN, INPUT);  // MKO change to INPUT, not INPUT_PULLUP
  pinMode(SENSOR2_PIN, INPUT);
  pinMode(SENSOR3_PIN, INPUT);
  pinMode(SENSOR4_PIN, INPUT); 

// for test (pull-up inputs)
  digitalWrite(SENSOR1_PIN, HIGH);
  digitalWrite(SENSOR2_PIN, HIGH);
  digitalWrite(SENSOR3_PIN, HIGH);
  digitalWrite(SENSOR4_PIN, HIGH);

  pinMode(LED0_PIN,   OUTPUT);   
  pinMode(LED1_PIN,   OUTPUT);
  pinMode(LED2_PIN,   OUTPUT);
  pinMode(LED3_PIN,   OUTPUT);
  pinMode(LED4_PIN,   OUTPUT);
  pinMode(SERVO_PIN,  OUTPUT);

  Serial.begin(9600);
  Serial.println("Railway Crossing Control Ready");
  Serial.println();
  Serial.println("Waiting for train");


  startPlayback();  // MKO Bell sound playback initiated.
  angle = 0;

  Serial.print("angle = ");
  Serial.println(angle);
  Serial.print("angle + DOWN = ");
  Serial.println(angle+DOWN); 
  Serial.print("state = ");
  Serial.println(state);
  Serial.print("transition = ");
  Serial.println(transition);

blink_speed = 2*(255/fade_delay + time_to_stop);
angle = 1;
openGate();
}


void loop() {

if((digitalRead(SENSOR1_PIN) == LOW) or (digitalRead(SENSOR3_PIN) == LOW))
{
  trains = trains + 1;
  delay(500);
}


if((digitalRead(SENSOR2_PIN) == LOW) or (digitalRead(SENSOR4_PIN) == LOW))
{
  trains = trains - 1;
  delay(500);
}
  
  switch(state) {
    case 1: // idle
//      if((digitalRead(SENSOR1_PIN) == LOW) or (digitalRead(SENSOR3_PIN) == LOW)) 
    if (trains != 0)
      {
        transition = 12;
        Serial.print("state = ");
        Serial.println(state);
        Serial.print("transition = ");
        Serial.println(transition);
      }
    break;
 
    case 2: // blinking, gate still open
      if (millis() > time_to_close_gate) 
      {
        transition = 23;
        Serial.print("state = ");
        Serial.println(state);
        Serial.print("transition = ");
        Serial.println(transition);
      }
    break;
 
    case 3: // blinking, gate closing
//      if((digitalRead(SENSOR2_PIN) == LOW) or (digitalRead(SENSOR4_PIN) == LOW))
       if (trains == 0)
      {
        transition = 34;
        Serial.print("state = ");
        Serial.println(state);
        Serial.print("transition = ");
        Serial.println(transition);
      }
    break;
 
    case 4: // blinking, gate opening
       transition = 41;
       sound_enabled = 0;
        Serial.print("state = ");
        Serial.println(state);
        Serial.print("transition = ");
        Serial.println(transition);
    break;
  }


  switch(transition) {
    case 12: //
      Serial.println("Train detected, start blinking");
      blink_enabled = 1;
      sound_enabled = 1;
      time_to_close_gate = millis() + (unsigned long)GATE_DELAY;
      transition = 0;
      state = 2;
        Serial.print("state = ");
        Serial.println(state);
        Serial.print("transition = ");
        Serial.println(transition);      
    break;
 
    case 23: //
      Serial.println("Time to close the gate");
      closeGate();
      sound_enabled = 1;
      transition = 0;
      state = 3;
        Serial.print("state = ");
        Serial.println(state);
        Serial.print("transition = ");
        Serial.println(transition);
    break;
 
    case 34:
      Serial.println("Train detected, open the gate");
      sound_enabled = 0;
      openGate();    
      transition = 0;
      state = 4;
        Serial.print("state = ");
        Serial.println(state);
        Serial.print("transition = ");
        Serial.println(transition);
    break;
 
    case 41:
      Serial.println("Gate is open, stop blinking");
      Serial.println();
      Serial.println("Waiting for train");
      blink_enabled = 0; led1 = 0; led2 = 0;
      sound_enabled = 0;
      digitalWrite(LED1_PIN, led1);
      digitalWrite(LED2_PIN, led2);
      digitalWrite(LED0_PIN, 0);
      transition = 0;
      state = 1;
        Serial.print("state = ");
        Serial.println(state);
        Serial.print("transition = ");
        Serial.println(transition);
    break;
  }


  if(blink_enabled == 1) 
  {
    blinkLights(); 
    fadeLights();
  }
  else noLights();

if (trains0 != trains)
{
  Serial.print("trains = ");
  Serial.println(trains);
}
trains0 = trains;
  } // end main loop
