// inital changes by MKO
// ver.2 - niq_ro added bidirectional trigger.. not good solution
// ver.2.x - niq_ro used parts from https://rudysarduinoprojects.wordpress.com/2020/09/23/fun-with-arduino-42-railway-crossing-multi-track-two-way/


int UP = 1400;  //2300;     // MKO Upper limit of gate travel; adjust these two values to get the travel required
int DOWN = 1100; //1100;   // MKO lower limit of gate travel
int GateSpeed = 3; // MKO set to an integer between say 1 and 8; lower = slower;
                   //   to set speed that gate moves from up to down and back
int angle = 0;     // MKO variable to track angle of ate opening (thougb it has nothing to do with degrees of travel) 

#define xdebug 0 // 1 = detailed info on serial monitor 
#define GATE_DELAY  4000 // [ms] delay time before gate closes once train detected on Sensor 1
#define END_OF_TRAIN_DELAY 2000 // [ms] time to wait before deciding this was the end of the train
#define BLINK_SPEED  400 // [ms] smaller number is faster blinking
#define SERVO_PIN      2  //MKO SERVO_PIN was 3 in JS original sketch. Change to D2
#define LED1_PIN       6
#define LED2_PIN       7
#define LED0_PIN       4

//byte state = 1;  // 1;
byte transition = 0; //0;
byte led1 = 0;            // MKO  Define inital values for these 3 variables
byte led2 = 0;            // MKO                                 |      
byte blink_enabled = 0;   // MKO                                 _


unsigned long time_to_blink;
unsigned long time_to_close_gate = millis();  //MKO set intial millis times


#define NUM_SENSORS           4 // two sensors per track, one left and one right of the gate
byte sensor_pin[NUM_SENSORS]  = {5,8,9,10}; // sensor pin numbers

byte state = 1, train_counter, n;
byte sensor_state[NUM_SENSORS];       // 0 idle, 1 detect arrival, 2 detect departure, 3 detect end of train
byte end_of_train[NUM_SENSORS];       // 0 idle, 1 end of train detected
//unsigned long time_to_blink;
//unsigned long time_to_close_gate;
unsigned long time_for_servo_step;
unsigned long time_end_of_train[NUM_SENSORS];


void blinkLights() {
    if(millis() > time_to_blink) {
      time_to_blink = millis() + (unsigned long)BLINK_SPEED;
      led1 = !led1;
      led2 = !led1;
    
      digitalWrite(LED1_PIN, led1);
      digitalWrite(LED2_PIN, led2);
      digitalWrite(LED0_PIN, (led1+led2)%2); }
}

void noLights() {
      digitalWrite(LED1_PIN, LOW);
      digitalWrite(LED2_PIN, LOW);
      digitalWrite(LED0_PIN, LOW); 
}



void closeGate() {
      while (angle < (UP-DOWN)) {
        angle += GateSpeed;  // Use 5 as the servo PWM increment/decrement
        digitalWrite(SERVO_PIN, HIGH);
        delayMicroseconds(angle+DOWN);    //position
        digitalWrite(SERVO_PIN, LOW);
        delayMicroseconds(20000-angle);   //balance of 20000 cycle
        blinkLights();
    if (xdebug == 1)
        {
        Serial.print("angle = ");
        Serial.println(angle);
        Serial.print("angle + DOWN = ");
        Serial.println(angle+DOWN);    
        Serial.print("state = ");
        Serial.println(state);
        Serial.print("transition = ");
        Serial.println(transition); 
        }
      }
        digitalWrite(SERVO_PIN, LOW); }


void openGate() {
      while ( angle > 0) {
        angle -= GateSpeed;
        digitalWrite(SERVO_PIN, HIGH);
        delayMicroseconds(angle+DOWN);    //position
        digitalWrite(SERVO_PIN, LOW);
        delayMicroseconds(20000-angle);   //balance of 20000 cycle
        blinkLights();
    if (xdebug == 1)
        {
        Serial.print("angle = ");
        Serial.println(angle);
        Serial.print("angle + DOWN = ");
        Serial.println(angle+DOWN); 
        Serial.print("state = ");
        Serial.println(state);
        Serial.print("transition = ");
        Serial.println(transition);
        }
      }
        digitalWrite(SERVO_PIN, LOW); }


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


const int speakerPin = 11; // Can be either 3 or 11, two PWM outputs connected to Timer 2; using 11 in this sketch
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
    else {OCR2A = (led1 || led2) * pgm_read_byte(&sounddata_data[sample]);} // <- first variable after the "+" sign needs to be 0 or 1 as dicated by the LED state


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
  for (byte i = 0; i < NUM_SENSORS; i++) pinMode(sensor_pin[i], INPUT_PULLUP);
  for (byte i = 0; i < NUM_SENSORS; i++) sensor_state[i] = 1; // enable sensors for train detection
     
  pinMode(LED1_PIN,   OUTPUT);
  pinMode(LED2_PIN,   OUTPUT);
  pinMode(LED0_PIN,   OUTPUT);
  pinMode(SERVO_PIN,  OUTPUT);


  Serial.begin(9600);
  Serial.println("Railway Crossing Control Ready");
  Serial.println();
  Serial.println("Waiting for train");

  startPlayback();  // MKO Bell sound playback initiated.
  angle = 0;
      if (xdebug == 1)
        {
          Serial.print("angle = ");
          Serial.println(angle);
          Serial.print("angle + DOWN = ");
          Serial.println(angle+DOWN); 
          Serial.print("state = ");
          Serial.println(state);
          Serial.print("transition = ");
          Serial.println(transition);
        }
angle = 1;
   openGate();
}


void loop() {

  for (byte i = 0; i < NUM_SENSORS; i++) {
    if(sensor_state[i] == 1) { // detect arrival of new train
      if(!digitalRead(sensor_pin[i])) { // train detected
        train_counter++;
        sensor_state[i] = 0;
        if(i%2) n = i - 1; else n = i + 1;
        sensor_state[n] = 2; // buddy sensor departure detection enabled
        Serial.print("Arrival:   ");
        Serial.println(i);
        Serial.print("Trains:    ");
        Serial.println(train_counter);
      }
    }
    else if(sensor_state[i] > 1) {
      if(!digitalRead(sensor_pin[i])) { // departure detected
        time_end_of_train[i] = millis() + (unsigned long)END_OF_TRAIN_DELAY;
        if(i%2) n = i - 1; else n = i + 1;
        sensor_state[n] = 1; // buddy sensor enabled again
        if(sensor_state[i] == 2) {
          Serial.print("Departure: ");
          Serial.println(i);
        }
        sensor_state[i] = 3;
      }
      if(sensor_state[i] == 3) // decide if end of train has passed based on a timer
        if(millis() > time_end_of_train[i]) end_of_train[i] = 1; 
      if(end_of_train[i]) { // this takes care train_counter-- is executed only once
        train_counter--;
        end_of_train[i] = 0;
        sensor_state[i] = 1;
        Serial.print("Trains:    ");
        Serial.println(train_counter);
      }
    }
  }


  
  switch(state) {
    case 1: // idle
    if (train_counter != 0)
      {
        transition = 12;
        if (xdebug == 1)
        {
        Serial.print("state = ");
        Serial.println(state);
        Serial.print("transition = ");
        Serial.println(transition);
        }
      }
    break;
 
    case 2: // blinking, gate still open
      if (millis() > time_to_close_gate) 
      {
        transition = 23;
        if (xdebug == 1)
        {
        Serial.print("state = ");
        Serial.println(state);
        Serial.print("transition = ");
        Serial.println(transition);
        }
      }
    break;
 
    case 3: // blinking, gate closing
       if (train_counter == 0)
      {
        transition = 34;
        if (xdebug == 1)
        {
        Serial.print("state = ");
        Serial.println(state);
        Serial.print("transition = ");
        Serial.println(transition);
        }
      }
    break;
 
    case 4: // blinking, gate opening
       transition = 41;
  if (xdebug == 1)
        {
        Serial.print("state = ");
        Serial.println(state);
        Serial.print("transition = ");
        Serial.println(transition);
        }
    break;
  }


  switch(transition) {
    case 12: //
      Serial.println("Train detected, start blinking");
      blink_enabled = 1;
      time_to_close_gate = millis() + (unsigned long)GATE_DELAY;
      transition = 0;
      state = 2;
      if (xdebug == 1)
        {
        Serial.print("state = ");
        Serial.println(state);
        Serial.print("transition = ");
        Serial.println(transition);
        } 
    break;
 
    case 23: //
      Serial.println("Time to close the gate");
      closeGate();
      transition = 0;
      state = 3;
      if (xdebug == 1)
        {
        Serial.print("state = ");
        Serial.println(state);
        Serial.print("transition = ");
        Serial.println(transition);
        }
    break;
 
    case 34:
      Serial.println("Train detected, open the gate");
      openGate();    
      transition = 0;
      state = 4;
  if (xdebug == 1)
        {
        Serial.print("state = ");
        Serial.println(state);
        Serial.print("transition = ");
        Serial.println(transition);
      }
    break;
 
    case 41:
      Serial.println("Gate is open, stop blinking");
      Serial.println();
      Serial.println("Waiting for train");
      blink_enabled = 0; led1 = 0; led2 = 0;
      digitalWrite(LED1_PIN, led1);
      digitalWrite(LED2_PIN, led2);
      digitalWrite(LED0_PIN, 0);
      transition = 0;
      state = 1;
      if (xdebug == 1)
        {
        Serial.print("state = ");
        Serial.println(state);
        Serial.print("transition = ");
        Serial.println(transition);
        }
    break;
  }


  if(blink_enabled == 1) { blinkLights(); }
  else noLights();

  } // end main loop
