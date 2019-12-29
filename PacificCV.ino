//  Prior Art:
//    https://playground.arduino.cc/Code/CapacitiveSensor

// * This program is free software; you can redistribute it and/or modify
// * it under the terms of the GNU General Public License as published by
// * the Free Software Foundation; either version 3 of the License, or
// * (at your option) any later version.


//  Middle C = midi note 60, defaults are E3 (52)-F4 (77)
//  Octave will default to 4 using iOS standard
#include <Wire.h>
#include <Adafruit_MCP4725.h>
#include "pins_arduino.h" // Arduino pre-1.0 needs this

Adafruit_MCP4725 noteVoltage;//volt-per-octave
Adafruit_MCP4725 pressureVoltage;//Pressure (expression)

#define octaveUp 25
#define octaveDown 24
#define bendDown 2
#define bendUp 3
#define gatePin 7
#define semitoneValue 68.25

int noteValue, velocity,  detect, lastRead, minCap, maxCap, 
    octave, octaveUpDetect, octaveDownDetect,lastOctaveUp,lastOctaveDown,
    bendUpDetect, bendDownDetect, bendAmount;
void setup(){
  //note keys
  for (int thisPin = 27; thisPin<=50;thisPin++){
    pinMode(thisPin,INPUT);
  }
  //octave keys
  pinMode(octaveUp, INPUT);
  pinMode(octaveDown, INPUT);
  pinMode(bendUp, INPUT);
  pinMode(bendDown, INPUT);
  pinMode(gatePin, OUTPUT);
  lastRead = 0;
  minCap = 5;
  maxCap = 12;
  octave = 2;
  digitalWrite(gatePin, LOW);
  noteVoltage.begin(0x62);
  pressureVoltage.begin(0x63);
}
void loop(){
  //kill last note if released
  detect = readCapacitivePin(lastRead);
  if (detect<minCap) {digitalWrite(gatePin, LOW);}
  //octave up or down

  if (readCapacitivePin(25)>3){
    octave = ((lastOctaveUp==0)&&(octave<3))?octave+1:octave;
    lastOctaveUp=1;
   }else{lastOctaveUp=0;}//debounce
  if (readCapacitivePin(24)>3){
    octave = ((lastOctaveDown==0)&&(octave>-1))?octave-1:octave;
    lastOctaveDown=1;
   }else{lastOctaveDown=0;}  //debounce
    //pitch bend
  bendUpDetect = readCapacitivePin(bendUp);
  bendDownDetect = readCapacitivePin(bendDown);
  if (bendUpDetect>minCap or bendDownDetect>minCap){
    if (bendUpDetect>bendDownDetect){
        bendAmount = map(bendUpDetect, minCap, maxCap, 5, 60);
    }
     if (bendUpDetect<bendDownDetect){
        bendAmount = -1*(map(bendDownDetect, minCap, maxCap, 5, 60));
      }
    }else{
      bendAmount=0;
    }
  //main play loop
  for (int key = 50; key >=27; key--){//mono with high-note priority
    detect = readCapacitivePin (key);
    if (detect >= minCap){
      velocity = map(detect, minCap, maxCap, 1, 4095);
      noteValue = ((octave==-1)&&(key<34))?0:(((octave*12)+(key-22))*semitoneValue);
      noteVoltage.setVoltage(noteValue, false);
      pressureVoltage.setVoltage(velocity, false);
      digitalWrite(gatePin, HIGH);
      lastRead = key;
      break;
    }
  }

}


uint8_t readCapacitivePin(int pinToMeasure) {
  // Variables used to translate from Arduino to AVR pin naming
  volatile uint8_t* port;
  volatile uint8_t* ddr;
  volatile uint8_t* pin;
  // Here we translate the input pin number from
  //  Arduino pin number to the AVR PORT, PIN, DDR,
  //  and which bit of those registers we care about.
  byte bitmask;
  port = portOutputRegister(digitalPinToPort(pinToMeasure));
  ddr = portModeRegister(digitalPinToPort(pinToMeasure));
  bitmask = digitalPinToBitMask(pinToMeasure);
  pin = portInputRegister(digitalPinToPort(pinToMeasure));
  // Discharge the pin first by setting it low and output
  *port &= ~(bitmask);
  *ddr  |= bitmask;
  delay(1);
  uint8_t SREG_old = SREG; //back up the AVR Status Register
  // Prevent the timer IRQ from disturbing our measurement
  noInterrupts();
  // Make the pin an input with the internal pull-up on
  *ddr &= ~(bitmask);
  *port |= bitmask;

  // Now see how long the pin to get pulled up. This manual unrolling of the loop
  // decreases the number of hardware cycles between each read of the pin,
  // thus increasing sensitivity.
  uint8_t cycles = 17;
  if (*pin & bitmask) { cycles =  0;}
  else if (*pin & bitmask) { cycles =  1;}
  else if (*pin & bitmask) { cycles =  2;}
  else if (*pin & bitmask) { cycles =  3;}
  else if (*pin & bitmask) { cycles =  4;}
  else if (*pin & bitmask) { cycles =  5;}
  else if (*pin & bitmask) { cycles =  6;}
  else if (*pin & bitmask) { cycles =  7;}
  else if (*pin & bitmask) { cycles =  8;}
  else if (*pin & bitmask) { cycles =  9;}
  else if (*pin & bitmask) { cycles = 10;}
  else if (*pin & bitmask) { cycles = 11;}
  else if (*pin & bitmask) { cycles = 12;}
  else if (*pin & bitmask) { cycles = 13;}
  else if (*pin & bitmask) { cycles = 14;}
  else if (*pin & bitmask) { cycles = 15;}
  else if (*pin & bitmask) { cycles = 16;}

  // End of timing-critical section; turn interrupts back on if they were on before, or leave them off if they were off before
  SREG = SREG_old;

  // Discharge the pin again by setting it low and output
  //  It's important to leave the pins low if you want to 
  //  be able to touch more than 1 sensor at a time - if
  //  the sensor is left pulled high, when you touch
  //  two sensors, your body will transfer the charge between
  //  sensors.
  *port &= ~(bitmask);
  *ddr  |= bitmask;

  return cycles;
}
