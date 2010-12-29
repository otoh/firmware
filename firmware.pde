/*
  Copyright (C) 2006-2008 Hans-Christoph Steiner.  All rights reserved.
  
  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.
 
  See file LICENSE.txt for further informations on licensing terms.

  formatted using the GNU C formatting and indenting
*/

/* 
 * TODO: use Program Control to load stored profiles from EEPROM
 */

#include <Servo.h>
#include <Firmata.h>

////////////////////////////////////////
// WAVEFORM + TIMELINE
////////////////////////////////////////
#include <Maxim.h>

#define OTOH_TIMELINE_MAX       2
#define OTOH_TIMELINE_REGISTERS 7
#define OTOH_TIMELINE_COLUMNS   8

// Maxim constants
#define OTOH_DATA_IN    12
#define OTOH_LOAD       13
#define OTOH_CLOCK      11
#define OTOH_MAX_IN_USE 3

// Shiftin constants
#define OTOH_SI_LATCH  9
#define OTOH_SI_DATA   10
#define OTOH_SI_CLOCK  8

boolean debug = false;

int values[] = {1, 2, 4, 8, 16, 32, 64, 128};

Maxim maxim(OTOH_DATA_IN, OTOH_LOAD, OTOH_CLOCK, OTOH_MAX_IN_USE);

/*==============================================================================
 * GLOBAL VARIABLES
 *============================================================================*/

/* analog inputs */
int analogInputsToReport = 0; // bitwise array to store pin reporting

/* digital input ports */
byte reportPINs[TOTAL_PORTS];       // 1 = report this port, 0 = silence
byte previousPINs[TOTAL_PORTS];     // previous 8 bits sent

/* pins configuration */
byte pinConfig[TOTAL_PINS];         // configuration of every pin
byte portConfigInputs[TOTAL_PORTS]; // each bit: 1 = pin in INPUT, 0 = anything else

/* timer variables */
unsigned long currentMillis;        // store the current value from millis()
unsigned long previousMillis;       // for comparison with currentMillis
int samplingInterval = 19;          // how often to run the main loop (in ms)

Servo servos[MAX_SERVOS];

/*==============================================================================
 * FUNCTIONS
 *============================================================================*/

void outputPort(byte portNumber, byte portValue, byte forceSend)
{
  // pins not configured as INPUT are cleared to zeros
  portValue = portValue & portConfigInputs[portNumber];
  // only send if the value is different than previously sent
  if(forceSend || previousPINs[portNumber] != portValue) {
    Firmata.sendDigitalPort(portNumber, portValue);
    previousPINs[portNumber] = portValue;
  }
}

/* -----------------------------------------------------------------------------
 * check all the active digital inputs for change of state, then add any events
 * to the Serial output queue using Serial.print() */
void checkDigitalInputs(void)
{
  /* Using non-looping code allows constants to be given to readPort().
   * The compiler will apply substantial optimizations if the inputs
   * to readPort() are compile-time constants. */
  if (TOTAL_PORTS > 0 && reportPINs[0]) outputPort(0, readPort(0), false);
  if (TOTAL_PORTS > 1 && reportPINs[1]) outputPort(1, readPort(1), false);
  if (TOTAL_PORTS > 2 && reportPINs[2]) outputPort(2, readPort(2), false);
  if (TOTAL_PORTS > 3 && reportPINs[3]) outputPort(3, readPort(3), false);
  if (TOTAL_PORTS > 4 && reportPINs[4]) outputPort(4, readPort(4), false);
  if (TOTAL_PORTS > 5 && reportPINs[5]) outputPort(5, readPort(5), false);
  if (TOTAL_PORTS > 6 && reportPINs[6]) outputPort(6, readPort(6), false);
  if (TOTAL_PORTS > 7 && reportPINs[7]) outputPort(7, readPort(7), false);
  if (TOTAL_PORTS > 8 && reportPINs[8]) outputPort(8, readPort(8), false);
  if (TOTAL_PORTS > 9 && reportPINs[9]) outputPort(9, readPort(9), false);
  if (TOTAL_PORTS > 10 && reportPINs[10]) outputPort(10, readPort(10), false);
  if (TOTAL_PORTS > 11 && reportPINs[11]) outputPort(11, readPort(11), false);
  if (TOTAL_PORTS > 12 && reportPINs[12]) outputPort(12, readPort(12), false);
  if (TOTAL_PORTS > 13 && reportPINs[13]) outputPort(13, readPort(13), false);
  if (TOTAL_PORTS > 14 && reportPINs[14]) outputPort(14, readPort(14), false);
  if (TOTAL_PORTS > 15 && reportPINs[15]) outputPort(15, readPort(15), false);
}

// -----------------------------------------------------------------------------
/* sets the pin mode to the correct state and sets the relevant bits in the
 * two bit-arrays that track Digital I/O and PWM status
 */
void setPinModeCallback(byte pin, int mode)
{
  if (IS_PIN_SERVO(pin) && mode != SERVO && servos[PIN_TO_SERVO(pin)].attached()) {
    servos[PIN_TO_SERVO(pin)].detach();
  }
  if (IS_PIN_ANALOG(pin)) {
    reportAnalogCallback(PIN_TO_ANALOG(pin), mode == ANALOG ? 1 : 0); // turn on/off reporting
  }
  if (IS_PIN_DIGITAL(pin)) {
    if (mode == INPUT) {
      portConfigInputs[pin/8] |= (1 << (pin & 7));
    } else {
      portConfigInputs[pin/8] &= ~(1 << (pin & 7));
    }
  }
  switch(mode) {
  case ANALOG:
    if (IS_PIN_ANALOG(pin)) {
      if (IS_PIN_DIGITAL(pin)) {
        pinMode(PIN_TO_DIGITAL(pin), INPUT); // disable output driver
        digitalWrite(PIN_TO_DIGITAL(pin), LOW); // disable internal pull-ups
      }
      pinConfig[pin] = ANALOG;
    }
    break;
  case INPUT:
    if (IS_PIN_DIGITAL(pin)) {
      pinMode(PIN_TO_DIGITAL(pin), INPUT); // disable output driver
      digitalWrite(PIN_TO_DIGITAL(pin), LOW); // disable internal pull-ups
      pinConfig[pin] = INPUT;
    }
    break;
  case OUTPUT:
    if (IS_PIN_DIGITAL(pin)) {
      digitalWrite(PIN_TO_DIGITAL(pin), LOW); // disable PWM
      pinMode(PIN_TO_DIGITAL(pin), OUTPUT);
      pinConfig[pin] = OUTPUT;
    }
    break;
  case PWM:
    if (IS_PIN_PWM(pin)) {
      pinMode(PIN_TO_PWM(pin), OUTPUT);
      pinConfig[pin] = PWM;
    }
    break;
  case SERVO:
    if (IS_PIN_SERVO(pin)) {
      pinConfig[pin] = SERVO;
      if (!servos[PIN_TO_SERVO(pin)].attached()) {
          servos[PIN_TO_SERVO(pin)].attach(PIN_TO_DIGITAL(pin));
      } else {
        Firmata.sendString("Servo only on pins from 2 to 13");
      }
    }
    break;
  case I2C:
    pinConfig[pin] = mode;
    Firmata.sendString("I2C mode not yet supported");
    break;
  default:
    Firmata.sendString("Unknown pin mode"); // TODO: put error msgs in EEPROM
  }
  // TODO: save status to EEPROM here, if changed
}

void analogWriteCallback(byte pin, int value)
{
  if (pin < TOTAL_PINS) {
    switch(pinConfig[pin]) {
    case SERVO:
      if (IS_PIN_SERVO(pin))
        servos[PIN_TO_SERVO(pin)].write(value);
      break;
    case PWM:
      if (IS_PIN_PWM(pin))
        analogWrite(PIN_TO_PWM(pin), value);
      break;
    }
  }
}

void digitalWriteCallback(byte port, int value)
{
  byte pin, lastPin, mask=1, pinWriteMask=0;

  if (port < TOTAL_PORTS) {
    // create a mask of the pins on this port that are writable.
    lastPin = port*8+8;
    if (lastPin > TOTAL_PINS) lastPin = TOTAL_PINS;
    for (pin=port*8; pin < lastPin; pin++) {
      // do not disturb non-digital pins (eg, Rx & Tx)
      if (IS_PIN_DIGITAL(pin)) {
        // only write to OUTPUT and INPUT (enables pullup)
        // do not touch pins in PWM, ANALOG, SERVO or other modes
        if (pinConfig[pin] == OUTPUT || pinConfig[pin] == INPUT) {
          pinWriteMask |= mask;
        }
      }
      mask = mask << 1;
    }
    writePort(port, (byte)value, pinWriteMask);
  }
}


// -----------------------------------------------------------------------------
/* sets bits in a bit array (int) to toggle the reporting of the analogIns
 */
//void FirmataClass::setAnalogPinReporting(byte pin, byte state) {
//}
void reportAnalogCallback(byte analogPin, int value)
{
  if (analogPin < TOTAL_ANALOG_PINS) {
    if(value == 0) {
      analogInputsToReport = analogInputsToReport &~ (1 << analogPin);
    } else {
      analogInputsToReport = analogInputsToReport | (1 << analogPin);
    }
  }
  // TODO: save status to EEPROM here, if changed
}

void reportDigitalCallback(byte port, int value)
{
  reportPINs[port] = (byte)value;
  // do not disable analog reporting on these 8 pins, to allow some
  // pins used for digital, others analog.  Instead, allow both types
  // of reporting to be enabled, but check if the pin is configured
  // as analog when sampling the analog inputs.  Likewise, while
  // scanning digital pins, portConfigInputs will mask off values from any
  // pins configured as analog
}

/*==============================================================================
 * SYSEX-BASED commands
 *============================================================================*/

////////////////////////////////////////////////////////
////////////////////////////////////////////////////////
////////////////////////////////////////////////////////
////////////////////////////////////////////////////////
////////////////////////////////////////////////////////

#define BLINK_LED 0x02 // 2
#define TURN_ON   0x03 // 3
#define TURN_OFF  0x04 // 4
int ledPin = 13; // LED connected to digital pin 13
boolean ledIsOn = false;
boolean blinkLedOn = false;
unsigned long blinkPreviousMillis;  // for comparison with currentMillis
int delayValue;

void turnOn()
{
  //digitalWrite(ledPin, HIGH);   // set the LED on
  maxim.one(2, 1, 1);
  ledIsOn = true;
}

void turnOff()
{
  //digitalWrite(ledPin, LOW);    // set the LED off
  maxim.one(2, 1, 0);
  ledIsOn = false;
}

void toggleLed() {
  if (ledIsOn) {
    turnOff();
  }
  else {
    turnOn();
  }
}

void blinkLed()
{
  if (blinkLedOn && (currentMillis - blinkPreviousMillis > delayValue / 2)) {
    blinkPreviousMillis += delayValue / 2;
    toggleLed();
  }
}

////////////////////////////////////////////////
// TIMELINE
////////////////////////////////////////////////
#define TIMELINE_START 0x05 // 5
#define TIMELINE_STOP  0x06 // 6

#define TIMELINE_DEFAULT_OFFSET 32
#define TIMELINE_LENGTH         56

int timelineMax = 2;
int timelineRegConversion[] = {1,5,7,3,4,6,2};
int timelineColConversion[] = {8,64,2,32,1,16,4,128};
unsigned long timelinePreviousMillis[] = {0, 0};  // for comparison with currentMillis
int timelineSpeed[] = {250, 250};
boolean timelineIsOn[] = {false, false};
int timelineOffset[] = {11, 3};
int timelineOrientation[] = {false, false}; // clockwise = true
int timelineLimit[] = {TIMELINE_LENGTH, TIMELINE_LENGTH};
int ti_0 = timelineOrientation[0] ? 0 : timelineLimit[0];
int ti_1 = timelineOrientation[1] ? 0 : timelineLimit[1];
int ti[] = {ti_0, ti_1};
int previous_reg_0 = (timelineOffset[0] / OTOH_TIMELINE_COLUMNS) % OTOH_TIMELINE_REGISTERS;
int previous_reg_1 = (timelineOffset[1] / OTOH_TIMELINE_COLUMNS) % OTOH_TIMELINE_REGISTERS;
int previous_reg[] = {previous_reg_0, previous_reg_1};
int reg[] = {0, 0};
int col[] = {0, 0};
int timelineColVal;

void timelineAsyncCall() {
  for(int i = 0; i < 2; i++) {
    if (timelineIsOn[i] && (currentMillis - timelinePreviousMillis[i] > timelineSpeed[i])) {
      timelinePreviousMillis[i] += timelineSpeed[i];
      timelineAsync(i);
    }
  }
}

void timelineAsync(int i) {
  reg[i] = ((ti[i]+timelineOffset[i]) / OTOH_TIMELINE_COLUMNS) % OTOH_TIMELINE_REGISTERS;
  col[i] = (ti[i]+timelineOffset[i]) % OTOH_TIMELINE_COLUMNS;
  
  if (reg[0] == reg[1] && timelineIsOn[0] && timelineIsOn[1]) {
    timelineColVal = timelineColConversion[col[0]] ^ timelineColConversion[col[1]];
  }
  else {
    timelineColVal = timelineColConversion[col[i]];
  }
  
  maxim.one(OTOH_TIMELINE_MAX, timelineRegConversion[reg[i]], timelineColVal);
  
  if(debug) {
    Serial.print("\n");
    Serial.print(i);
    Serial.print(" -- ");
    Serial.print(ti[i]);
    Serial.print(" -- ");
    Serial.print(timelineRegConversion[reg[i]]);
    Serial.print(", ");
    Serial.print(timelineColVal);
  }
  
  if(previous_reg[i] != reg[i]) {
    if (previous_reg[i] == reg[1] || previous_reg[i] == reg[0]) {
      if(debug) {
        Serial.print(" --> YOO! -->  ");
      }
      
      if(timelineIsOn[-1*(1-i)]) {
        timelineColVal = timelineColConversion[col[-1*(1-i)]];
      }
      else {
        timelineColVal = 0;
      }
    }
    else {
      timelineColVal = 0;
    }
    
    maxim.one(OTOH_TIMELINE_MAX, timelineRegConversion[previous_reg[i]], timelineColVal);
    
    if(debug) {
      Serial.print(" --> (previous_reg[i] != reg[i]) -->  ");
      Serial.print(i);
      Serial.print(" -- ");
      Serial.print(ti[i]);
      Serial.print(" -- ");
      Serial.print(timelineRegConversion[reg[i]]);
      Serial.print(", ");
      Serial.print(timelineColVal);
    }
  }
  
  previous_reg[i] = reg[i];
  
  if (timelineOrientation[i]) {
    ti[i]++;
    
    if(ti[i] > timelineLimit[i]-1) {
      ti[i] = 0;
    }
  }
  else {
    ti[i]--;
    
    if(ti[i] < 1) {
      ti[i] = timelineLimit[i];
    }
  }
}

void timelineReset() {
  for(int reg = 1; reg < 8; reg++) {
    timelineColVal = 0;
    maxim.one(OTOH_TIMELINE_MAX, reg, timelineColVal);
  }
}

void timelineStartCallback(byte argc, byte *argv) {
  timelineReset();
  
  // timeline id
  // argv[0] --> [1, 2] --> timeline [0, 1]
  int t_id = argv[0] - 1;
  timelineIsOn[t_id] = true;
  
  // timeline speed
  // argv[1..2] --> speed [0,16383] ms
  timelineSpeed[t_id] = argv[1] + (argv[2] << 7);
  
  // timeline orientation
  // argv[3] --> [1, 2] --> [true, false] --> [clockwise, anticlockwise]
  timelineOrientation[t_id] = (1 == argv[3] ? true : false);
  
  // timeline offset (start point)
  // argv[4] --> [0..55]
  timelineOffset[t_id] = (TIMELINE_DEFAULT_OFFSET + argv[4]) % TIMELINE_LENGTH;
  
  // timeline limit (end point)
  // argv[5] --> [1,56]
  timelineLimit[t_id] = argv[5];
  
  ti[t_id] = timelineOrientation[t_id] ? 0 : timelineLimit[t_id];
  previous_reg[t_id] = (timelineOffset[t_id] / OTOH_TIMELINE_COLUMNS) % OTOH_TIMELINE_REGISTERS;
  timelinePreviousMillis[t_id] = currentMillis;
}

void timelineStopCallback(byte argc, byte *argv) {
  // timeline id
  // argv[0] --> [1, 2] --> timeline [0, 1]
  int t_id = argv[0] - 1;
  timelineIsOn[t_id] = false;
  
  //timelinePreviousMillis[t_id] = 0;
  timelineReset();
}

////////////////////////////////////////////////
// WAVEFORM
////////////////////////////////////////////////
#define WAVEFORM       0x07 // 7
#define WAVEFORM_CLEAR 0x08 // 8

int waveformMax[] = {1, 3};
int waveformRegConversion[] = {1,5,7,3,4,8,6,2};
int waveformColConversion[] = {64,2,32,1,8,128,4,16};
int waveformRegValue[][8] = {{0,0,0,0,0,0,0,0},{0,0,0,0,0,0,0,0}};
int vel = 100;

// waveform examples
int cuorizini[] = {4,14,7,14,4,0,4,14,7,14,4,0,4,14,7,14,4,0,4,14,7,14,4,0,4,14,7,14,4,0,0,0};
int otohWF[]    = {0,6,9,6,0,8,15,8,0,6,9,6,0,15,2,15,0,6,9,6,0,8,15,8,0,6,9,6,0,15,2,15};
int wave[]      = {1,3,7,15,7,3,1,3,7,15,7,3,1,3,7,15,7,3,1,3,7,15,7,3,1,3,7,15,7,3,1,0};
int squares[]   = {15,9,9,15,15,9,9,15,15,9,9,15,15,9,9,15,15,9,9,15,15,9,9,15,15,9,9,15,15,9,9,15};
int triangles[] = {1,3,7,15,1,3,7,15,1,3,7,15,1,3,7,15,1,3,7,15,1,3,7,15,1,3,7,15,1,3,7,15};

// x   => [0, 31]
// val => [0, 15]
void waveformFuncOn(int x, byte val) {
  if(x > 15) {
    if(x < 24) {
      x += 8;
    }
    else {
      x -= 8;
    }
  }
  
  int waveformMaxNum = x / 16;
  int reg, col;
  
  x %= 16;
  
  if(x < 8) {
    reg = -1 * ((x % 8) - 7);
  }
  else {
    reg = x % 8;
  }
  
  if(x < 8) {
    for(int i = 0; i < 4; i++) {
      if((val >> i) & 1) {
        if(0 == waveformMaxNum) {
          waveformRegValue[waveformMaxNum][reg] ^= waveformColConversion[i+4];
        }
        else {
          waveformRegValue[waveformMaxNum][reg] ^= waveformColConversion[i];
        }
      }
    }
  }
  else {
    for(int i = 0; i < 4; i++) {
      if((val >> i) & 1) {
        if(0 == waveformMaxNum) {
          waveformRegValue[waveformMaxNum][reg] ^= waveformColConversion[i];
        }
        else {
          waveformRegValue[waveformMaxNum][reg] ^= waveformColConversion[i+4];
        }
      }
    }
  }
  
  if(debug) {
    Serial.print("\n");
    Serial.print(x);
    Serial.print(", ");
    Serial.print(waveformMaxNum);
    Serial.print(", ");
    Serial.print(waveformRegConversion[reg]);
    Serial.print(", ");
    Serial.print(waveformRegValue[waveformMaxNum][reg]);
  }
  
  maxim.one(waveformMax[waveformMaxNum], waveformRegConversion[reg], waveformRegValue[waveformMaxNum][reg]);
}

void waveformReset() {
  for(int i = 0; i < 2; i++) {
    for(int reg = 0; reg < 8; reg++) {
      waveformRegValue[i][reg] = 0;
      maxim.one(waveformMax[i], waveformRegConversion[reg], 0);
    }
  }
}

void waveform() {
  for(int i = 0; i < 32; i++) {
//    waveformFuncOn(i, cuorizini[i]);
    waveformFuncOn(i, otohWF[i]);
//    waveformFuncOn(i, wave[i]);
//    waveformFuncOn(i, squares[i]);
//    waveformFuncOn(i, triangles[i]);
    delay(vel);
  }
}

void waveformCallback(byte argc, byte *argv) {
  // argv[0] --> [1..32] --> waveform reg [0,31]
  int reg = argv[0] - 1;
  // argv[0] --> [1..16] --> waveform reg [0,15]
  byte col = argv[1] - 1;
  waveformFuncOn(reg,col);
}

void waveformClearCallback() {
  waveformReset();
}

////////////////////////////////////////////////////////
// SHIFTIN
////////////////////////////////////////////////////////
#define SHIFTIN_MESSAGE 0xA0

unsigned long shiftInPreviousMillis = 0;  // for comparison with currentMillis
int shiftInSamplingInterval = 10;         // how often to run the main loop (in ms)

byte switchVar1 = 0;
byte switchVar2 = 0;

void setupShiftIn() {
  pinMode(OTOH_SI_LATCH, OUTPUT);
  pinMode(OTOH_SI_CLOCK, OUTPUT);
  pinMode(OTOH_SI_DATA, INPUT);
}

byte shiftIn(int myDataPin, int myClockPin) {
  int i;
  int temp = 0;
  int pinState;
  byte myDataIn = 0;
  
  pinMode(myClockPin, OUTPUT);
  pinMode(myDataPin, INPUT);
  
  for (i=7; i>=0; i--){
    digitalWrite(myClockPin, 0);
    delayMicroseconds(1);
    
    temp = digitalRead(myDataPin);
    if (temp) {
    
      pinState = 1;
      myDataIn = myDataIn | (1 << i);
    
    }else {
      pinState = 0;
    }
    
    digitalWrite(myClockPin, 1);
  }
  return myDataIn;
}

void loopShiftIn() {
  if (currentMillis - shiftInPreviousMillis > shiftInSamplingInterval) {
    shiftInPreviousMillis += shiftInSamplingInterval;
    
    digitalWrite(OTOH_SI_LATCH,1);
    delayMicroseconds(20);
    digitalWrite(OTOH_SI_LATCH,0);
    switchVar1 = shiftIn(OTOH_SI_DATA, OTOH_SI_CLOCK);
    switchVar2 = shiftIn(OTOH_SI_DATA, OTOH_SI_CLOCK);
    Serial.print(SHIFTIN_MESSAGE, BYTE);
    Serial.print(switchVar1, BYTE);
    Serial.print(switchVar2, BYTE);
  }
}

////////////////////////////////////////////////////////
////////////////////////////////////////////////////////
////////////////////////////////////////////////////////
////////////////////////////////////////////////////////
////////////////////////////////////////////////////////

void sysexCallback(byte command, byte argc, byte *argv)
{
  switch(command) {
  case SERVO_CONFIG:
    if(argc > 4) {
      // these vars are here for clarity, they'll optimized away by the compiler
      byte pin = argv[0];
      int minPulse = argv[1] + (argv[2] << 7);
      int maxPulse = argv[3] + (argv[4] << 7);

      if (IS_PIN_SERVO(pin)) {
        // servos are pins from 2 to 13, so offset for array
        if (servos[PIN_TO_SERVO(pin)].attached())
          servos[PIN_TO_SERVO(pin)].detach();
        servos[PIN_TO_SERVO(pin)].attach(PIN_TO_DIGITAL(pin), minPulse, maxPulse);
        setPinModeCallback(pin, SERVO);
      }
    }
    break;
  case BLINK_LED:
    if(argc > 1) {
      delayValue = argv[0] + (argv[1] << 7);
      blinkLedOn = true;
    }
    else {
      blinkLedOn = false;
      Firmata.sendString("Wrong number of params");
    }
    break;
  case TURN_ON:
    turnOn();
    break;
  case TURN_OFF:
    turnOff();
    break;
  case TIMELINE_START:
    timelineStartCallback(argc, argv);
    break;
  case TIMELINE_STOP:
    timelineStopCallback(argc, argv);
    break;
  case WAVEFORM:
    waveformCallback(argc, argv);
    break;
  case WAVEFORM_CLEAR:
    waveformClearCallback();
    break;
  case SAMPLING_INTERVAL:
    if (argc > 1)
      samplingInterval = argv[0] + (argv[1] << 7);
    else
      Firmata.sendString("Not enough data");
    break;
  default:
    Firmata.sendString("SYSEX command not found");
    toggleLed();
  }
}


/*==============================================================================
 * SETUP()
 *============================================================================*/
void setup() 
{
  //waveform();
  
  // initialize shift in
  setupShiftIn();
  
  // initialize the digital pin as an output:
  pinMode(ledPin, OUTPUT);
  
  byte i;

  Firmata.setFirmwareVersion(2, 1);

  Firmata.attach(ANALOG_MESSAGE, analogWriteCallback);
  Firmata.attach(DIGITAL_MESSAGE, digitalWriteCallback);
  Firmata.attach(REPORT_ANALOG, reportAnalogCallback);
  Firmata.attach(REPORT_DIGITAL, reportDigitalCallback);
  Firmata.attach(SET_PIN_MODE, setPinModeCallback);
  Firmata.attach(START_SYSEX, sysexCallback);

  // TODO: load state from EEPROM here

  /* these are initialized to zero by the compiler startup code
  for (i=0; i < TOTAL_PORTS; i++) {
    reportPINs[i] = false;
    portConfigInputs[i] = 0;
    previousPINs[i] = 0;
  }
  */
  for (i=0; i < TOTAL_PINS; i++) {
    if (IS_PIN_ANALOG(i)) {
      // turns off pullup, configures everything
      setPinModeCallback(i, ANALOG);
    } else {
      // sets the output to 0, configures portConfigInputs
      setPinModeCallback(i, OUTPUT);
    }
  }
  // by defult, do not report any analog inputs
  analogInputsToReport = 0;

  Firmata.begin(57600);

  /* send digital inputs to set the initial state on the host computer,
   * since once in the loop(), this firmware will only send on change */
  for (i=0; i < TOTAL_PORTS; i++) {
    outputPort(i, readPort(i), true);
  }
  
  //
  // FIX ME: this doesn't work!
  //
  // initialize digital ins
  setPinModeCallback(2, INPUT);
  setPinModeCallback(3, INPUT);
  setPinModeCallback(4, INPUT);
  setPinModeCallback(5, INPUT);
  setPinModeCallback(6, INPUT);
  setPinModeCallback(7, INPUT);
}

/*==============================================================================
 * LOOP()
 *============================================================================*/
void loop() 
{
  // shift in loop
  currentMillis = millis();
  loopShiftIn();
  
  byte pin, analogPin;

  /* DIGITALREAD - as fast as possible, check for changes and output them to the
   * FTDI buffer using Serial.print()  */
  checkDigitalInputs();  

  /* SERIALREAD - processing incoming messagse as soon as possible, while still
   * checking digital inputs.  */
  while(Firmata.available())
    Firmata.processInput();

  // blinkLed debug function loop
  currentMillis = millis();
  blinkLed();
  
  // timeline loop
  currentMillis = millis();
  timelineAsyncCall();
  
  // NOTE: turn off debug after 3 secs if debug is on
  if(debug && currentMillis > 3000) {
    debug = false;
  }

  /* SEND FTDI WRITE BUFFER - make sure that the FTDI buffer doesn't go over
   * 60 bytes. use a timer to sending an event character every 4 ms to
   * trigger the buffer to dump. */

  currentMillis = millis();
  if (currentMillis - previousMillis > samplingInterval) {
    previousMillis += samplingInterval;
    /* ANALOGREAD - do all analogReads() at the configured sampling interval */
    for(pin=0; pin<TOTAL_PINS; pin++) {
      if (IS_PIN_ANALOG(pin) && pinConfig[pin] == ANALOG) {
        analogPin = PIN_TO_ANALOG(pin);
        if (analogInputsToReport & (1 << analogPin)) {
          Firmata.sendAnalog(analogPin, analogRead(analogPin));
        }
      }
    }
  }
}
