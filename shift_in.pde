////////////////////////////////////////////////////////
// OTOH SHIFT IN
////////////////////////////////////////////////////////

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
