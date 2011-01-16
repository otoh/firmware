////////////////////////////////////////
// OTOH BLINK LED
////////////////////////////////////////

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
