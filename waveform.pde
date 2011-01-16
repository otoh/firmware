////////////////////////////////////////////////
// OTOH WAVEFORM
////////////////////////////////////////////////

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
  // argv[0] --> [1..16] --> waveform col [0,15]
  byte col = argv[1] - 1;
  waveformFuncOn(reg,col);
}

void waveformClearCallback() {
  waveformReset();
}
