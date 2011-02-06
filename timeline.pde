////////////////////////////////////////
// OTOH TIMELINE
////////////////////////////////////////

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

int parseTimelineId(byte argc, byte *argv) {
  // timeline id
  // argv[0] --> [1, 2] --> timeline [0, 1]
  return argv[0] - 1;
}

int parseTimelineSpeed(byte argc, byte *argv) {
  // timeline speed
  // argv[1..2] --> speed [0,16383] ms
  return argv[1] + (argv[2] << 7);
}

void timelineStartCallback(byte argc, byte *argv) {
  timelineReset();
  
  int t_id = parseTimelineId(argc, argv);
  timelineIsOn[t_id] = true;
  
  timelineSpeed[t_id] = parseTimelineSpeed(argc, argv);
  
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
  timelineIsOn[parseTimelineId(argc, argv)] = false;
  
  //timelinePreviousMillis[t_id] = 0;
  timelineReset();
}

void timelineSpeedCallback(byte argc, byte *argv) {
  timelineSpeed[parseTimelineId(argc, argv)] = parseTimelineSpeed(argc, argv);
}
