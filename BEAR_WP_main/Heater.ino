void task_heater() {
  //only print when something changes, or not it will cause the watchdog to panic if the radio start time is long
  unsigned long curr_time = millis();
  static unsigned long last_time = 0;  //static long just requires one time initalisation
  static bool heater_state = false;    //actually changes the state, heateOn is a bool for telemetry

  bool last_heater_state = heater_state;



  //start of routine heater operation with safeties//
  // Update state only, we don't want to pre-maturely
  // turn heater on when there is possibly a safety
  // cutoff about to be triggered after this.
  if (curr_time - last_time >= 2000) {
    if (ambient_temp <= TEMP_SP_LOW_DEGC) {
      heater_state = true;
      heaterOn = true;
    } else if (ambient_temp >= TEMP_SP_HIGH_DEGC) {
      heater_state = false;
      heaterOn = false;
    }
    last_time = curr_time;
    /*
    //debug heater state
    Serial.print(F(" Heater State: "));
    Serial.print(heaterOn);
    */
  }

  //Heater operation is a NO GO if these are operating//
  //return statements will return to main loop.

  //do not turn heater On IF RTTY is NOT IDLE
  if (rttyState != RTTY_IDLE) {
    heater_state = false;
    digitalWrite(PIN_HEATER, LOW);
    if (heaterOn) {
      Serial.println("RTTY TX cutoff!");
    }
    heaterOn = false;
    return;
  }

  //do not turn heater On IF cutter is operating
  if (cutterOn == true) {
    heater_state = false;
    digitalWrite(PIN_HEATER, LOW);
    if (heaterOn) {
      Serial.println("CUTTER cutoff!");
    }
    heaterOn = false;
    return;
  }


  // Safeties
  if (ambient_temp > 50.0) {
    digitalWrite(PIN_HEATER, LOW);
    heater_state = false;
    if (heaterOn) {
      Serial.println("Overtemp heater cutoff!");
    }
    heaterOn = false;
    return;
  }
  // Battery likely won't have any capacity left when we've
  // been up long enough to reach -10degC. Don't bother using
  // heater as the voltage sag might kill us.
  // Battery won't survive -50degC, likely erronous reading
  if (ambient_temp < -10.0 || ambient_temp < -50.0) {
    digitalWrite(PIN_HEATER, LOW);
    heater_state = false;
    if (heaterOn) {
      Serial.println("Undertemp heater cutoff!");
    }
    heaterOn = false;
    return;
  }


  // Actually turn on/off heater if safeties not triggered
  // This checks if heater should be on based on some previous logic
  if (heater_state) {
    digitalWrite(PIN_HEATER, HIGH);
    // Only print message if heater state has changed from OFF to ON
    if (last_heater_state != heater_state) {
      Serial.println("Heater on!");
    }
  } else {
    // Only print message if heater state has changed from ON to OFF
    digitalWrite(PIN_HEATER, LOW);
    if (last_heater_state != heater_state) {
      Serial.println("Heater off!");
    }
  }
}

void setup_heater() {
  pinMode(PIN_HEATER, OUTPUT);
  digitalWrite(PIN_HEATER, LOW);
  heaterOn = false;
  Serial.print("Heater setup");
}
