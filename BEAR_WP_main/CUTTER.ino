void Cutter(){

  unsigned long curr_time = millis();
  static unsigned long last_time = 0; //altitude check timer
  static unsigned long cutter_last_time=0; //cutter timer
  //check altitude
  //if altitude>set_point for more than 10 counts, we cut
  static int cut_count=0;
  static bool cutter_state=false;
  static bool last_cutter_state=cutter_state; //make sure this happens only once;

  //check altitude, CUT_ALTITUDE has been converted to mm already(GNSS lib)
  //only changes the state
  if (curr_time - last_time >= 1500) {
    last_time = curr_time; 
    if (altitude>=CUT_ALTITUDE){
      cut_count++;
      Serial.print("Cutter count:");
      Serial.println(cut_count);
      if (cut_count>=5){
        cutterOn=true;
        cutter_state=true;
        return; //this will allow the heater and radio to OFF themselves
      }
    }
    else{
      cut_count=0;
    }
    digitalWrite(PIN_CUTTER, LOW);
    Serial.print("CutterOn state: ");
    Serial.println(cutterOn);
  }


  

  //during cutting time, no transmission shall occur, battery heater must be off
  //transmission state, rttyState change to RTTY_IDLE, HeaterOn=0(bool) and heaterstate=0(bool);
  if (cutter_state){
    if (cutter_last_time == 0) {  // Only set timer once when cutting starts
      cutter_last_time = curr_time;
    }
    digitalWrite(PIN_CUTTER,HIGH);

    if (curr_time-cutter_last_time>=CUT_DURATION){
      digitalWrite(PIN_CUTTER,LOW);
      cutterOn=false;
      cutter_state=false;
      cut_count=0;//reset count
      cutter_last_time=0; //reset timer for next use
      return; //exit after cutting to get altitude reading
    }
    return;  // Exit while cutting is active 
  }
}


void Cutter_setup(){
  pinMode(PIN_CUTTER,OUTPUT);
  digitalWrite(PIN_CUTTER,LOW);
  cutterOn=false;
  Serial.print("Cutter Setup");
}