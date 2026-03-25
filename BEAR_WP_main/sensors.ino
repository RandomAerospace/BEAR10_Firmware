void GNSS_setup() {

  if (myGNSS.setDynamicModel(DYN_MODEL_AIRBORNE2g) == false) {  // Set the dynamic model to PORTABLE
    Serial.println(F("*** Warning: setDynamicModel failed ***"));
  } else {
    Serial.println(F("Dynamic platform model changed successfully!"));
  }

  // Let's read the new dynamic model to see if it worked
  uint8_t newDynamicModel = myGNSS.getDynamicModel();
  if (newDynamicModel == DYN_MODEL_UNKNOWN) {
    Serial.println(F("*** Warning: getDynamicModel failed ***"));
  } else {
    Serial.print(F("The new dynamic model is: "));
    Serial.println(newDynamicModel);
  }
  myGNSS.setNavigationFrequency(4);  //Produce two solutions per second, GPS+GAL+BDS B1C+GLO
  myGNSS.setI2COutput(COM_TYPE_UBX);
  myGNSS.saveConfiguration();
}


/*
void thermocouple_setup() {
  Serial.println("MCP9600 HW test");
  mcp.begin(MCP9600_ADDR);
  Initialise the driver with I2C_ADDRESS and the default I2C bus. 
  if (!mcp.begin(MCP9600_ADDR)) {
    Serial.println("Sensor not found. Check wiring!");
  }

  Serial.println("Found MCP9600!");

  mcp.setADCresolution(MCP9600_ADCRESOLUTION_12);
  Serial.print("ADC resolution set to ");
  switch (mcp.getADCresolution()) {
    case MCP9600_ADCRESOLUTION_18: Serial.print("18"); break;
    case MCP9600_ADCRESOLUTION_16: Serial.print("16"); break;
    case MCP9600_ADCRESOLUTION_14: Serial.print("14"); break;
    case MCP9600_ADCRESOLUTION_12: Serial.print("12"); break;
  }
  Serial.println(" bits");

  mcp.setThermocoupleType(MCP9600_TYPE_K);
  Serial.print("Thermocouple type set to ");
  switch (mcp.getThermocoupleType()) {
    case MCP9600_TYPE_K: Serial.print("K"); break;
    case MCP9600_TYPE_J: Serial.print("J"); break;
    case MCP9600_TYPE_T: Serial.print("T"); break;
    case MCP9600_TYPE_N: Serial.print("N"); break;
    case MCP9600_TYPE_S: Serial.print("S"); break;
    case MCP9600_TYPE_E: Serial.print("E"); break;
    case MCP9600_TYPE_B: Serial.print("B"); break;
    case MCP9600_TYPE_R: Serial.print("R"); break;
  }
  Serial.println(" type");

  mcp.setFilterCoefficient(0);
  Serial.print("Filter coefficient value set to: ");
  Serial.println(mcp.getFilterCoefficient());
  mcp.enable(true);

  Serial.println(F("------------------------------"));
}

*/

void baro_setup() {
  unsigned long currentMillis = millis();
  static unsigned long previousMillis = 0;

  if (MS5611.begin() == true) {
    Serial.println("MS5611 found.");
  } else {

    if (currentMillis - previousMillis >= setup_interval) {
      previousMillis = currentMillis;
      Serial.println("MS5611 not found. halt.");
    }
  }
  Serial.println("Baro Setup done");
  //performance parameters
  /*
  There are 5 oversampling settings, each corresponding to a different amount of milliseconds
  The higher the oversampling, the more accurate the reading will be, however the longer it will take.
  OSR_ULTRA_HIGH -> 8.22 millis
  OSR_HIGH       -> 4.11 millis
  OSR_STANDARD   -> 2.1 millis
  OSR_LOW        -> 1.1 millis
  OSR_ULTRA_LOW  -> 0.5 millis   Default = backwards compatible
  */
  MS5611.setOversampling(OSR_ULTRA_HIGH);
  MS5611.read();
  Serial.print((uint8_t)MS5611.getOversampling());
  Serial.print("\t");

  if (currentMillis - previousMillis >= setup_interval) {
    previousMillis = currentMillis;  //to flush serial
  }
  Serial.println("\ndone...");
}

//setup onewire temp sensor
/*
void Bat_temp_setup() {
  onesense.begin();
  Serial.print("Onewire setup");
}
*/

void setup_temp() {
  Serial.println(F("[TEMP] Initializing MAX31856 (Type K) on VSPI..."));

  if (!maxthermo.begin()) {
    Serial.println(F("[TEMP ERROR] Check MAX31856 wiring!"));
    while (1) delay(10);
  }

  // Set to Type K (Very important!)
  maxthermo.setThermocoupleType(MAX31856_TCTYPE_K);
  
  Serial.print(F("[TEMP] Thermocouple type: "));
  switch (maxthermo.getThermocoupleType()) {
    case MAX31856_TCTYPE_K: Serial.println(F("K")); break;
    default: Serial.println(F("Unknown")); break;
  }
}

void task_temp() {
  // Read the temperature
  float temp = maxthermo.readThermocoupleTemperature();
  //float internal = maxthermo.readColdJunctionTemperature();

  // Update global variable for APRS/MAVLink
  external_temp = temp;

  // Check for faults (Open circuit, short to GND, etc.)
  uint8_t fault = maxthermo.readFault();
  if (fault) {
    if (fault & MAX31856_FAULT_CJRANGE) Serial.println(F("Cold Junction Range Fault"));
    if (fault & MAX31856_FAULT_TCRANGE) Serial.println(F("Thermocouple Range Fault"));
    if (fault & MAX31856_FAULT_CJHIGH)  Serial.println(F("Cold Junction High Fault"));
    if (fault & MAX31856_FAULT_CJLOW)   Serial.println(F("Cold Junction Low Fault"));
    if (fault & MAX31856_FAULT_TCHIGH)  Serial.println(F("Thermocouple High Fault"));
    if (fault & MAX31856_FAULT_TCLOW)   Serial.println(F("Thermocouple Low Fault"));
    if (fault & MAX31856_FAULT_OVUV)    Serial.println(F("Over/Under Voltage Fault"));
    if (fault & MAX31856_FAULT_OPEN)    Serial.println(F("Thermocouple Open Circuit"));
  }

  /* Serial.print(F("Hot: ")); Serial.print(temp);
  Serial.print(F(" C, Cold: ")); Serial.print(internal); Serial.println(F(" C"));
  */
}

void read_gnss() {


  // 1. Latitude/Longitude: Keep as Decimal Degrees for the DDM conversion
  glatitude = myGNSS.getLatitude() / 10000000.0f;
  glongitude = myGNSS.getLongitude() / 10000000.0f;

  // 2. Altitude: Store in Meters
  galtitude = myGNSS.getAltitude() / 1000.0f; 

  // 3. Speed: Store in km/h
  // Raw mm/s / 277.78 = km/h
  gspeed = myGNSS.getGroundSpeed() / 277.78f; 

  // 4. Heading: Degrees
  gheading = myGNSS.getHeading() / 100000.0f;

  // Debugging
  Serial.printf("Lat: %.6f, Lon: %.6f, Alt: %.1fm, Spd: %.1fkt\n", 
                glatitude, glongitude, galtitude, gspeed);
}



void read_baro() {
  MS5611.read();
  //since oversampling of 8.12millis in effect do take note of interval of sensor reads
  ambient_temp = MS5611.getTemperature();  //reads ambient temp and passes to global variable thermocouple_temp
  baro_press = MS5611.getPressure();            //reads baro press and passes to global variable baro_press
  paltitudeMSL=MS5611.getAltitude();
  Serial.print("Baro temperature");
  Serial.println(ambient_temp);
  Serial.print("Baro pressure:");
  Serial.println(baro_press);
}

/*
void read_thermocouple() {
  
  unsigned long currentMillis=millis();
  static unsigned long previousthermocoupleMillis=0;
  
  if (currentMillis - previousthermocoupleMillis >= thermocouple_wakeup) {
    //try to enable it for 10 times
    if (!mcp.enabled()){
      for (int i=0;i<10;i++){
        mcp.enable(true);
      } 
    }
    //attempt to read 10 times
    for (int i=0;i<10;i++){
      thermocouple_temp = mcp.readThermocouple();
    } 
    for (int y=0;y<10;y++){
      cold_junc=mcp.readAmbient();
    }
    //thermocouple_adc = mcp.readADC() * 2;
    Serial.print("\nHot Junction: ");
    Serial.println(thermocouple_temp);
    Serial.print("Cold Junction: ");
    Serial.println(cold_junc);
    //Serial.print("ADC: ");
    //Serial.print(thermocouple_adc);
    //Serial.println(" uV");
    currentMillis = previousthermocoupleMillis;
  } 
}
*/

/*
void read_bat_temp() {
  onesense.requestTemperatures();
  ambient_temp = onesense.getTempCByIndex(0);
}

void readIMU() {
  AccGyr.Get_X_Axes(accelerometer);
  AccGyr.Get_G_Axes(gyroscope);
  // Output data.
  Serial.print("ASM330LHH: | Acc[mg]: ");
  Serial.print(accelerometer[0]);
  Serial.print(" ");
  Serial.print(accelerometer[1]);
  Serial.print(" ");
  Serial.print(accelerometer[2]);
  Serial.print(" | Gyr[mdps]: ");
  Serial.print(gyroscope[0]);
  Serial.print(" ");
  Serial.print(gyroscope[1]);
  Serial.print(" ");
  Serial.print(gyroscope[2]);
  Serial.println(" |");
  Ax = accelerometer[0];
  Ay = accelerometer[1];
  Az = accelerometer[2];
  Gx = gyroscope[0];
  Gy = gyroscope[1];
  Gz = gyroscope[2];
}
*/
