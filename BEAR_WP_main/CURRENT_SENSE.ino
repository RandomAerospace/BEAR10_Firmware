  
void setup_currentsense(){
  Serial.println(F("[INA228] Attempting to find sensor on I2C2 (32/33)..."));

  // 1. Initialize using the custom address AND the Bus pointer in ONE call
  // This is the correct way to avoid Null Pointer panics
  if (!ina228.begin(INA228_ADDR, &I2C2_Bus)) {
    Serial.println(F("[INA228 ERROR] Hardware not detected. Check 3.3V and GND!"));
    return; // Stop here if sensor is missing
  }

  // 2. Configure Shunt (Only AFTER a successful begin)
  // Ensure 0.0002 is correct for your specific hardware!
  ina228.setShunt(0.0002, 10.0); 

  // 3. Optional: Set conversion time to be more power-efficient/stable
  // INA228_VBUS_CONV_TIME_1052US is a good balance for balloons
  
  Serial.println(F("[INA228] Setup Successful. Monitoring active."));

}


void read_currentsense(){
  // Grab Power Data
  BV = ina228.getBusVoltage_V();
  Current = (ina228.getCurrent_mA() / 1000.0);
  BatTemp=ina228.readDieTemp();
  // Print to Serial Monitor
  Serial.print(F("[BATT] Voltage: "));
  Serial.print(BV, 2);           // 2 decimal places (e.g., 12.45V)
  Serial.print(F("V | Current: "));
  Serial.print(Current, 3);        // 3 decimal places for mA precision (e.g., 0.850A)
  Serial.print(F("A | Die Temp: "));
  Serial.print(BatTemp, 1);
  Serial.println(F("°C"));
  

}