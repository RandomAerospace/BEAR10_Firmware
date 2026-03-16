void read_battery() {

  //takes 15 samples and returns the average
  // Take multiple readings
  float sum = 0.0f;
  for (int i = 0; i < NUM_SAMPLES; i++) {
      sum += analogRead(batt_pin);
      }
    
  // Calculate average
  float average = sum / NUM_SAMPLES;
    
  // Convert to actual voltage
  battery_voltage = (average / ADC_MAX) * ADC_REF_VOLTAGE * VOLTAGE_DIVIDER;
  Serial.print(battery_voltage);
}
