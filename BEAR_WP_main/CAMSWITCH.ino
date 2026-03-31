void setup_cam_switch() {
  pinMode(PIN_CAM_SWITCH, OUTPUT);
  // 1. Configure Hardware PWM Output (LEDC)
  ledcSetup(ledcChannel2, ledcFreq, ledcRes);
  ledcAttachPin(PIN_CAM_SWITCH, ledcChannel2);
  
  Serial.println(F("[CAM] Camera Switcher Setup Complete (50Hz Output)"));
}


void task_cam_switch() {
  uint32_t target_pulse_us = 1500; 

  // Toggle logic: Switch based on whether frame_counter is even or odd
  // frame_counter % 2 == 0 -> Camera 1 (1.1ms)
  // frame_counter % 2 == 1 -> Camera 2 (1.9ms)
  
  if (frame_counter % 2 == 0) {
    target_pulse_us = 1100; // C1 Camera Output (0.9-1.4ms range)
  } else {
    target_pulse_us = 1900; // C2 Camera Output (1.6-2.1ms range)
  }

  // Convert Microseconds to LEDC Duty Cycle (13-bit) for 50Hz
  // (target_pulse_us / 20000.0) * 8191
  uint32_t duty = (target_pulse_us * 8191) / 20000;
  
  ledcWrite(ledcChannel2, duty);
}