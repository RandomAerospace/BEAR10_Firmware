
#ifdef HW_ESP32S3
inline uint32_t calc_clkdiv(uint32_t freq) {
  return ((((uint32_t)APB_CLK_FREQ) / freq) >> 4);
}


void sine_setup(uint32_t freq) {
  // Dummy call to ledc library required to enable LEDC module
  // Enabling requires the periph_module_enable() function which
  // is hidden from us.
  ledcSetup(LEDC_CHANNEL_0, freq, LEDC_TIMER_12_BIT);
  
  ledc->conf.apb_clk_sel = 1;
  ledc->timer_group[LEDC_LOW_SPEED_MODE].timer[LEDC_TIMER_0].conf.clock_divider = calc_clkdiv(freq);
  ledc->timer_group[LEDC_LOW_SPEED_MODE].timer[LEDC_TIMER_0].conf.tick_sel = 0;
  ledc->timer_group[LEDC_LOW_SPEED_MODE].timer[LEDC_TIMER_0].conf.duty_resolution = LEDC_TIMER_12_BIT;
  ledc->timer_group[LEDC_LOW_SPEED_MODE].timer[LEDC_TIMER_0].conf.low_speed_update = 1;
  ledc->timer_group[LEDC_LOW_SPEED_MODE].timer[LEDC_TIMER_0].conf.rst = 1;
  ledc->timer_group[LEDC_LOW_SPEED_MODE].timer[LEDC_TIMER_0].conf.rst = 0;
  ledc->timer_group[LEDC_LOW_SPEED_MODE].timer[LEDC_TIMER_0].conf.low_speed_update = 1;

  ledc->channel_group[LEDC_LOW_SPEED_MODE].channel[LEDC_CHANNEL_0].hpoint.hpoint = 0;
  ledc->channel_group[LEDC_LOW_SPEED_MODE].channel[LEDC_CHANNEL_0].duty.duty = (2047 << 4);
  ledc->channel_group[LEDC_LOW_SPEED_MODE].channel[LEDC_CHANNEL_0].conf1.duty_inc = 1;
  ledc->channel_group[LEDC_LOW_SPEED_MODE].channel[LEDC_CHANNEL_0].conf1.duty_num = 0;
  ledc->channel_group[LEDC_LOW_SPEED_MODE].channel[LEDC_CHANNEL_0].conf1.duty_cycle = 0;
  ledc->channel_group[LEDC_LOW_SPEED_MODE].channel[LEDC_CHANNEL_0].conf1.duty_scale = 0;
  ledc->channel_group[LEDC_LOW_SPEED_MODE].channel[LEDC_CHANNEL_0].conf0.low_speed_update = 1;
  ledc->channel_group[LEDC_LOW_SPEED_MODE].channel[LEDC_CHANNEL_0].conf0.sig_out_en = true;
  ledc->channel_group[LEDC_LOW_SPEED_MODE].channel[LEDC_CHANNEL_0].conf1.duty_start = true;
  ledc->channel_group[LEDC_LOW_SPEED_MODE].channel[LEDC_CHANNEL_0].conf0.low_speed_update = 1;

  ledc->channel_group[LEDC_LOW_SPEED_MODE].channel[LEDC_CHANNEL_0].conf0.timer_sel = LEDC_TIMER_0;
  ledc->channel_group[LEDC_LOW_SPEED_MODE].channel[LEDC_CHANNEL_0].conf0.low_speed_update = 1;

  ledc->int_ena.val = (ledc->int_ena.val & (~(BIT(LEDC_DUTY_CHNG_END_LSCH0_INT_ENA_S + LEDC_CHANNEL_0))));

  gpio_hal_iomux_func_sel(GPIO_PIN_MUX_REG[PIN_TX_AUD], PIN_FUNC_GPIO);
  gpio_set_direction(PIN_TX_AUD, GPIO_MODE_OUTPUT);
  esp_rom_gpio_connect_out_signal(PIN_TX_AUD, ledc_periph_signal[LEDC_LOW_SPEED_MODE].sig_out0_idx + LEDC_CHANNEL_0, 0, 0);
}

inline void set_dac_freq(uint32_t freq) {
  ledc->timer_group[LEDC_LOW_SPEED_MODE].timer[LEDC_TIMER_0].conf.clock_divider = calc_clkdiv(freq);
  ledc->timer_group[LEDC_LOW_SPEED_MODE].timer[LEDC_TIMER_0].conf.low_speed_update = 1;
}

inline void dac_cw_generator_disable() {
  ledc->timer_group[LEDC_LOW_SPEED_MODE].timer[LEDC_TIMER_0].conf.pause = 1;
  ledc->timer_group[LEDC_LOW_SPEED_MODE].timer[LEDC_TIMER_0].conf.low_speed_update = 1;
}

inline void dac_cw_generator_enable() {
  ledc->timer_group[LEDC_LOW_SPEED_MODE].timer[LEDC_TIMER_0].conf.pause = 0;
  ledc->timer_group[LEDC_LOW_SPEED_MODE].timer[LEDC_TIMER_0].conf.low_speed_update = 1;
}
#else
void sine_setup(uint32_t freq) {
  dac_cw_config_t dac_cw_config = {
    .en_ch = DAC_CHANNEL_1,
    .scale = DAC_CW_SCALE_1,
    .phase = DAC_CW_PHASE_0,
    .freq = freq << 3,
    .offset = 1
  };
  dac_cw_generator_config(&dac_cw_config);
  REG_SET_FIELD(RTC_CNTL_CLK_CONF_REG, RTC_CNTL_CK8M_DIV_SEL, 7);
  dac_cw_generator_enable();
  dac_output_enable(DAC_CHANNEL_1);
}

inline void set_dac_freq(uint32_t freq) {
  dac_ll_cw_set_freq(freq << 3);
}
#endif



inline void sleep_dra818() {
  digitalWrite(PIN_RAD_PW, LOW);
}

inline void tune_dra818(String freq) {
  for (int i = 0; i < 3; i++) {
    radioCtrl.print("AT+DMOSETGROUP=0," + freq + ",144.3900,0000,2,0000\r\n");
    //radioCtrl.print("AT+DMOSETGROUP=0,144.3900,144.3900,0000,2,0000\r\n");
    //radioCtrl.print("AT+DMOSETGROUP=0,145.5000,144.3900,0000,2,0000\r\n");
    char tc = 0;
    char atp[3] = {0};
    bool atdetected = false;
    unsigned long atp_start = millis();
    do {
      while (!radioCtrl.available() && ((millis() - atp_start) < 5000));
      tc = radioCtrl.read();
      Serial.write(tc);
      atp[0] = atp[1];
      atp[1] = atp[2];
      atp[2] = tc;
      if (atp[0] == '+' && atp[1] == 'D' && atp[2] == 'M') atdetected = true;
    } while ((tc != '\n' || !atdetected) && ((millis() - atp_start) < 5000));
    if (atdetected) break;
  }
  Serial.println();
}

inline void wake_dra818() {
  digitalWrite(PIN_RAD_PW, HIGH);
  delay(600);
  for (int i = 0; i < 3; i++) {
    radioCtrl.print("AT+DMOCONNECT\r\n");
    char tc = 0;
    char atp[3] = {0};
    bool atdetected = false;
    unsigned long atp_start = millis();
    do {
      while (!radioCtrl.available() && ((millis() - atp_start) < 5000));
      tc = radioCtrl.read();
      Serial.write(tc);
      atp[0] = atp[1];
      atp[1] = atp[2];
      atp[2] = tc;
      if (atp[0] == '+' && atp[1] == 'D' && atp[2] == 'M') atdetected = true;
    } while ((tc != '\n' || !atdetected) && ((millis() - atp_start) < 5000));
    if (atdetected) break;
  }
  Serial.println();

  for (int i = 0; i < 3; i++) {
    radioCtrl.print("AT+DMOSETVOLUME=8\r\n");
    char tc = 0;
    char atp[3] = {0};
    bool atdetected = false;
    unsigned long atp_start = millis();
    do {
      while (!radioCtrl.available() && ((millis() - atp_start) < 5000));
      tc = radioCtrl.read();
      Serial.write(tc);
      atp[0] = atp[1];
      atp[1] = atp[2];
      atp[2] = tc;
      if (atp[0] == '+' && atp[1] == 'D' && atp[2] == 'M') atdetected = true;
    } while ((tc != '\n' || !atdetected) && ((millis() - atp_start) < 5000));
    if (atdetected) break;
  }
  Serial.println();
}

void setup_dra818() {
  radioCtrl.begin(9600);
  //0 TO ENTER TX STATE
  //1 TO ENTER RX STATE
  pinMode(PIN_RAD_PTT, OUTPUT);
  digitalWrite(PIN_RAD_PTT, HIGH);

  pinMode(PIN_RAD_PW, OUTPUT);
  wake_dra818();
  tune_dra818("144.3900");

  //Turn on module
  pinMode(PIN_RAD_PW, OUTPUT);
  digitalWrite(PIN_RAD_PW, HIGH);

  pinMode(PIN_TX_AUD, OUTPUT);
  pinMode(PIN_RX_AUD, INPUT);

  pinMode(PIN_RAD_SQL, INPUT);

  sine_setup(400);

  // DRA818 Power: 1W
  
  //pinMode(PIN_RAD_EN, OUTPUT);
  //digitalWrite(PIN_RAD_EN, HIGH);

  // DRA818 Power: 0.5W
  pinMode(PIN_RAD_EN, OUTPUT);
  digitalWrite(PIN_RAD_EN, LOW);
}


//reuse this code for state machine
void RTTY_TX() {
  unsigned long currentTime = millis();
  static bool transmissionStarted = false;

  //TX safeties//
  if (cutterOn == true) {
    rttyState = RTTY_IDLE;  //returns RTTY state machine to idle to allow sensors to update between cuts or heaters, if not sensors will not be read
    transmissionStarted = false;
    return;  //disables any transmission
  }
  if (heaterOn == true) {
    rttyState = RTTY_IDLE;
    //returns RTTY state machine to idle to allow sensors to update between cuts or heaters,  if not sensors will not be read
    transmissionStarted = false;
    return;  //disables any transmission
  }


  //Timed events start here
  switch (rttyState) {
    case RTTY_IDLE:
      transmissionStarted = false;  // Reset flag when back in IDLE OR START
      if (currentTime - rttyStateStartTime >= RTTY_IDLE_TIME) {
        rttyState = RTTY_START;
        rttyStateStartTime = currentTime;
       
      }
      break;

    case RTTY_START:

       
      Serial.println(F("[RTTY] Starting transmission process..."));
      if (currentTime - rttyStateStartTime >= RTTY_START_TIME) {
        rttyState = RTTY_TRANSMITTING;
        rttyStateStartTime = currentTime;
        Serial.println(F("[RTTY] Beginning transmission..."));
      }
      break;

    case RTTY_TRANSMITTING:

      if (!transmissionStarted) {  // transmissionStarted=True
        int tx_mode = 0;
        //int tx_mode=frame_counter%2; //switch between 2 modes, aprs and rtty
        //send report
        if (tx_mode == 0) {
          bool use_gps = false;
          if (fixType == 0) use_gps = false;        // No Fix
          else if (fixType == 1) use_gps = false;   // Dead reckoning
          else if (fixType == 2) use_gps = true;    // 2D
          else if (fixType == 3) use_gps = true;    // 3D
          else if (fixType == 4) use_gps = true;    // GNSS + Dead reckoning
          else if (fixType == 5) use_gps = false;   // Time only

          send_report(use_gps);
          transmissionStarted = true;
        }
      }
      //this will bring the the transmitter out of the transmission loop
      if (currentTime - rttyStateStartTime >= RTTY_TRANSMIT_TIME) {
        rttyState = RTTY_COOLDOWN;
        rttyStateStartTime = currentTime;

        frame_counter++; // Increment the counter
        task_cam_switch(); // <--- TRIGGER THE CAMERA SWITCH HERE

        Serial.print(F("[SYSTEM] Transmission finished. Switched to Camera "));
        Serial.println((frame_counter % 2) + 1);
        Serial.println(F("[RTTY] Transmission complete, entering cooldown..."));
      }
      break;

    case RTTY_COOLDOWN:
      //put stuff here
      if (currentTime - rttyStateStartTime >= RTTY_COOLDOWN_TIME) {
        rttyState = RTTY_IDLE;
        rttyStateStartTime = currentTime;
        Serial.println(F("[RTTY] Cooldown complete, returning to idle."));
      }
      break;
  }
}


