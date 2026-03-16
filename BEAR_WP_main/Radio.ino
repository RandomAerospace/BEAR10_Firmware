// Function prototypes
String updateRTTY() {
  Payload info;
  info.frame_counter = frame_counter,
  info.hour = hour;
  info.minute = minute;
  info.second = second;
  info.latitude_processed = latitude / 10000000.0f;  //GNSS units is degrees*10e-07
  info.longitude_processed = longitude / 10000000.0f;
  info.altitude_processed = altitude / 1000.0f;  //mm to m
  info.speed_processed = speed * 0.0036f;        //mm/s to km/h
  info.heading_processed = heading / 100000.0f;  //GNSS units is degrees*10e-05

  dtostrf(thermocouple_temp, 2, 1, info.thermocouple_temp_str);  // (float_value, min_width, decimal_places, buffer)
  dtostrf(ambient_temp, 2, 1, info.ambient_temp_str);            // (float_value, min_width, decimal_places, buffer)

  info.battery_voltage = battery_voltage;
  info.baro_press = baro_press;
  info.payload_info = payload_info;
  info.heaterOn = heaterOn;



  //create RTTY String
  String rtty_payload = makeString(info);
  Serial.println(rtty_payload);
  return rtty_payload;
}

// initialize SX1262 with default settings
void radio_setup() {

  unsigned long currentMillis = millis();
  pinMode(RADIO_TXEN, OUTPUT);
  digitalWrite(RADIO_TXEN, LOW);


  Serial.print(F("[SX1268] Initializing ... "));
  int state = radio.beginFSK(RF_FREQUENCY, 1.2, 6);
  if (state == RADIOLIB_ERR_NONE) {
    Serial.println(F("success!"));
  } else {
    Serial.print(F("failed, code "));
    Serial.println(state);
  }

  // when using one of the non-LoRa modules for RTTY
  // (RF69, CC1101, Si4432 etc.), use the basic begin() method
  // int state = radio.begin();



  // Set output power
  Serial.print(F("[SX1268] Setting output power ... "));
  state = radio.setOutputPower(RF_OUTPUT_POWER);
  if (state == RADIOLIB_ERR_NONE) {
    Serial.println(F("success!"));
  } else {
    Serial.print(F("failed, code "));
    Serial.println(state);
  }

  // Set current limit
  Serial.print(F("[SX1268] Setting current limit ... "));
  state = radio.setCurrentLimit(100.0);
  if (state == RADIOLIB_ERR_NONE) {
    Serial.println(F("success!"));
  } else {
    Serial.print(F("failed, code "));
    Serial.println(state);
  }

  //To use XTAL, either set this value to 0, or set SX126x::XTAL to true
  //This is because the E22-400M30S has a XTAL TCXO
  //The documentation is fucking ass, here is a github source for good specs:
  //https://github.com/jgromes/RadioLib/issues/12
  state = radio.setTCXO(TCXO_VOLTAGE);
  if (state == RADIOLIB_ERR_NONE) {
    Serial.println(F("success!"));
  } else {
    Serial.print(F("failed, code "));
    Serial.println(state);
  }




  // initialize RTTY client
  // NOTE: RTTY frequency shift will be rounded
  //       to the nearest multiple of frequency step size.
  //       The exact value depends on the module:
  //         SX127x/RFM9x - 61 Hz
  //         RF69 - 61 Hz
  //         CC1101 - 397 Hz
  //         SX126x - 1 Hz
  //         nRF24 - 1000000 Hz
  //         Si443x/RFM2x - 156 Hz
  //         SX128x - 198 Hz
  Serial.print(F("[RTTY] Initializing ... "));
  // low ("space") frequency:     434.0 MHz
  // frequency shift:             600 Hz
  // baud rate:                   300 baud
  // encoding:                    ASCII (8-bit)"RADIOLIB_ASCII_EXTENDED" or ASCII(7-bit) "Leave blank"
  // stop bits:                   2
  state = rtty.begin(RF_FREQUENCY, RTTY_FREQUENCY_SHIFT, RTTY_BAUD_RATE, RADIOLIB_ASCII, RTTY_STOP_BITS);  //follow UKHAS specs
  if (state == RADIOLIB_ERR_NONE) {
    Serial.println(F("success!"));
  } else {
    Serial.print(F("failed, code "));
    Serial.println(state);
  }

  /*
  // initialize AX.25 client
  Serial.print(F("[AX.25] Initializing ... "));
  // source station callsign:     "N7LEM"
  // source station SSID:         0
  // preamble length:             8 bytes
  state = ax25.begin("9V1WP" );
  if(state == RADIOLIB_ERR_NONE) {
    Serial.println(F("success!"));
  } else {
    Serial.print(F("failed, code "));
    Serial.println(state);
  }

  // initialize APRS client
  Serial.print(F("[APRS] Initializing ... "));
  // symbol:                      'o' (balloon)
  state = aprs.begin('O');
  if(state == RADIOLIB_ERR_NONE) {
    Serial.println(F("success!"));
  } else {
    Serial.print(F("failed, code "));
    Serial.println(state);
  }
  */
}


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

  switch (rttyState) {
    case RTTY_IDLE:
      transmissionStarted = false;  // Reset flag when back in IDLE OR START
      if (currentTime - rttyStateStartTime >= RTTY_IDLE_TIME) {
        rttyState = RTTY_START;
        rttyStateStartTime = currentTime;
        Serial.println(F("[RTTY] Starting transmission process..."));
      }
      break;

    case RTTY_START:
      transmissionStarted = false;  // Reset flag when back in IDLE or START
      //Oscillator to be used in standby mode. Can be set to RADIOLIB_SX126X_STANDBY_RC (13 MHz RC oscillator)
      //or RADIOLIB_SX126X_STANDBY_XOSC (32 MHz external crystal oscillator).
      //this replaces the idle setting found in the RTTY samples.
      radio.standby(RADIOLIB_SX126X_STANDBY_XOSC, true);
      digitalWrite(RADIO_TXEN, HIGH);
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
        String message = updateRTTY();
        if (tx_mode == 0) {
          for (uint8_t i = 0; i < RTTY_PREAMBLE_LENGTH; i++) {
            //this will bring the the transmitter out of the transmission loop
            if (currentTime - rttyStateStartTime >= RTTY_TRANSMIT_TIME) {
              rttyState = RTTY_COOLDOWN;
              rttyStateStartTime = currentTime;
              Serial.println(F("[RTTY] Transmission complete, entering cooldown..."));
            }
            rtty.print(RTTY_PREAMBLE);
          }
          rtty.println(message);
          rtty.println(message);
          //this will bring the the transmitter out of the transmission loop
          if (currentTime - rttyStateStartTime >= RTTY_TRANSMIT_TIME) {
            rttyState = RTTY_COOLDOWN;
            rttyStateStartTime = currentTime;
            Serial.println(F("[RTTY] Transmission complete, entering cooldown..."));
          }
        } else {
          //APRS_message(message);
        }
      }
      rttyState = RTTY_COOLDOWN;
      frame_counter++;
      rttyStateStartTime = currentTime;
      transmissionStarted = true;
      Serial.println(F("[RTTY] Transmission complete, entering cooldown..."));

      break;

    case RTTY_COOLDOWN:
      digitalWrite(RADIO_TXEN, LOW);
      radio.sleep();
      if (currentTime - rttyStateStartTime >= RTTY_COOLDOWN_TIME) {
        rttyState = RTTY_IDLE;
        rttyStateStartTime = currentTime;
        Serial.println(F("[RTTY] Cooldown complete, returning to idle."));
      }
      break;
  }
}





uint16_t _crc_xmodem_update(uint16_t crc, uint8_t data) {
  crc = crc ^ (data << 8);
  for (int i = 0; i < 8; i++) {
    if (crc & 0x8000)
      crc = (crc << 1) ^ 0x1021;
    else
      crc <<= 1;
  }
  return crc;
}

uint16_t gps_CRC16_checksum(const char* string) {
  size_t i;
  uint16_t crc;
  uint8_t c;

  crc = 0xFFFF;

  // Calculate checksum ignoring the first two $s
  for (i = 2; i < strlen(string); i++) {
    c = string[i];
    crc = _crc_xmodem_update(crc, c);
  }

  return crc;
}

String makeString(Payload info) {
  char s[100];  // This is the payload Make sure this is large enough for your data
  //String payload_info="012345789ABR";
  snprintf(s, sizeof(s),
           "$$9V1WP,%u,%u:%u:%u,%.4f,%.4f,%.4f,%.2f,%.2f,%.2f,%s,%s,%u,%s,%d",
           info.frame_counter,
           info.hour,
           info.minute,
           info.second,
           info.latitude_processed,
           info.longitude_processed,
           info.altitude_processed,
           info.speed_processed,
           info.heading_processed,
           info.battery_voltage,
           info.ambient_temp_str,
           info.thermocouple_temp_str,
           info.baro_press,
           info.payload_info,
           info.heaterOn);



  char checksumStr[10];
  snprintf(checksumStr, sizeof(checksumStr), "*%04X\n", gps_CRC16_checksum(s));  //%04X refers to HEX data format
    // Check if there's enough space for the checksum
  if (strlen(s) > sizeof(s) - strlen(checksumStr) - 1) {
    // Don't overflow the buffer. You should have made it bigger.
    return String("Error: Buffer too small");
  }

  // Append checksum to the main string
  memcpy(s + strlen(s), checksumStr, strlen(checksumStr) + 1);

  return String(s);
}


void APRS_message(String message) {
  // send a location without message or timestamp
  char destination[] = "N0CALL";
  char latitude_str[10];
  char longitude_str[10];
  char message_aprs[200];
  float latitude_float = latitude / 10000000.0f;
  float longitude_float = longitude / 10000000.0f;
  sprintf(latitude_str, "%.4f", latitude_float);    //GNSS units is degrees*10e-07
  sprintf(longitude_str, "%.4f", longitude_float);  //GNSS units is degrees*10e-07
  sprintf(message_aprs, "%s", message);
  int state = aprs.sendPosition(destination, 0, latitude_str, longitude_str, message_aprs);
  if (state != RADIOLIB_ERR_NONE) {
    Serial.print(F("[APRS] Failed to send location, code "));
    Serial.println(state);
  }
}
