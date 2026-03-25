//this funciton builds the APRS packet

void send_report(bool use_gps) {
  static uint8_t page = 0;
  uint8_t packet[200];
  uint8_t sz = 0;

  //temporary vlaues
  bool tempb;
  uint8_t tempu8;
  uint16_t tempu16;
  uint32_t tempu32;
  //  int8_t tempi8;
  //  int16_t tempi16;
  //  int32_t tempi32;
  float tempf;
  //  String temps;

  static float last_alti = 0;
  float thealti = 0;

  packet[sz++] = use_gps ? '@' : '>';

  sz += sprintf((char*)&(packet[sz]), "%02d%02d%02d", tday % 32, thour % 60, tminute % 60);

  if (use_gps) {
    packet[sz++] = 'z';

    // LATITUDE (DD to DDM)
    tempf = glatitude;
    tempb = tempf >= 0;
    if (!tempb) tempf *= -1.0;
    tempu8 = (uint8_t)tempf;
    tempf = (tempf - tempu8) * 60.0;
    sz += sprintf((char*)&(packet[sz]), "%02u%05.2f", tempu8 % 91, tempf);
    packet[sz++] = tempb ? 'N' : 'S';

    packet[sz++] = '/';


    // LONGITUDE (DD to DDM)
    tempf = glongitude;
    tempb = tempf >= 0;
    if (!tempb) tempf *= -1.0;
    tempu16 = (uint16_t)tempf;
    tempf = (tempf - tempu16) * 60.0;
    sz += sprintf((char*)&(packet[sz]), "%03u%05.2f", tempu16 % 181, tempf);
    packet[sz++] = tempb ? 'E' : 'W';

    packet[sz++] = 'O'; // Balloon Symbol

    sz += sprintf((char*)&(packet[sz]), "%03d", gheading % 360);

    packet[sz++] = '/';

    // COURSE and SPEED (Standard APRS expects Knots here)
    // Convert km/h to Knots just for this field: km/h / 1.852
    uint16_t speed_knots = (uint16_t)(gspeed / 1.852f);
    sz += sprintf((char*)&(packet[sz]), "%03u/%03u", (uint16_t)gheading % 360, speed_knots % 1000);

    thealti = galtitude; 
    } else {
    thealti = paltitudeMSL;
    }
    //
    //    tempf = myGNSS.getAltitudeMSL();
    //    if (tempf < 0) tempf = 0.0;
    //    tempf *= 3.280839895;
    //    tempf /= 1000;
    //    tempu32 = tempf;
    //    tempu32 %= 1000000L;
    //    temps = String(tempu32);
    //    for (tempu8 = 6; tempu8 > 0; tempu8--) {
    //      if (tempu8 > temps.length()) {
    //        packet[sz++] = '0';
    //      } else {
    //        packet[sz++] = temps.c_str()[temps.length() - tempu8];
    //      }
    //    }
 
  // ALTITUDE FIELD (Standard APRS expects Feet here)
  packet[sz++] = '/';
  packet[sz++] = 'A';
  packet[sz++] = '=';
  uint32_t alt_feet = (uint32_t)(thealti * 3.28084f);
  sz += sprintf((char*)&(packet[sz]), "%06u", alt_feet % 1000000);

  // METRIC COMMENT SECTION
  // This adds " 12345m 123km/h" to the end of your packet for human reading
  sz += sprintf((char*)&(packet[sz]), " %um %ukph ", (uint32_t)thealti, (uint32_t)gspeed);

  packet[sz++] = ' ';

  sz += sprintf((char*)&(packet[sz]), "%06dTx", msg_id++);

  packet[sz++] = 'C' + page;
  packet[sz++] = ' ';
  //number of page
  switch (page) {
    default:
      case 0:
      // PRIMARY ENVIRONMENTAL PAGE
      // T: Ambient (Internal Baro)
      // E: External (Type K)
      // P: Pressure in hPa
      sz += sprintf((char*)&(packet[sz]), 
                    "T:%+05.1f E:%+05.1f P:%04u ", 
                    ambient_temp, external_temp, baro_press);
      break;

    case 1:
      // POWER & SYSTEM PAGE
      // B: Battery Temp (from UART bridge)
      // V: Battery Voltage
      // H: Heater Status
      sz += sprintf((char*)&(packet[sz]), 
                    "B:%+05.1f V:%04.2fV H:%c ", 
                    bat_temp, BV, heaterOn ? 'H' : 'X');
  }
  if (++page == 2) {
    page = 0;
  }

  packet[sz++] = ' ';
  for (tempu8 = 0; tempu8 < comment_suffix.length(); tempu8++) {
    packet[sz++] = comment_suffix.c_str()[tempu8];
  }

  // And send the update
  //wake_dra818();
  // Adjust path depending on altitude)
  if (thealti > 2000) {
    APRS_setPath1("WIDE2", 1);
    APRS_sendPkt(packet, sz, 3);
  } else {
    APRS_setPath1("WIDE1", 1);
    APRS_sendPkt(packet, sz, 4);
  }
  //sleep_dra818();

  // Check if we have been descending for a long time
  // Turn on fast SSTV if we are falling
  /*
  if (last_alti > thealti) {
    descent_count++;
    if (descent_count >= 6) {
      DBGPORT.println("Descent detected, started fast SSTV");
      fast_sstv = true;
    }
  } else {
    descent_count = 0;
  }
  last_alti = thealti;
  */
}





void setup_aprs() {
  APRS_init();

  APRS_setCallsign(callsign.c_str(), callsign_ssid);

  APRS_useAlternateSymbolTable(false);
  APRS_setSymbol('O');

  APRS_printSettings(Serial);

  //wake_dra818();
  APRS_sendMsg(boot_message.c_str(), boot_message.length());
  //sleep_dra818();
}