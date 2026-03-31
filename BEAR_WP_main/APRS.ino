//this funciton builds the APRS packet

void send_report(bool use_gps) {
  static uint8_t page = 0;
  uint8_t packet[250];
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

  sz += sprintf((char*)&(packet[sz]), "%02d%02d%02d", tday % 32, thour % 24, tminute % 60);

  if (use_gps) {
    packet[sz++] = 'z';

    // LATITUDE (DD to DDM)
    tempf = glatitude;
    tempb = tempf >= 0;
    if (!tempb) tempf *= -1.0;
    tempu8 = tempf;
    tempf -= tempu8;
    tempf *= 60;
    //tempu16 %= 91;
    sz += sprintf((char*)&(packet[sz]), "%02d%05.2f", tempu8, tempf);
    packet[sz++] = tempb ? 'N' : 'S';

    packet[sz++] = '/';


    // LONGITUDE (DD to DDM)
    tempf = glongitude;
    tempb = tempf >= 0;
    if (!tempb) tempf *= -1.0;
    tempu16 = tempf;
    tempf -= tempu16;
    tempf *= 60;
    tempu16 %= 181;
    sz += sprintf((char*)&(packet[sz]), "%03d%05.2f", tempu16, tempf);
    packet[sz++] = tempb ? 'E' : 'W';

   
    // Balloon symbol
    packet[sz++] = 'O';

    sz += sprintf((char*)&(packet[sz]), "%03d", gheading % 360);
    packet[sz++] = '/';

    tempf = gspeed;
    tempf /= 1.852;
    tempu16 = tempf;
    sz += sprintf((char*)&(packet[sz]), "%03d", tempu16 % 1000);

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

  tempf = thealti;
  tempf *= 3.280839895;
  if (tempf <= 0.0) {
    tempu32 = 0;
  } else {
    tempu32 = tempf;
  }
  sz += sprintf((char*)&(packet[sz]), "%06u", tempu32 % 1000000U);

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
                    "V:%05.2f I:%05.3f C:%c ", 
                    BV, Current, cutterOn ? 'C' : 'X');
      break;
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