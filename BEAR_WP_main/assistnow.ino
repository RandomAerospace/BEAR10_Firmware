//initialise GNSS
void Assistnow_setup() {
   Serial.println(F("AssistNow"));
  
  // Connect to GNSS module
  if (myGNSS.begin() == false) {
    Serial.println(F("u-blox GPS not detected at default I2C address. Please check wiring. Freezing."));
    while (1);
  }
  Serial.println(F("u-blox module connected"));

  myGNSS.setI2COutput(COM_TYPE_UBX);  //Turn off NMEA noise
  
  // WiFi connection with timeout
  const unsigned long WIFI_TIMEOUT = 15000; // 15 seconds timeout
  unsigned long startAttemptTime = millis();
  
  Serial.print(F("Connecting to local WiFi"));
  
  WiFi.begin(ssid, password);
  
  // Try to connect to WiFi with timeout
  while (WiFi.status() != WL_CONNECTED && (millis() - startAttemptTime) < WIFI_TIMEOUT) {
       Serial.print(F("."));
  }
  
  // Check if we connected successfully
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println();
    Serial.println(F("WiFi connected!"));
    
    //=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
    // Set the RTC using network time
    configTime(0, 0, ntpServer,ntpServer_01,ntpServer_02); 
    // GMT offset are 0, daylight savings is 0(we need UTC),servers

    struct tm timeinfo;
    if (!getLocalTime(&timeinfo)) {
      Serial.println("Failed to obtain time");
    } else {
      Serial.println(&timeinfo, "Time is: %A, %B %d %Y %H:%M:%S");
    }

    // Rest of your AssistNow online data fetching code
    const int URL_BUFFER_SIZE = 256;
    char theURL[URL_BUFFER_SIZE];
    int payloadSize = 0;
    String payload;
  

#ifdef USE_SERVER_ASSISTANCE
    snprintf(theURL, URL_BUFFER_SIZE, "%s/%s%s%s%s%s%s%s%s%s%s",
             assistNowServer,
             getQuery,
             tokenPrefix,
             myAssistNowToken,
             tokenSuffix,
             getGNSS,
             getDataType,
             useLatitude,
             useLongitude,
             useAlt,
             usePosAcc);
#else
    snprintf(theURL, URL_BUFFER_SIZE, "%s/%s%s%s%s%s%s",
             assistNowServer,
             getQuery,
             tokenPrefix,
             myAssistNowToken,
             tokenSuffix,
             getGNSS,
             getDataType);
#endif

    Serial.print(F("HTTP URL is: "));
    Serial.println(theURL);

    HTTPClient http;
    http.begin(theURL);
    int httpCode = http.GET();

    if (httpCode > 0) {
      Serial.printf("[HTTP] GET... code: %d\r\n", httpCode);

      if (httpCode == HTTP_CODE_OK) {
        payloadSize = http.getSize();
        Serial.printf("Server returned %d bytes\r\n", payloadSize);
        payload = http.getString();
      }
    } else {
      Serial.printf("[HTTP] GET... failed, error: %s\r\n", http.errorToString(httpCode).c_str());
    }

    http.end();

    // Push the AssistNow data to the module
    myGNSS.setAckAiding(1);
    myGNSS.setI2CpollingWait(1);

    if (getLocalTime(&timeinfo)) {
      myGNSS.setUTCTimeAssistance(timeinfo.tm_year + 1900, timeinfo.tm_mon + 1, timeinfo.tm_mday,
                                  timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec, 0, 2, 0, 0, 
                                  SFE_UBLOX_MGA_ASSIST_ACK_YES, 100);
    } else {
      Serial.println("Failed to obtain time. This will not work well.");
    }

#ifndef USE_SERVER_ASSISTANCE
    myGNSS.setPositionAssistanceLLH(13521000, 1038198000, 100, 5000000, 
                                   SFE_UBLOX_MGA_ASSIST_ACK_YES, 100);
#endif

    if (payloadSize > 0) {
      myGNSS.pushAssistNowData(true, payload, (size_t)payloadSize, 
                              SFE_UBLOX_MGA_ASSIST_ACK_YES, 100);
    }
    
  } else {
    Serial.println();
    Serial.println(F("WiFi connection timed out after 60 seconds"));
    Serial.println(F("Proceeding with cold start..."));
  }
  
  // Clean up WiFi regardless of connection status
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  
  // Set final I2C polling wait
  myGNSS.setI2CpollingWait(125);
  
  Serial.println(F("Setup complete!"));
}