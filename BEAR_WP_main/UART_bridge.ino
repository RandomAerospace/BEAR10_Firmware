// Add this to your setup()
void setup_uart_bridge() {
  // Serial2.begin(baud, config, rxPin, txPin);
  Serial2.begin(9600, SERIAL_8N1, UART_RX, UART_TX);
  Serial.println(F("[BRIDGE] UART Bridge Initialized at 9600 baud"));
}


void handle_uart_receiver() {
  if (Serial2.available() > 0) {
    // Read the incoming line until the newline character
    String rx_packet = Serial2.readStringUntil('\n');
    
    // Basic validation: ensure we have 3 commas (4 data points)
    int commaCount = 0;
    for (int i = 0; i < rx_packet.length(); i++) {
      if (rx_packet[i] == ',') commaCount++;
    }

    if (commaCount == 3) {
      // Parse the CSV string
      int firstComma = rx_packet.indexOf(',');
      int secondComma = rx_packet.indexOf(',', firstComma + 1);
      int thirdComma = rx_packet.indexOf(',', secondComma + 1);

      BV = rx_packet.substring(0, firstComma).toFloat();
      Current = rx_packet.substring(firstComma + 1, secondComma).toFloat();
      heaterOn = (rx_packet.substring(secondComma + 1, thirdComma).toInt() == 1);
      BatTemp = rx_packet.substring(thirdComma + 1).toFloat();

      // Debug output to verify
      Serial.print(F("[RX SUCCESS] BV: ")); Serial.print(BV);
      Serial.print(F("V, Current: ")); Serial.print(Current);
      Serial.print(F("A, Heater: ")); Serial.print(heaterOn ? "ON" : "OFF");
      Serial.print(F(", BatTemp: ")); Serial.println(BatTemp);
    } else {
      Serial.println(F("[RX ERROR] Malformed packet received"));
    }
  }
}