void setup_mavlink() {
  // MinimOSD usually expects 57600 baud
  MavSerial.begin(57600, SERIAL_8N1, UART2_RX, UART2_TX);
  Serial.println(F("[MAV] MAVLink UART2 Initialized at 57600"));
}

void setup_VTX_EN(){

  pinMode(PIN_VTX_EN, OUTPUT);
  digitalWrite(PIN_VTX_EN, LOW);
}

void enable_VTX(){
  digitalWrite(PIN_VTX_EN, HIGH);
}

void disable_VTX(){
  digitalWrite(PIN_VTX_EN, LOW);


}



void task_mavlink_osd() {
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  // 1. Send HEARTBEAT (Required for OSD to "wake up")
  mavlink_msg_heartbeat_pack(1, 200, &msg, MAV_TYPE_ANTENNA_TRACKER, MAV_AUTOPILOT_GENERIC, MAV_MODE_GUIDED_ARMED, 0, MAV_STATE_ACTIVE);
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  MavSerial.write(buf, len);

  // 2. Send SYS_STATUS (Battery Voltage and Current)
  // Voltage is in millivolts (mV), Current is in 10 * milliamps (cA)
  uint16_t voltage_mv = (uint16_t)(BV * 1000); 
  int16_t current_ca = (int16_t)(Current * 100); 
  
  len = mavlink_msg_to_send_buffer(buf, &msg);
  MavSerial.write(buf, len);
  mavlink_msg_sys_status_pack(
    1, 200, &msg,
    0,          // onboard_control_sensors_present
    0,          // onboard_control_sensors_enabled
    0,          // onboard_control_sensors_health
    0,        // load (50.0%)
    voltage_mv, // voltage_battery (mV)
    current_ca, // current_battery (10 * mA)
    0,        // battery_remaining (%)
    0,          // drop_rate_comm
    0,          // errors_comm
    0,          // errors_count1
    0,          // errors_count2
    0,          // errors_count3
    0,          // errors_count4
    0,          // battery_ma_h (extended field 17)
    0,          // battery_v_extra (extended field 18)
    0           // battery_status_flags (extended field 19)
  );
  len = mavlink_msg_to_send_buffer(buf, &msg);
  MavSerial.write(buf, len);

  // 3. GLOBAL_POSITION_INT
  int32_t lat_int = (int32_t)(glatitude * 1e7);
  int32_t lon_int = (int32_t)(glongitude * 1e7);
  int32_t alt_mm = (int32_t)(galtitude * 1000); 
  int16_t speed_cms = (int16_t)(gspeed * 27.77); 

  mavlink_msg_global_position_int_pack(1, 200, &msg, millis(), lat_int, lon_int, alt_mm, alt_mm, 0, 0, 0, (uint16_t)gheading);
  len = mavlink_msg_to_send_buffer(buf, &msg);
  MavSerial.write(buf, len);
}