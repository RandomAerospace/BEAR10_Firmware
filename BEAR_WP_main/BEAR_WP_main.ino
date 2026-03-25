//TO DO:
//APRS PARAMETERS
String callsign = "9V1WP";
String callsign_suffix = "-11";
uint8_t callsign_ssid = 11;
String comment_suffix = "FPV@1.28GHz";
String boot_message = "BEAR13 Project";

//APRS
uint16_t msg_id = 0;
bool freefall=false;


//CUTTER CONFIG
const float CUT_ALTITUDE = 30000 * 1000.0f;  //input as  meters, this will be converted to mm
bool cutterOn = false;                       //for use in telemetry only
const unsigned long CUT_DURATION = 15000;    //cut duration in ms

#define TEMP_SP_HIGH_DEGC 15.0f
#define TEMP_SP_LOW_DEGC 10.0f

//timing variables
const unsigned long sensor_interval = 250;  // read sensors every 100ms
const unsigned long setup_interval = 2000;
const unsigned long GNSS_interval = 1000;  //poll GNSS every 1 second
const unsigned long Bat_temp_interval = 800;

// Enum to represent the states of our RTTY transmission
enum RTTYState {
  RTTY_IDLE,
  RTTY_START,
  RTTY_TRANSMITTING,
  RTTY_COOLDOWN
};

// rtty specific baro_temptiming variables
RTTYState rttyState = RTTY_IDLE;
unsigned long rttyStateStartTime = 0;
const unsigned long RTTY_IDLE_TIME = 30000;      // 20 seconds between transmissions
const unsigned long RTTY_START_TIME = 1000;      // 250 ms idle signal
const unsigned long RTTY_TRANSMIT_TIME = 6000;  // give 6000 to tiemeout transmission
const unsigned long RTTY_COOLDOWN_TIME = 250;    // 250 second cooldown


// Global variables for UART BRIDGE
float BV = 0.0f;
float Current = 0;
bool heaterOn = false;
float BatTemp = 0;

//RADIO PINS

// GPIO where the DS18B20 is connected to
//#include <OneWire.h>
//#include <DallasTemperature.h>
// Setup a oneWire instance to communicate with any OneWire devices
//#define oneWireBus 17
// Used to be GPIO35 but it's INPUT only so switched to GPIO17 or the seventh pin on the left side of the esp32
//OneWire oneWire(oneWireBus);
// Pass our oneWire reference to Dallas Temperature sensor
//DallasTemperature onesense(&oneWire);

//MCP9600 thermocouple
//#include "Adafruit_MCP9600.h"
//#define MCP9600_ADDR (0x60)
//Adafruit_MCP9600 mcp;
//const int thermocouple_wakeup = 500;  //give 250ms to wakeup
//float cold_junc = 0.0f;
//uint16_t thermocouple_adc = 0;

//MS5611 baro
#include "MS5611.h"
MS5611 MS5611(0x77);
//int16_t baro_temp = 30;  //needs to be signed due to -ve temps


//GPS
const char assistNowServer[] = "https://online-live1.services.u-blox.com";
//const char assistNowServer[] = "https://online-live2.services.u-blox.com"; // Alternate server

const char getQuery[] = "GetOnlineData.ashx?";
const char tokenPrefix[] = "token=";
const char tokenSuffix[] = ";";
const char getGNSS[] = "gnss=gps,glo,qzss,bds,gal;";  // GNSS can be: gps,qzss,glo,bds,gal
const char getDataType[] = "datatype=eph,alm,aux;";   // Data type can be: eph,alm,aux,pos

#ifdef USE_SERVER_ASSISTANCE
const char useLatitude[] = "lat=1.3521;";     // Use an approximate latitude of 55 degrees north. Replace this with your latitude.
const char useLongitude[] = "lon=103.8198;";  // Use an approximate longitude of 1 degree west. Replace this with your longitude.
const char useAlt[] = "alt=100;";             // Use an approximate latitude of 100m above WGS84. Replace this with your altitude.
const char usePosAcc[] = "pacc=60000;";       // Use a position accuracy of 60000m (60km)
#endif

#include <SparkFun_u-blox_GNSS_Arduino_Library.h>  //http://librarymanager/All#SparkFun_u-blox_GNSS
SFE_UBLOX_GNSS myGNSS;

#include "time.h"
// The Network Time Protocol Servers, ntpServer is main, the rest are back ups
const char* ntpServer = "time.nist.gov";
const char* ntpServer_01 = "1.pool.ntp.org";
const char* ntpServer_02 = "0.pool.ntp.org";

// GPS
// Fix Type
// 0: No fix
// 1: Dead reckoning
// 2: 2D
// 3: 3D
// 4: GNSS + Dead reckoning
// 5: Time only

byte fixType = 0;
float glatitude = 1.3521;   //CHANGE BEFORE FLIGHT
float glongitude =  103.8198;  //CHANGE BEFORE FLIGHT
float galtitude = 0;
float paltitudeMSL=0;
float gspeed = 0;
uint16_t gheading = 0;

//RTC stuff [RBF]
uint16_t tyear=2026;
uint8_t tmonth=3;
uint8_t tday=24;
uint16_t thour = 0;
uint16_t tminute = 0;
uint16_t tsecond = 0;
uint16_t tms = 0;
unsigned long long tsync = 0;


//CAM SWITCH PWM SETTINGS
// Hardware PWM Settings
const int ledcChannel = 0;
const int ledcFreq = 50;      // 50Hz (20ms period)
const int ledcRes = 13;       // 13-bit resolution (0-8191)



//RTTY PACKET FORMAT AND VARIABLE DECLARATIONS
uint8_t frame_counter = 0;
float ambient_temp = 0.0f; //barometer temp
float external_temp=0.0f; //type K temp
float bat_temp=0.0f; //battery temperature from UART bridge
uint16_t baro_press = 0;



//////////////////////////////////////////////////////
//// Pinmap
//////////////////////////////////////////////////////

#define PIN_DTR 0
// Define VSPI pins
#define VSPI_SCK 18
#define VSPI_MISO 19
#define VSPI_MOSI 23
#define VSPI_SS 27

//I2C sensors setup
#define I2C_SDA 21
#define I2C_SCL 22

//UART pins (header pins) remmeber to pin matrix
//purpose is to serve as UART bridge with arduino
//THIS IS THE SDA/SCL ON THE 1X06 JST HEADER
#define UART_TX 32 //I2C SDA
#define UART_RX 33 //I2C SCL

//Mavlink
#define UART2_TX 16
#define UART2_RX 17

//Camera Switch pins
#define PIN_CAM_SWITCH 26

//VTX_EN PIN
#define PIN_VTX_EN 35

//CUTTER EN PIN
#define PIN_CUTTER 34

//DRA818V PIN MAPPING
#define PIN_RAD_PTT 2
#define PIN_RAD_EN 13
#define PIN_RAD_PW 12
#define PIN_RAD_SQL 39 //sensor_vn
#define PIN_RAD_TX 15 //HARDWARE FLIPPED ALREADY
#define PIN_RAD_RX 14 //HARDWARE FLIPPED ALREADY
#define PIN_TX_AUD 25
#define PIN_RX_AUD 36 //sensor_VP

// include the libraries
#include <SPI.h>
#include <Wire.h>


//temperature sensr
#include <Adafruit_MAX31856.h>
// Initialize with Software SPI to force the specific VSPI pins
Adafruit_MAX31856 maxthermo = Adafruit_MAX31856(VSPI_SS, VSPI_MOSI, VSPI_MISO, VSPI_SCK);




#include <Adafruit_I2CDevice.h>
#include <Adafruit_I2CRegister.h>

#include <WiFi.h>
#include <HTTPClient.h>
#include "secrets.h"


//mavlink
#include <MAVLink.h>
HardwareSerial MavSerial(2);

//APRS
#include <APRSLite.h>


////////////////////////////////////////////////////////
//// DRA818
//////////////////////////////////////////////////////
#ifdef HW_ESP32S3
#include "hal/ledc_types.h"
#include "soc/ledc_periph.h"
#include "soc/ledc_struct.h"
#include "hal/gpio_hal.h"
#include "esp_rom_gpio.h"
ledc_dev_t *ledc = &LEDC;
#else
#include "soc/rtc_io_reg.h"
#include "soc/rtc_cntl_reg.h"
#include "soc/sens_reg.h"
#include "soc/rtc.h"
#include <driver/dac.h>
#include <hal/dac_hal.h>
#include <hal/dac_ll.h>
#endif
#include <SoftwareSerial.h>
SoftwareSerial radioCtrl(PIN_RAD_RX, PIN_RAD_TX);

//////////////////////////////////////////////////////
//// WIFI
//////////////////////////////////////////////////////
#include <WiFi.h>
#include <WiFiAP.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <DynamicArduinoOTA.h>
#include <UDPStream.h>
#include <esp_wifi.h>

UDPStream udpstream;


void setup() {
  unsigned long currentMillis = millis();
  static unsigned long previousMillis = 0;
  // Bug fix, pullup DTR so we can boot properly if we reset
  pinMode(PIN_DTR, INPUT_PULLUP);



  Serial.begin(115200);
  if (currentMillis - previousMillis >= setup_interval) {
    previousMillis = currentMillis;
    Serial.println("");
  }  // Small delay to avoid reset during SPI initialization
  Serial.print("Show SPI pins");
  Serial.print("MOSI: ");
  Serial.println(MOSI);
  Serial.print("MISO: ");
  Serial.println(MISO);
  Serial.print("SCK: ");
  Serial.println(SCK);
  Serial.print("SS: ");
  Serial.println(SS);
  if (currentMillis - previousMillis >= setup_interval) {
    previousMillis = currentMillis;
    Serial.println("");
  }  // Small delay to avoid reset during SPI initialization


  //initialise i2c
  Serial.print("Initialising I2c");
  Wire.begin();
  Wire.setClock(100000);  // Set to 10 kHz to clock stretch the MCP9600

  //initialise i2c for asm330lhhtr

  // Set I2C_disable = 0 in CTRL4_C (0x13) (optional, as it is 0 by default)
  /*
  Wire.beginTransmission(ASM330_ADDR);
  Wire.write(0x13);
  Wire.write(0x00); //Clear I2C_disable
  
  // Set DEVICE_CONF = 1 in CTRL9_XL (0x18)
  Wire.beginTransmission(ASM330_ADDR);
  Wire.write(0x18);
  Wire.write(0x01); //Clear I2C_disable
  
  Serial.println("Configuration complete.");
  */

  Assistnow_setup(); 
  GNSS_setup();
  //thermocouple_setup();
  //radio_setup();
  baro_setup();
  //Bat_temp_setup();
  //setup_heater();
  //Cutter_setup();
  setup_cam_switch();
  setup_temp();
  setup_uart_bridge();

  // Will transmit boot message, ensure dra818 is ready
  //setup DRA818 first before APRS.
  Serial.println("DRA818");
  setup_dra818();
  Serial.println("APRS");
  setup_aprs();

  Serial.print("All setup");

  setup_mavlink();
}

void loop() {

  //task_heater();

  //Cutter();


  //checks the battery temp every 5s, changes state every 5s, also checks if RTTY is going to be transmitted.
  //this is why it is placed at the front of the loop because i don't want GPIO to be high before RTTY Tx.
  //also the heater eats too much current with the energizer lithiums, it will cause the esp32 to bootloop

  //only during IDLE and cutterOn FALSE will sensors update
  if (rttyState == RTTY_IDLE && cutterOn == false) {
    updateSensors();
  }

  // Update OSD every 500ms for smooth display
  static unsigned long lastMavUpdate = 0;
  if (millis() - lastMavUpdate >= 2000) {
    task_mavlink_osd();
    lastMavUpdate = millis();
  }

  RTTY_TX();
}

//THIS FUNCTION UPDATES SENSORS ACCORDING TO RTTY_STATE
void updateSensors() {
  unsigned long currentMillis = millis();
  static unsigned long previousMillis = 0;
  static unsigned long previousGNSSMillis = 0;
  static unsigned long previousBATTEMPMillis = 0;
  // Break out early if we're not in RTTY_IDLE state
  if (rttyState != RTTY_IDLE) {
    return;
  }
  if (currentMillis - previousGNSSMillis >= GNSS_interval) {  //this happens every 1000ms
    read_gnss();
    handle_uart_receiver();
    previousGNSSMillis = currentMillis;
  }

  if (currentMillis - previousMillis >= sensor_interval) {  //this happens every 250ms
    read_baro();
    task_temp();
    //readIMU();
    //read_battery();
    previousMillis = currentMillis;
  }
  /*
  // Add debugging
  static unsigned long last_debug_time = 0;
  if (currentMillis - last_debug_time >= 5000) {  // Print every 5 seconds
    Serial.print(F("[DEBUG] RTTY State: "));
    Serial.print(rttyState);
    
    Serial.print(F(" Heater Pin: "));
    Serial.print(digitalRead(PIN_HEATER));
    Serial.print(F(" Temp: "));
    Serial.println(ambient_temp);
    last_debug_time = currentMillis;
  }
  */
}
