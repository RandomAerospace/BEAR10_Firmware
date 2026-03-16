//TO DO:
//1.RTC

// include the libraries
#include <RadioLib.h>
#include <SPI.h>
#include <Wire.h>


#include <Adafruit_I2CDevice.h>
#include <Adafruit_I2CRegister.h>

#include <WiFi.h>
#include <HTTPClient.h>
#include "secrets.h"


//HEATER AND CUTTER PINS
#define PIN_HEATER 25
#define PIN_CUTTER 26
bool heaterOn = false;  //for use in telemetry only

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
const unsigned long RTTY_TRANSMIT_TIME = 15000;  // give 15000 to tiemeout transmission
const unsigned long RTTY_COOLDOWN_TIME = 250;    // 250 second cooldown


// Define VSPI pins
#define VSPI_SCK 18
#define VSPI_MISO 19
#define VSPI_MOSI 23
#define VSPI_SS 5

//I2C sensors setup
#define I2C_SDA 21
#define I2C_SCL 22

//BATTERY MONITOR//
//define analog read for battery voltage
#define batt_pin 32
// Constants for battery monitoring
float battery_voltage = 0.0f;
const float ADC_MAX = 4095.0f;              // 12-bit ADC (ESP32)
const float ADC_REF_VOLTAGE = 3.3f;         // Reference voltage
const float VOLTAGE_DIVIDER = 8.0f / 3.3f;  // Voltage divider ratio
const int NUM_SAMPLES = 15;                 // Number of samples to average

// GPIO where the DS18B20 is connected to
#include <OneWire.h>
#include <DallasTemperature.h>
// Setup a oneWire instance to communicate with any OneWire devices
#define oneWireBus 17
// Used to be GPIO35 but it's INPUT only so switched to GPIO17 or the seventh pin on the left side of the esp32
OneWire oneWire(oneWireBus);
// Pass our oneWire reference to Dallas Temperature sensor
DallasTemperature onesense(&oneWire);

//MCP9600 thermocouple
#include "Adafruit_MCP9600.h"
#define MCP9600_ADDR (0x60)
Adafruit_MCP9600 mcp;
const int thermocouple_wakeup = 500;  //give 250ms to wakeup
float cold_junc = 0.0f;
uint16_t thermocouple_adc = 0;

//MS5611 baro
#include "MS5611.h"
MS5611 MS5611(0x77);
//int16_t baro_temp = 30;  //needs to be signed due to -ve temps


//ASM330 IMU setup
#include <ASM330LHHSensor.h>
#define ASM330_ADDR (0x6A)
ASM330LHHSensor AccGyr(&Wire, ASM330_ADDR);

int32_t accelerometer[3] = {};
int32_t gyroscope[3] = {};
int32_t Ax = 0;
int32_t Ay = 0;
int32_t Az = 0;
int32_t Gx = 0;
int32_t Gy = 0;
int32_t Gz = 0;

#ifndef MSBFIRST
#define MSBFIRST SPI_MSBFIRST
#endif


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

long latitude = 0;   //CHANGE BEFORE FLIGHT
long longitude = 0;  //CHANGE BEFORE FLIGHT
long altitude = 0;
long speed = 0;
long heading = 0;

//RTC stuff
uint16_t hour = 0;
uint16_t minute = 0;
uint16_t second = 0;


const int RF_OUTPUT_POWER = 20;
const float RF_FREQUENCY = 432.600f;
const float TCXO_VOLTAGE = 2.4f;
String RTTY_PREAMBLE = "$$";
const int RTTY_PREAMBLE_LENGTH = 8;  // number of RTTY_PREAMBLE repetitions
const int RTTY_FREQUENCY_SHIFT = 425;
const int RTTY_BAUD_RATE = 100;
const int RTTY_STOP_BITS = 2;

//RTTY PACKET FORMAT AND VARIABLE DECLARATIONS
uint8_t frame_counter = 0;
char thermocouple_temp_str[10];
float thermocouple_temp = 0.0f;
char ambient_temp_str[10];
float ambient_temp = 0.0f;
uint16_t baro_press = 0;
String payload_info = "012345789ABR";

//0->PayloadID/callsign
//1->Tx counter
//2->Time
//3,4,5->Lat long Alt
//7->speed
//8->heading
//9->battery Voltage

//A->Internal Temp
//B->External Temp
//R->Pressure
//DOUBLE CHECK THE PAYLOAD TYPE DEFINITIONS!!!!
//"$$9V1WP,%u,%u:%u:%u,%.4f,%.4f,%.4f,%.2f,%.2f,%.2f,%.2f,%.2f,%u,%s,%d",
//payload struct
struct Payload {
  uint8_t frame_counter;
  uint16_t hour;
  uint16_t minute;
  uint16_t second;
  float latitude_processed;
  float longitude_processed;
  float altitude_processed;
  float speed_processed;
  float heading_processed;
  float battery_voltage;
  char thermocouple_temp_str[10];  //Variable for ext temp remember that char is different from char*(char array) which is this
  char ambient_temp_str[10];       //variable for internal temp/onewire temp sensor
  uint16_t baro_press;
  String payload_info;
  bool heaterOn;  //on telemetry it will be 1 or 0

  /*
        accelerometer[0]+","+
        accelerometer[1]+","+
        accelerometer[3]+","+
        gyroscope[0]+","+
        gyroscope[1]+","+
        gyroscope[3];
        */
};






// SX1278 has the following connections:
// NSS pin:   5
// DIO0 pin:  33
// RESET pin: 2
// DIO1 pin:  13
#define RADIO_TXEN 27  //this is to enable the outer Power Amplifier, if you do RX in the future, it will be good to have the RXen pin too.
#define RADIO_DIO2 14  //dio2 pijn

SX1268 radio = new Module(VSPI_SS, 33, 2, 13);
// get pointer to the common layer
PhysicalLayer* phy = (PhysicalLayer*)&radio;

// create AFSK client instance using the FSK module
// this requires connection to the module direct
// input pin, here connected to GPIO12
// SX127x/RFM9x:  DIO2
// RF69:          DIO2
// SX1231:        DIO2
// CC1101:        GDO2
// Si443x/RFM2x:  GPIO
// SX126x/LLCC68: DIO2
AFSKClient audio(&radio, RADIO_DIO2);

// create AX.25 client instance using the AFSK instance
AX25Client ax25(&audio);
// create APRS client instance using the AX.25 client
APRSClient aprs(&ax25);
// create RTTY client instance using the FSK module
RTTYClient rtty(&radio);

void setup() {
  unsigned long currentMillis = millis();
  static unsigned long previousMillis = 0;
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
  radio_setup();
  baro_setup();
  Bat_temp_setup();
  setup_heater();
  Cutter_setup();

  Serial.print("All setup");
}

void loop() {

  task_heater();

  Cutter();


  //checks the battery temp every 5s, changes state every 5s, also checks if RTTY is going to be transmitted.
  //this is why it is placed at the front of the loop because i don't want GPIO to be high before RTTY Tx.
  //also the heater eats too much current with the energizer lithiums, it will cause the esp32 to bootloop

  //only during IDLE and cutterOn FALSE will sensors update
  if (rttyState == RTTY_IDLE && cutterOn == false) {
    updateSensors();
  }

  RTTY_TX();
}


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
    previousGNSSMillis = currentMillis;
  }
  if (currentMillis - previousBATTEMPMillis >= Bat_temp_interval) {  //this happens every 750ms
    read_bat_temp();
    //this is because the conversion time is <750ms
  }
  if (currentMillis - previousMillis >= sensor_interval) {  //this happens every 250ms
    read_baro();
    //readIMU();
    read_battery();
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
