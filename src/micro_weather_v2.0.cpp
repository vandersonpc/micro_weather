
/**
 * micro_weather_v2.0.cpp
 *
 *
 * Solar Powered Micro Weather Station based on ESP-01 and BME280 Modules.
 *
 * Copyright (c) 2022 Vanderson Carvalho - https://www.vandersonpc.com
 *
 * --[ Used Libraries ]--
 *
 * - BMx280I2C
 * - ESP8266 Boards
 * - WiFiManager
 *
 * --[ Revisions ]--
 *
 * v1.0  - 01Mar22 - Initial Release
 * v1.1  - 03Mar22 - Fix sensor reading issues. Add PRINT_STATUS definition
 * v1.2  - 04MAR22 - Implemented Deep Sleep to save battery. Remove TaskScheduler.
 * v2.0  - 12MAR22 - Implement WifiManager and add config struct
 * v2.01 - 30APR22 - Minor Code layout changes 
 * 
 * 
 * --[ TODO ]--
 * 1.
 * 2.
 * 3.
 * 
**/

/**
 * -- Operational Modes Definitions --
 * 
*/

//#define PRINT_STATUS // uncomment to see serial data
//#define TEST_TIME    // uncomment to test elapsed time to read sensor and send data

/**
 * -- Version Definition --
 * 
 * 
*/
#define __VERSION__ 2.0

/**
 * -- Include Libraries --
 * 
 * 
*/
#include <EEPROM.h>
#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <WiFiManager.h>

// Sleep Time in sec

#define DFT_SLEEP_TIME  1  // Default publish interval = 1 minute

/**
 * -- BMP280 Sensor --
 * 
 * 
*/

#include <Wire.h>
#include <BMx280I2C.h>

#define I2C_ADDRESS 0x76

// create a BMx280I2C object using the I2C interface with I2C Address 0x76
BMx280I2C bmx280(I2C_ADDRESS);

// Variables

float temperature;
float pressure;
float local_altitude;
float humidity;

#define P_SEA_LEVEL 1026 // Atm Pressure @ sea level

/**
 * -- Functions protoypes declaration  --
 * 
 * 
*/
void readSensorCallBack();
void readVCC();
void saveConfig();
void loadConfig();
void printConfig();

float readAltitude (float seaLevel);
int eepromWriteString(int addr, String s);
String eepromReadString(int addr);


/** 
 * --  Wifi Config   --
 * 
 * Wifi AP Config is performed via WifiManager
 *
*/
WiFiClient  client;

const char* host        = "api.thingspeak.com";


/** 
 * -- Config Struct --
 * 
 * 
*/
typedef struct {
    String tsChannel;         // thingspeak channel number
    String tsKey;             // thingspeak write key
    String SSID;              // wifi SSID
    String wifiPass;          // wifi Password
    int    publishInterval;   // publish interval in minutes
    int    cfgMarker;         // config identification value
} _Config;

_Config mwConfig;

#define CFG_INITIALIZED_MARKER 2803
#define EEPROM_SIZE 1024

// Config ADC & Prepare chip to read battery voltage  //

ADC_MODE(ADC_VCC);
float batVoltage;

/* -----------[ START ]------------ */

/**
 * --[ Read BMP280 Sesnor Callback ]--
 * 
 * 
*/

void readSensorCallBack(){

  //start a measurement
  if (!bmx280.measure())
  {
    #ifdef PRINT_STATUS
      Serial.println("could not start measurement, is a measurement already running?");
    #endif
    return;
  }

  //wait for the measurement to finish
  do
  {
    delay(100);
  } while (!bmx280.hasValue());

  temperature     = bmx280.getTemperature();
  pressure        = bmx280.getPressure64();
  local_altitude  = readAltitude(P_SEA_LEVEL);


  // important: measurement data is read from the sensor in function hasValue() only.
  // make sure to call get*() functions only after hasValue() has returned true.
  if (bmx280.isBME280())
  {
    humidity  = bmx280.getHumidity();

  }

  readVCC(); // read VCC voltage

  // Print values via serial
  #ifdef PRINT_STATUS
    Serial.print("Temperature: ");  Serial.println(temperature);
    Serial.print("Pressure: ");     Serial.println(pressure)/100;
    Serial.print("Altitude: ");     Serial.println(local_altitude);
    Serial.print("Humidity: ");     Serial.println(humidity);
    Serial.print("Battery Volts: ");Serial.println(batVoltage);
    Serial.println();
  #endif

  // TCP CONNECTION
  #ifndef NO_TCP
    const int httpPort = 80;
    if (!client.connect(host, httpPort))
      {
        #ifdef PRINT_STATUS
          Serial.println("ERROR: Client NOT Connected. Nothing Sent! :-(");
        #endif
        return;
      }
    #ifdef PRINT_STATUS
      Serial.println("Client Connected. Data Sent!");
    #endif
  
  // Create thingspeak url 

    String url = "/update?key=";
    url += mwConfig.tsKey;
    url += "&field1=";
    url += String(temperature);     // Celsius
    url += "&field2=";
    url += String(pressure/100);    // Millibar
    url += "&field3=";
    url += String(local_altitude);  // Altitude
    url += "&field4=";
    url += String(humidity);        // Relative humidity
    url += "&field5=";
    url += String(batVoltage);      // Supply Voltage
    url += "\r\n";
    // Send request to the server
    client.print(String("GET ") + url + " HTTP/1.1\r\n" +
                 "Host: " + host + "\r\n" +
                 "Connection: close\r\n\r\n");
  #endif

}

/**
 * --[ Read Battery Voltage ]--
 * 
 * 
*/
void readVCC() {
  batVoltage = ESP.getVcc() / 1000.0;
}

/**
 * --[ Calculate Altitude ]--
 * 
 * @param seaLevel 
 * @return *** float 
*/
float readAltitude (float seaLevel) {

  float atmospheric = pressure / 100.0F;
  return 44330.0 * (1.0 - pow(atmospheric / seaLevel, 0.1903));

}

/**
 * --[ Save config to EEPROM ]--
 * 
 * 
*/
void saveConfig() {
    EEPROM.begin(EEPROM_SIZE);
    int addr = 0;
    EEPROM.put(addr, CFG_INITIALIZED_MARKER);
    addr += sizeof(CFG_INITIALIZED_MARKER);
    addr = eepromWriteString(addr, mwConfig.tsChannel);
    addr = eepromWriteString(addr, mwConfig.tsKey);
    //addr = eepromWriteString(addr, mwConfig.SSID);
    //addr = eepromWriteString(addr, mwConfig.wifiPass);

    EEPROM.put(addr, mwConfig.publishInterval);
    addr += sizeof(mwConfig.publishInterval);

    // update loadConfig() and printConfig() if anything else added here

    EEPROM.commit();

    // show saved config
    Serial.println("Config saved to eeprom");
    printConfig();
}

/**
 * --[ Load config from EEPROM ]--
 * 
 * 
*/
void loadConfig() {
    EEPROM.begin(EEPROM_SIZE);

    int addr = 0;
    EEPROM.get(addr, mwConfig.cfgMarker);         addr += sizeof(mwConfig.publishInterval);
    if (mwConfig.cfgMarker != CFG_INITIALIZED_MARKER) {
        Serial.println("*** No config found! ***");
        mwConfig.publishInterval = DFT_SLEEP_TIME; // Set default publish interval
        return;
    }

    mwConfig.tsChannel  = eepromReadString(addr);   addr += mwConfig.tsChannel.length() + 1;
    mwConfig.tsKey      = eepromReadString(addr);   addr += mwConfig.tsKey.length() + 1;
    //mwConfig.SSID       = eepromReadString(addr);   addr += mwConfig.SSID.length() + 1;
    //mwConfig.wifiPass   = eepromReadString(addr);   addr += mwConfig.wifiPass.length() + 1;
    EEPROM.get(addr, mwConfig.publishInterval);     addr += sizeof(mwConfig.publishInterval);

    EEPROM.commit();

    Serial.println("Config loaded from eeprom");
    #ifdef PRINT_STATUS
      printConfig();
    #endif
}

/**
 * --[ Print Configuration via Serial ]--
 * 
 * 
*/
void printConfig() {
    Serial.print("ThingSpeak channel: ");   Serial.println(mwConfig.tsChannel);
    Serial.print("ThingSpeak Key: ");       Serial.println(mwConfig.tsKey);
    Serial.print("Publish Interval: ");     Serial.println(mwConfig.publishInterval);
    Serial.println();
}

/**
 * --[ Write String from EEPROM ]--
 * 
 * @param addr - adress
 * @param s - string
 * @return int - address
*/
int eepromWriteString(int addr, String s) {
    int l = s.length();
    for (int i = 0; i < l; i++) {
        EEPROM.write(addr++, s.charAt(i));
    }
    EEPROM.write(addr++, 0x00);
    return addr;
}

/**
 * --[ Read String from EEPROM ]--
 * 
 * @param addr - address
 * @return String 
*/
String eepromReadString(int addr) {
    String s;
    char c;
    while ((c = EEPROM.read(addr++)) != 0x00) {
        s += c;
    }
    return s;
}

/**
 * --[ Execute ESP WifiManager ]--
 * 
*/
bool shouldSaveConfig = false;

void doWifiManager() {
    WiFiManagerParameter custom_thingspeak_channel("channel", "ThingSpeak Channel", mwConfig.tsChannel.c_str(), 9);
    WiFiManagerParameter custom_thingspeak_key("key", "ThingSpeak Key", mwConfig.tsKey.c_str(), 17);
    WiFiManagerParameter custom_publish_interval("publishInterval", "Publish Interval", String(mwConfig.publishInterval).c_str(), 6);

    WiFiManager wifiManager;

    Serial.println("Wifi Manager AP is accessible now");

    // disable WM debug if PRINT_STATUS flag is not set
    #ifndef PRINT_STATUS
      wifiManager.setDebugOutput(false);
    #endif

    wifiManager.setSaveConfigCallback([]() {
        Serial.println("Should save config");
        shouldSaveConfig = true;
    });

    wifiManager.addParameter(&custom_thingspeak_channel);
    wifiManager.addParameter(&custom_thingspeak_key);
    wifiManager.addParameter(&custom_publish_interval);

    wifiManager.setTimeout(60); // 1 minutes


    String ap_Name = String("Weather Station ID - ") + ESP.getChipId();

    // Always start config page and keep it on for 1 minute
    if (!wifiManager.startConfigPortal(ap_Name.c_str())){
        Serial.println("Failed to connect and hit 1 minute timeout");
        delay(3000);
    }

    // autoconnect if there's saved credentials
    // fetches ssid and pass and tries to connect
    // if it does not connect it starts an access point with the specified name
    // and goes into a blocking loop awaiting configuration
    wifiManager.setTimeout(240); // 4 minutes
    if (!wifiManager.autoConnect(ap_Name.c_str())) {
        Serial.println("Failed to connect and hit 4 minutes timeout");
        delay(3000);
        ESP.restart(); // Reset if timeout
        delay(5000);
    }

    //read updated parameters
    mwConfig.tsChannel        = String(custom_thingspeak_channel.getValue());
    mwConfig.tsKey            = String(custom_thingspeak_key.getValue());
    mwConfig.publishInterval  = String(custom_publish_interval.getValue()).toInt();

    //save the custom parameters to FS
    if (shouldSaveConfig) {
        Serial.println("Saving config...");
        saveConfig();
    }
}

/**
 * --[ Setup ]--
 * 
 * 
*/
void setup() {

  // test time to read sensor and send data.
  #ifdef TEST_TIME
    int t_start = millis();
  #endif

  // put your setup code here, to run once:
	Serial.begin(9600);

	// wait for serial connection to open (only necessary on some boards)
  while (!Serial);

  Serial.println(); Serial.println(); // 2 spaces
  Serial.println("Micro Weather Station - ID: " + String(ESP.getChipId()));
  Serial.println("Firmware Version " + String(__VERSION__));

  // set i2c pins
  Wire.begin(0, 2); // SDA, SDL

  // load configuration
  Serial.println("Loading config...");
  loadConfig();

	//begin() checks the Interface, reads the sensor ID (to differentiate between BMP280 and BME280)
	//and reads compensation parameters.
	if (!bmx280.begin())
	{
		Serial.println("begin() failed. check your BMx280 Interface and I2C Address.");
		while (1);
	}

  if (bmx280.isBME280())
  	Serial.println("Weather Sensor is a BME280");
  else
  	Serial.println("Weather Sensor is a BMP280");

  // reset sensor to default parameters.
	bmx280.resetToDefaults();

	// by default sensing is disabled and must be enabled by setting a non-zero
	// oversampling setting.
	// set an oversampling setting for pressure and temperature measurements.
	bmx280.writeOversamplingPressure(BMx280MI::OSRS_P_x16);
	bmx280.writeOversamplingTemperature(BMx280MI::OSRS_T_x16);

	// if sensor is a BME280, set an oversampling setting for humidity measurements.
	if (bmx280.isBME280())
		bmx280.writeOversamplingHumidity(BMx280MI::OSRS_H_x16);

  // Connect to WiFi network
  #ifdef PRINT_STATUS
    Serial.println("Connecting to WIFI...");
  #endif
  //if (WiFi.status() != WL_CONNECTED) {
  if (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("Running Wifi Manager...");
    doWifiManager();
  }

  // print wifi info
  Serial.print("WIFI Connected to: ");
  Serial.println(WiFi.SSID());
  Serial.print("Local IP: ");
  Serial.println(WiFi.localIP());

  // read sensor and send data
  readSensorCallBack();

  #ifdef TEST_TIME
    int t_stop = millis();
    Serial.println("Task time [ms]: ");
    Serial.println(t_stop-t_start);
  #endif

  // enable deepsleep to saving power
  Serial.println();
  Serial.println("Sleeping... See you in " + String(mwConfig.publishInterval) + " min....");
  delay(500); // delay 500ms
  ESP.deepSleep(mwConfig.publishInterval * 60 * 1e6); // Sleep time in minutes

}

/**
 * --[ Main Loop ]--
 * 
*/
void loop() {
  // not need to do nothing here :-)
}
