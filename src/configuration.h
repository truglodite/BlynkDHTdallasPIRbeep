//////////////////////////////////////////////////////////
// configuration.h
// by: Truglodite
// updated: 5/29/2019
//
// General configuration for BlynkDHTdallasPIRbeep.
//
//////////////////////////////////////////////////////////
#pragma once
// Github users can add a /src/privacy.h with #define privacy, etc.
#ifndef privacy
  const char deviceName[] =     "deviceName";// This is appended to hostname & messages
  const char authToken[] =      "myAuthToken"; // Blynk App authToken token
  #define dhtType               DHT11         // DHT22, DHT11, etc...

  const char ssid[] =           "mySSID";    // Wifi SSID
  const char pass[]=            "myWifiPassword";// Wifi WPA2 password
  //const IPAddress staticIP      (192,168,1,3);// Static local IP (setup your router accordingly)
  //const byte mac[] =            {0xDE,0xAD,0xBE,0xEF,0xFE,0xED};// Wifi MAC
  const char* update_username = "username";      // OTA username
  const char otaPassword[] =    "password";    // OTA password (old pass: kwaker5)
  const char* update_path =     "/firmware";  // OTA webserver update directory
#endif

#define firmwareVpin                V0            // Firmware Upload Button, 0= Normal, 1= Firmware OTA
#define armButtonVpin               V1            // Arm/Disarm button
#define ledVpin                     V2            // Alarm status LED
#define triggersVpin                V3            // Slider to indicate # of triggers
#define temp1Vpin                   V4            // Dallas temperature [F]
#define humidVpin                   V5            // Relative Humidity [%]
#define heatIndexVpin               V6            // Heat Index [F]
#define dewPointVpin                V7            // Dew Point [F]
#define pirPin                      13            // Physical pin: PIR output
#define dhtPin                      12            // DHT sensor data pin (default io12)
#define triggersMax                 50            // Reset trigger count to 0 when > this value
#define dallasPin                   14            // Dallas sensor data pin
#define beepPin                     16            // Beeper signal pin
#define dallasResolution            12            // temperature resolution bits (9 default, up to 12)
#define dhtRetriesMax               5             // Number of DHT "NAN" reads before rebooting
#define unitsFarenheit                            // Comment out to enable Celsius output
const unsigned long dallasPeriod=   800;          // msec to wait between start of conversion and reading (>760 for 12bit)
const unsigned long dhtPeriod =     2000;         // msec between DHT readings
const unsigned long pirTimeout =    30000;        // msec between last PIR read and trigger reset
const unsigned long checkConnPeriod=11000;        // msec between connection checks
const unsigned long uploadPeriod=   30000;        // msec between uploads (don't flood Blynk... "10 values per sec")
#define minNotifyPeriod             300000        // msec notificaitons will be delayed if they repeat faster than this
const unsigned long restartPeriod=  60000;        // msec to wait for Wifi+Blynk connection before reset
const unsigned long otaTimeout =    300000;       // msec to wait for OTA upload before reboot (ie. period between OTA notifications)
#define beepDelay                   100           // msec length of and delay between beeps

//*******************  Global Variables  *********************//
////////////////////////////////////////////////////////////////
const char hostNameX[] =           "ESP-";               // WiFi and OTA hostname, default "ESP-[deviceName]"
const char notifyOTAreadyX[] =     "OTA Waiting\nhttp://"; // Template for OTA ready notification text (just to store the required chars)
const char notifyPIRX[] =          ": Movement!"; // Armed PIR notification text
const char notifyDHTfailX[] =      ": DHT read error";   // Failed DHT read notification text
