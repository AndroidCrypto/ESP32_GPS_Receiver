
/*
Version Management
27.02.2025 V01 first version
*/

/**
*
* Don't forget to change the Board in Arduino:
* for Heltec V2: Heltec WiFi LoRa 32(V2)
* for Heltec V3: Heltec WiFi LoRa 32(V3) / Wireless shell (V3) / ...
* for LilyGo T3S3 LoRa: ESP32S3 Dev Module
* or ESP32 Development Boards: ESP32-WROOM-DA Module
*
* - or in Tools menue:
* for Heltec V2: Tools - Board - esp32 - Heltec WiFi LoRa 32(V2)
* for Heltec V3: Tools - Board - esp32 - Heltec WiFi LoRa 32(V3) / Wireless shell (V3) / ...
* for LilyGo T3S3 LoRa: Tools - Board - esp32 - ESP32S3 Dev Module
* for ESP32 Development Boards: Tools - Board - esp32 - ESP32-WROOM-DA Module
*
*/

const char *PROGRAM_VERSION_FULL = "ESP32 MD GPS Module Tester V01";

//#define HELTEC_V2
#define HELTEC_V3
//#define LILYGO_T3S3_SX1262
//#define ESP32_SX1276_OLED
//#define ESP32_SX1276_TFT

// ------------------------------------------------------------------
// simple selection of the GPIO pins
#ifdef HELTEC_V2
// Control external power
#define IS_VEXT_CONTROL // uncomment this if not available
#define VEXT_POWER_CONTROL_PIN 21 // pin controls power to external devices
// NEO-6M GPS module
#define IS_NEO_6M
#define NEO6M_GPS_RX_PIN 12 // The GPS board has an RX terminal - connect it with GPIO 12
#define NEO6M_GPS_TX_PIN 13 // The GPS board has a  TX terminal - connect it with GPIO 13
#define NEO6M_GPS_BAUD 9600 // The GPS Baud rate on hardware serial
#endif

#ifdef HELTEC_V3
// Control external power
#define IS_VEXT_CONTROL // uncomment this if not available
#define VEXT_POWER_CONTROL_PIN 36 // pin controls power to external devices
// NEO-6M GPS module
#define IS_NEO_6M
#define NEO6M_GPS_RX_PIN 46 // The GPS board has an RX terminal - connect it with GPIO 46
#define NEO6M_GPS_TX_PIN 45 // The GPS board has a TX terminal - connect it with GPIO 45
#define NEO6M_GPS_BAUD 9600 // The GPS Baud rate on hardware serial
#endif

#ifdef LILYGO_T3S3_SX1262
// NEO-6M GPS module
#define IS_NEO_6M
#define NEO6M_GPS_RX_PIN 46 // The GPS board has an RX terminal - connect it with GPIO 46
#define NEO6M_GPS_TX_PIN 42 // The GPS board has a TX terminal - connect it with GPIO 42
#define NEO6M_GPS_BAUD 9600 // The GPS Baud rate on hardware serial
#endif

#ifdef ESP32_SX1276_OLED
// NEO-6M GPS module
#define IS_NEO_6M
#define NEO6M_GPS_RX_PIN 14 // The GPS board has an RX terminal - connect it with GPIO 12
#define NEO6M_GPS_TX_PIN 12 // The GPS board has a  TX terminal - connect it with GPIO 14
#define NEO6M_GPS_BAUD 9600 // The GPS Baud rate on hardware serial
#endif

#ifdef ESP32_SX1276_TFT
// NEO-6M GPS module
#define IS_NEO_6M
#define NEO6M_GPS_RX_PIN 12 // The GPS board has an RX terminal - connect it with GPIO 12
#define NEO6M_GPS_TX_PIN 14 // The GPS board has a  TX terminal - connect it with GPIO 14
#define NEO6M_GPS_BAUD 9600 // The GPS Baud rate on hardware serial
#endif

// -----------------------------------------------------------------------
// LilyGo T3S3 support for battery mode
#include "LilyGoLoRaBoard.h"

// -----------------------------------------------------------------------
// GPS interface

// Create an instance of the HardwareSerial class for Serial 2 for GPS
HardwareSerial gpsSerial(2);

void loop() {
  delay(10000);
  GPSTest();
}

bool GPSTest() {
  bool success = false;
  uint32_t counter = 0;
  uint32_t startmS;
  startmS = millis();
  while ((uint32_t)(millis() - startmS) < 2000)  // allows for millis() overflow
  {
    if (gpsSerial.available() > 0) {
      Serial.write(gpsSerial.read());
      counter++;
      if (counter > 3) success = true;
    }
  }
  Serial.println();
  Serial.flush();
  return success;
}

void setup() {
#ifdef LILYGO_T3S3_SX1262
  setupLilyGoBoard();
#else
  Serial.begin(115200);
  while (!Serial) {}
#endif
  Serial.println(PROGRAM_VERSION_FULL);

  // if we have a power control for devices put it on
#ifdef IS_VEXT_CONTROL
  setVextControl(true);
#endif

  // set up the second hardware serial interface for the GPS module
#ifdef IS_NEO_6M
  gpsSerial.begin(NEO6M_GPS_BAUD, SERIAL_8N1, NEO6M_GPS_TX_PIN, NEO6M_GPS_RX_PIN);
  bool gpsSuccess = GPSTest();
  Serial.println();
  Serial.println();
  if (gpsSuccess) {
    Serial.println(F("GPS module has responded"));
  } else {
    Serial.println(F("GPS module not responding, System is halting"));
    while (1)
      ;
  }
#else
  Serial.println(F("No GPS module available, System is halting"));
  while (1)
    ;
#endif
}

void setVextControl(boolean trueIsOn) {
#ifdef IS_VEXT_CONTROL
  if (trueIsOn) {
    pinMode(VEXT_POWER_CONTROL_PIN, OUTPUT);
    digitalWrite(VEXT_POWER_CONTROL_PIN, LOW);
  } else {
    // pulled up, no need to drive it
    pinMode(VEXT_POWER_CONTROL_PIN, INPUT);
  }
#endif
}
