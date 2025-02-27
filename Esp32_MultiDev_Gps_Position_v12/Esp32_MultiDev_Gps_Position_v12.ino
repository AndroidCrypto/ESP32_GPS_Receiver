
/*
Version Management
27.02.2025 V12 final version
           V02 - V11 development 
21.02.2025 V01 first version
*/

/**
* Please uncomment just one hardware definition file that reflects your hardware combination
* for Heltec WiFi LoRa 32 V2 boards use HELTEC_V2
* for Heltec WiFi LoRa 32 V3 boards use HELTEC_V3
* for LilyGo T3S3 LoRa boards use LILYGO_T3S3_SX1262
* for ESP32 Development boards with attached LoRa module SX1276 module and OLED use ESP32_SX1276_OLED
* for ESP32 Development boards with attached LoRa module SX1276 module and TFT use ESP32_SX1276_TFT
* for all other boards and hardware combination you should consider to modify an existing one to your needs
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

//#define HELTEC_V2
#define HELTEC_V3
//#define LILYGO_T3S3_SX1262
//#define ESP32_SX1276_OLED
//#define ESP32_SX1276_TFT

// ------------------------------------------------------------------
// include the hardware definition files depending on the uncommenting
#ifdef HELTEC_V2
#include "Heltec_V2_Hardware_Settings.h"
#endif

#ifdef HELTEC_V3
#include "Heltec_V3_Hardware_Settings.h"
#endif

#ifdef LILYGO_T3S3_SX1262
#include "LilyGo_T3S3_LoRa_SX1262_Hardware_Settings.h"
#endif

#ifdef ESP32_SX1276_OLED
#include "ESP32_SX1276_OLED_Hardware_Settings.h"
#endif

#ifdef ESP32_SX1276_TFT
#include "ESP32_SX1276_TFT_Hardware_Settings.h"
#endif

// ------------------------------------------------------------------

// when using the (default) OLED display SSD1306 128 * 64 px the maximum length is 26 chars
const char *PROGRAM_VERSION = "GPS Position          V12";
const char *PROGRAM_VERSION_FULL = "ESP32 MD GPS Position Data V12";

// ------------------------------------------------------------------
// internal or external OLED SSD1306 128 * 64 px display

#ifdef IS_OLED
#include "FONT_MONOSPACE_9.h"
#include "FONT_MONOSPACE_18.h"
// For a connection via I2C using the Arduino Wire include:
#include <Wire.h>
#include "SSD1306.h"  // https://github.com/ThingPulse/esp8266-oled-ssd1306
SSD1306Wire display(OLED_I2C_ADDRESS, OLED_I2C_SDA_PIN, OLED_I2C_SCL_PIN);
#endif

#ifdef IS_TFT
// ------------------------------------------------------------------
// TFT display ST7735 1.8' 128 * 160 RGB
#include "FONT_MONOSPACE_9.h"
#include <SPI.h>
#include <Adafruit_GFX.h>                                        // Core graphics library, https://github.com/adafruit/Adafruit-GFX-Library
#include <Adafruit_ST7735.h>                                     // Hardware-specific library for ST7735, https://github.com/adafruit/Adafruit-ST7735-Library
Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);  // hardware SPI
#endif

// vars for displaying line 1 to 5 to display in a loop
String display1 = "";
String display2 = "";
String display3 = "";
String display4 = "";
String display5 = "";
// for TFT only
String display6, display7, display8, display9, display10, display11, display12, display13;

bool showDisplay = false;
bool isDisplayOn = false;  // this bool is needed for switching the display off after timer exceeds

// -----------------------------------------------------------------------
//  Node specific settings
#include "Node_Settings.h"  // include the node/sketch specific settings

// -----------------------------------------------------------------------
// PRG/Boot button
// #define BOOT_BUTTON_PIN 0 // see settings or hardware settings

bool isBootButtonPressed = false;

void IRAM_ATTR bootButtonPressed() {
  isBootButtonPressed = true;
  // deactivate the interrupt to avoid bouncing
  detachInterrupt(BOOT_BUTTON_PIN);
}

// -----------------------------------------------------------------------
// LilyGo T3S3 support for battery mode
#include "LilyGoLoRaBoard.h"

// -----------------------------------------------------------------------
// GPS library

#include <TinyGPS++.h>  // get library here > http://arduiniana.org/libraries/tinygpsplus/
// https://github.com/mikalhart/TinyGPSPlus
TinyGPSPlus gps;  // create the TinyGPS++ object
// Create an instance of the HardwareSerial class for Serial 2 for GPS
HardwareSerial gpsSerial(2);

bool isWaitingForFirstGpsFix = false;
bool GPSfix = false;
uint8_t TXStatus = 0;  //  used to store current status flag bits of Tracker transmitter (TX)
uint8_t TXPacketL;     // length of LoRa packet (TX)
// https://kleuthold.wordpress.com/2019/09/05/gps-modul-neo-6m-im-einsatz/
float TXLat;      // Latitude from GPS on Tracker transmitter (TX)
float TXLon;      // Longitude from GPS on Tracker transmitter (TX)
float TXAlt;      // Altitude from GPS on Tracker transmitter (TX)
uint8_t TXSats;   // number of GPS satellites seen (TX)
uint32_t TXHdop;  // HDOP from GPS on Tracker transmitter (TX), horizontal dilution of precision
// https://novotech.com/pages/horizontal-dilution-of-precision-hdop
float TXHdopFloat;      // HDOP from GPS on Tracker transmitter (TX)
uint16_t TXVolts;       // Volts (battery) level on Tracker transmitter (TX)
uint32_t TXGPSFixTime;  // GPS fix time in hot fix mode of GPS on Tracker transmitter (TX)

// is the gps data updated by a new fix ?
uint8_t TXGpsUpdateStatus = 0;
bool TXGpsLocationIsUpdated;    // TXGpsUpdateStatus Bit 0
bool TXGpsDateIsUpdated;        // TXGpsUpdateStatus Bit 1
bool TXGpsTimeIsUpdated;        // TXGpsUpdateStatus Bit 2
bool TXGpsSpeedIsUpdated;       // TXGpsUpdateStatus Bit 3
bool TXGpsCourseIsUpdated;      // TXGpsUpdateStatus Bit 4
bool TXGpsAltitudeIsUpdated;    // TXGpsUpdateStatus Bit 5
bool TXGpsSatellitesIsUpdated;  // TXGpsUpdateStatus Bit 6
bool TXGpsHdopIsUpdated;        // TXGpsUpdateStatus Bit 7

uint32_t TXGpsLocationAge;
bool TXGpsDateIsValid;

bool TXGpsTimeIsValid;
time_t localTime;
uint16_t TXyear;
uint8_t TXmonth;
uint8_t TXday;
uint8_t TXhour;
uint8_t TXminute;
uint8_t TXsecond;
uint8_t TXGpsDateIsValid8;
uint8_t TXGpsDateIsUpdated8;
uint8_t TXGpsTimeIsValid8;
uint8_t TXGpsTimeIsUpdated8;
double TXGpsSpeedKmh;
double TXGpsCourseDegree;
float TXGpsDistanceToReference;
uint16_t TXGpsCourseToReference;

uint32_t loopCnt = 0;
const unsigned long gpsDataIntervalMillis = GPS_DATA_INTERVAL_SECONDS * 1000;
long lastGpsDataMillis = 0;

//#define GPS_TEST_MODE 1
// A sample NMEA stream for testing.
const char *gpsStream =
  "$GPRMC,045103.000,A,3014.1984,N,09749.2872,W,0.67,161.46,030913,,,A*7C\r\n"
  "$GPGGA,045104.000,3014.1985,N,09749.2873,W,1,09,1.2,211.6,M,-22.5,M,,0000*62\r\n"
  "$GPRMC,045200.000,A,3014.3820,N,09748.9514,W,36.88,65.02,030913,,,A*77\r\n"
  "$GPGGA,045201.000,3014.3864,N,09748.9411,W,1,10,1.2,200.8,M,-22.5,M,,0000*6C\r\n"
  "$GPRMC,045251.000,A,3014.4275,N,09749.0626,W,0.51,217.94,030913,,,A*7D\r\n"
  "$GPGGA,045252.000,3014.4273,N,09749.0628,W,1,09,1.3,206.9,M,-22.5,M,,0000*6F\r\n";

// -----------------------------------------------------------------------
// UTC time converting
#include <Timezone.h>  // https://github.com/JChristensen/Timezone

// see here: https://github.com/JChristensen/Timezone/blob/master/examples/WorldClock/WorldClock.ino
// Central European Time (Frankfurt, Paris)
TimeChangeRule CEST = { "CEST", Last, Sun, Mar, 2, 120 };  // Central European Summer Time
TimeChangeRule CET = { "CET ", Last, Sun, Oct, 3, 60 };    // Central European Standard Time
Timezone CE(CEST, CET);

TimeChangeRule *tcr;  // pointer to the time change rule, use to get TZ abbrev

// -----------------------------------------------------------------------
// Battery Voltage measure - this is a dummy !
float TXVoltsFloat = 3.81;  // dummy value

void loop() {

  if (isWaitingForFirstGpsFix) {
#ifdef GPS_TEST_MODE
    while (*gpsStream) gps.encode(*gpsStream++);
    display3 = "GPS TEST MODE";
    displayData();
    isWaitingForFirstGpsFix = false;
    lastGpsDataMillis = millis();
    lastLoRaDataMillis = millis();
#else
    bool gpsSuccess = gpsWaitFix(WaitFirstGPSFixSeconds);
    if (gpsSuccess) {
      ledFlash(2, 125);
      displayGpsDataPosition();
      isWaitingForFirstGpsFix = false;
      lastGpsDataMillis = millis();
    } else {
      display4 = "no GPS fix data,";
      display5 = "waiting for first fix";
      displayData();
    }
#endif
  }
 
  if (isBootButtonPressed) {
    Serial.println("BOOT");
    isBootButtonPressed = false;
    attachInterrupt(BOOT_BUTTON_PIN, bootButtonPressed, RISING);
  }

  // get the gps data every xx seconds
  if ((millis() - lastGpsDataMillis > gpsDataIntervalMillis) && (!isWaitingForFirstGpsFix)) {
    loopCnt++;
    Serial.print(F("Loop cnt: "));
    Serial.println(loopCnt);

    if (gpsWaitFix(WaitGPSFixSeconds)) {
      Serial.println(F("Fix received"));
      ledFlash(2, 125);
      displayGpsDataPosition();
      printGpsData();
      Serial.print(gpsDataCsvHeader());
      Serial.println(gpsDataCsv(loopCnt, 0, 0, 0));  // dummy values for LoRa data
    } else {
      //ledFlash(1, 125);
      displayGpsData();
      Serial.println(F("No Fix received"));
    }
    lastGpsDataMillis = millis();
  }
}

void displayGpsData() {
  display1 = "** Current location **";
  display2 = "Lat:" + String(TXLat, 4);
  display3 = "Lon:" + String(TXLon, 4);
  display4 = "Alt:" + String(TXAlt, 1);
  displayData();
}

void displayGpsDataPosition() {
  display1 = "Lat:" + String(TXLat, 5) + " Lon:" + String(TXLon, 5);
  display2 = "Alt:" + String(TXAlt, 1) + " Sats:" + (String)TXSats + " Stat:" + (String)TXGpsUpdateStatus;
  display3 = "Hdop:" + (String)TXHdop + " Fixtime:" + (String)TXGPSFixTime;
  // the local date/time data is written in display4: printDateTime()
  Serial.println(display4);
  display5 = "Dis:" + String(TXGpsDistanceToReference, 1) + " Dir:" + String(TXGpsCourseToReference);
  displayData();
}

// used for timezone conversion
// format and print a time_t value, with a time zone appended.
void printDateTime(time_t t, const char *tz) {
  char buf[32];
  char m[4];  // temporary storage for month string (DateStrings.cpp uses shared buffer)
  strcpy(m, monthShortStr(month(t)));
  sprintf(buf, "%.2d:%.2d:%.2d %s %.2d %s %d %s",
          hour(t), minute(t), second(t), dayShortStr(weekday(t)), day(t), m, year(t) - 2000, tz);
  Serial.println(buf);
  display4 = buf;
}

void printGpsData() {
  Serial.println();
  Serial.println(F("--== GPS Data ==--"));
  Serial.print(F("LAT: "));
  Serial.print(TXLat, 8);
  Serial.print(F(""));
  Serial.print(F(" upd: "));
  Serial.println(TXGpsLocationIsUpdated);
  Serial.print(F("LOT:"));
  Serial.print(TXLon, 8);
  Serial.print(F(""));
  Serial.print(F(" upd: "));
  Serial.println(TXGpsLocationIsUpdated);
  Serial.print(F("Location Age: "));
  Serial.print(TXGpsLocationAge);
  Serial.print(F(""));
  Serial.print(F(" upd: "));
  Serial.println(TXGpsLocationIsUpdated);
  Serial.print(F("Alt:"));
  Serial.print(TXAlt);
  Serial.print(F(" m"));
  Serial.print(F(" upd: "));
  Serial.println(TXGpsAltitudeIsUpdated);
  char buf[32];
  char m[4];  // temporary storage for month string (DateStrings.cpp uses shared buffer)
  strcpy(m, monthShortStr(month(localTime)));
  sprintf(buf, "%.2d:%.2d:%.2d %s %.2d %s %d %s",
          hour(localTime), minute(localTime), second(localTime), dayShortStr(weekday(localTime)), day(localTime), m, year(localTime) - 2000, tcr->abbrev);
  Serial.print(F("Date & Time local: "));
  Serial.print(buf);
  Serial.print(F(""));
  Serial.print(F(" upd Date: "));
  Serial.print(TXGpsDateIsUpdated);
  Serial.print(F(" upd Time: "));
  Serial.println(TXGpsTimeIsUpdated);
  Serial.print(F("Speed: "));
  Serial.print(TXGpsSpeedKmh);
  Serial.print(F(" km/h"));
  Serial.print(F(" upd: "));
  Serial.println(TXGpsSpeedIsUpdated);
  Serial.print(F("Course: "));
  Serial.print(TXGpsCourseDegree);
  Serial.print(F(" degrees"));
  Serial.print(F(" upd: "));
  Serial.println(TXGpsCourseIsUpdated);
  Serial.print(F("Sats: "));
  Serial.print(TXSats);
  Serial.print(F(""));
  Serial.print(F(" upd: "));
  Serial.println(TXGpsSatellitesIsUpdated);
  Serial.print(F("Horizontal dilution of precision (HDOP): "));
  Serial.print(TXHdop);
  Serial.print(F(""));
  Serial.print(F(" upd: "));
  Serial.println(TXGpsHdopIsUpdated);
  Serial.print(F("Distance to reference: "));
  Serial.print(TXGpsDistanceToReference);
  Serial.print(F(" m"));
  Serial.print(F(" upd: "));
  Serial.println(TXGpsLocationIsUpdated);

  const char *cardinalToReference = TinyGPSPlus::cardinal(TXGpsCourseToReference);
  Serial.print(F("CourseToReference: "));
  Serial.print(TXGpsCourseToReference);
  Serial.print(F(" degrees"));
  Serial.print(F(" upd: "));
  Serial.println(TXGpsLocationIsUpdated);
  Serial.print(F("CourseToReference: "));
  Serial.print(cardinalToReference);
  Serial.print(F(""));
  Serial.print(F(" upd: "));
  Serial.println(TXGpsLocationIsUpdated);
}

String gpsDataCsvHeader() {
  String csv = "Date,Time, Counter, LAT, LON, Altitude, Distance, Sat, HDOP, TXCounter, RSSI, SNR\n";
  return csv;
}

String gpsDataCsv(uint32_t rxCounter, uint32_t txCounter, int16_t packetRssi, int8_t packetSnr) {
  String csv = "";
  csv += String(year(localTime) - 2000) + "-" + String(month(localTime)) + "-" + String(day(localTime)) + ",";
  csv += String(hour(localTime)) + ":" + String(minute(localTime)) + ":" + String(second(localTime)) + ",";
  csv += String(rxCounter) + ",";
  csv += String(TXLat, 10) + "," + String(TXLon, 10) + ",";
  csv += String(TXAlt, 0) + ",";
  csv += String(TXGpsDistanceToReference, 0) + ",";
  csv += String(TXSats) + ",";
  csv += String(TXHdop) + ",";
  //csv += String(txCounter) + ",";
  //csv += String(packetRssi) + ",";
  //csv += String(packetSnr) + "\n";
  // dummy data
  csv += String(0) + ",";
  csv += String(0) + ",";
  csv += String(0) + "\n";
  return csv;
}

void setTXGpsStatusByte(uint8_t bitnum, uint8_t bitval) {
  // program the status byte
  if (bitval == 0) {
    bitClear(TXGpsUpdateStatus, bitnum);
  } else {
    bitSet(TXGpsUpdateStatus, bitnum);
  }
}

bool readTXGpsStatusByte(byte bitnum) {
  return bitRead(TXStatus, bitnum);
}

// taken from https://github.com/StuartsProjects/SX12XX-LoRa/blob/bc9cf344c07c79dc1486514a35f36919e331831e/examples/SX126x_examples/Tracker/23_GPS_Tracker_Transmitter/23_GPS_Tracker_Transmitter.ino
bool gpsWaitFix(uint32_t waitSecs) {
  //waits a specified number of seconds for a fix, returns true for good fix
  Serial.print(F("Wait GPS Fix "));
  Serial.print(waitSecs);
  Serial.println(F("s"));

  uint32_t endwaitmS, GPSonTime;
  bool GPSFix = false;
  float tempfloat;
  uint8_t GPSchar;

  TXGpsUpdateStatus = 0;
  TXGpsLocationIsUpdated = false;
  TXGpsDateIsUpdated = false;
  TXGpsTimeIsUpdated = false;
  TXGpsSpeedIsUpdated = false;
  TXGpsCourseIsUpdated = false;
  TXGpsAltitudeIsUpdated = false;
  TXGpsSatellitesIsUpdated = false;
  TXGpsHdopIsUpdated = false;

  GPSonTime = millis();
  gpsSerial.begin(NEO6M_GPS_BAUD, SERIAL_8N1, NEO6M_GPS_TX_PIN, NEO6M_GPS_RX_PIN);  // start GPSserial

  endwaitmS = millis() + (waitSecs * 1000);

  while (millis() < endwaitmS) {
    if (gpsSerial.available() > 0) {
      GPSchar = gpsSerial.read();
      gps.encode(GPSchar);
    }

    if (gps.location.isUpdated() && gps.altitude.isUpdated()) {
      GPSfix = true;
      Serial.print(F("Have GPS Fix "));
      TXGPSFixTime = millis() - GPSonTime;
      Serial.print(TXGPSFixTime);
      Serial.println(F(" mS"));
      break;  //exit while loop reading GPS
    }

    // was the boot button pressed during wait time ?
    if (isBootButtonPressed) {
      isBootButtonPressed = false;
      attachInterrupt(BOOT_BUTTON_PIN, bootButtonPressed, RISING);
    }
  }

  if (gps.location.isUpdated()) {
    TXGpsLocationIsUpdated = true;
    TXLat = gps.location.lat();
    TXLon = gps.location.lng();
    TXGpsLocationAge = gps.location.age();
    setTXGpsStatusByte(0, 1);

    // calculate the distance and course to the reference GPS coordinates
    TXGpsDistanceToReference = TinyGPSPlus::distanceBetween(TXLat, TXLon, GPS_REFERENCE_LAT, GPS_REFERENCE_LON);
    TXGpsCourseToReference = (int16_t)TinyGPSPlus::courseTo(TXLat, TXLon, GPS_REFERENCE_LAT, GPS_REFERENCE_LON);
  }

  if (gps.date.isUpdated()) {
    TXGpsDateIsUpdated = true;
    TinyGPSDate dt = gps.date;
    TXyear = dt.year();
    TXmonth = dt.month();
    TXday = dt.day();
    setTXGpsStatusByte(1, 1);
  }

  if (gps.time.isUpdated()) {
    TXGpsTimeIsUpdated = true;
    TinyGPSTime tm = gps.time;
    TXhour = tm.hour();
    TXminute = tm.minute();
    TXsecond = tm.second();
    setTXGpsStatusByte(2, 1);
  }

  if (gps.speed.isUpdated()) {
    TXGpsSpeedIsUpdated = true;
    TXGpsSpeedKmh = gps.speed.kmph();
    setTXGpsStatusByte(3, 1);
  }

  if (gps.course.isUpdated()) {
    TXGpsCourseIsUpdated = true;
    TXGpsCourseDegree = gps.course.deg();
    setTXGpsStatusByte(4, 1);
  }

  if (gps.altitude.isUpdated()) {
    TXGpsAltitudeIsUpdated = true;
    TXAlt = gps.altitude.meters();
    setTXGpsStatusByte(5, 1);
  }

  if (gps.satellites.isUpdated()) {
    TXGpsSatellitesIsUpdated = true;
    TXSats = gps.satellites.value();
    setTXGpsStatusByte(6, 1);
  }

  if (gps.hdop.isUpdated()) {
    TXGpsHdopIsUpdated = true;
    TXHdop = gps.hdop.value();
    TXHdopFloat = ((float)TXHdop / 100.0);
    setTXGpsStatusByte(7, 1);
  }

  Serial.print(TXLat, 5);
  Serial.print(F(","));
  Serial.print(TXLon, 5);
  Serial.print(F(","));
  Serial.print(TXAlt, 1);
  Serial.print(F(","));
  Serial.print(TXSats);
  Serial.print(F(","));
  Serial.print(tempfloat, 2);
  Serial.println();

  // conversion utc/gps time to local time
  //setTime(myTZ.toUTC(compileTime()));
  //setTime(01, 55, 00, 11, 3, 2012);        //another way to set the time (hr,min,sec,day,mnth,yr)
  setTime(TXhour, TXminute, TXsecond, TXday, TXmonth, TXyear);  // another way to set the time (hr,min,sec,day,mnth,yr)

  // time from Timezone library
  time_t utc = now();
  localTime = CE.toLocal(utc, &tcr);
  display5 = "";
  printDateTime(localTime, tcr->abbrev);  // this will set display4

  //if here then there has either been a fix or no fix and a timeout

  if (GPSfix) {
    // GPSfix is already set to true
  } else {
    Serial.println();
    Serial.println(F("Timeout - No GPSFix"));
    Serial.println();
    GPSfix = false;
  }

  gpsSerial.end();  //serial RX interrupts interfere with SPI, so stop GPSserial
  return GPSfix;
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

void ledFlash(uint16_t flashes, uint16_t delaymS) {
  // run only if a LED is connected
  if (LED_PIN >= 0) {
    uint16_t index;
    for (index = 1; index <= flashes; index++) {
      digitalWrite(LED_PIN, HIGH);
      delay(delaymS);
      digitalWrite(LED_PIN, LOW);
      delay(delaymS);
    }
  }
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

  if (LED_PIN >= 0) {
    pinMode(LED_PIN, OUTPUT);  // setup pin as output for indicator LED
    ledFlash(1, 125);          // two quick LED flashes to indicate program start
  }

  // setup display
#ifdef IS_OLED
  if (OLED_I2C_RST_PIN >= 0) {
    pinMode(OLED_I2C_RST_PIN, OUTPUT);
    digitalWrite(OLED_I2C_RST_PIN, LOW);  // set GPIO16 low to reset OLED
    delay(50);
    digitalWrite(OLED_I2C_RST_PIN, HIGH);
    delay(50);
  }
  clearDisplayData();
  display.init();
#ifdef DISPLAY_ORIENTATION_FLIPPED
  // do nothing
#else
  display.flipScreenVertically();  // Landscape 90 degrees right rotated
#endif
  display.setFont(ArialMT_Plain_10);
  delay(50);
  display1 = PROGRAM_VERSION;
  displayData();
  delay(500);
#endif

  // init TFT display
#ifdef IS_TFT
  tft.initR(INITR_BLACKTAB);     // den ST7735S Chip initialisieren, schwarz
  tft.fillScreen(ST77XX_BLACK);  // und den Schirm mit Schwarz füllen
  tft.setTextWrap(false);        // automatischen Zeilenumbruch ausschalten
#ifdef DISPLAY_ORIENTATION_FLIPPED
  tft.setRotation(1);  // Landscape 270 degrees right rotated
#else
  tft.setRotation(3);              // Landscape 90 degrees right rotated
#endif
  Serial.println(F("Display init done"));
#endif

  display1 = PROGRAM_VERSION;
  display2 = "Display init done";
  displayData();
  delay(1000);

  // init the mode select button
  pinMode(BOOT_BUTTON_PIN, INPUT);
  attachInterrupt(BOOT_BUTTON_PIN, bootButtonPressed, RISING);

  // set up the second hardware serial interface for the GPS module
#ifdef IS_NEO_6M
  gpsSerial.begin(NEO6M_GPS_BAUD, SERIAL_8N1, NEO6M_GPS_TX_PIN, NEO6M_GPS_RX_PIN);
  bool gpsSuccess = GPSTest();
  Serial.println();
  Serial.println();
  if (gpsSuccess) {
    display3 = "Wait for first GPS response";
    displayData();
  } else {
    Serial.println(F("GPS module not responding, System is halting"));
    display3 = "GPS module not responding";
    display4 = "System is halting";
    displayData();
    ledFlash(50, 50);  // long fast speed flash indicates no GPS device error
    while (1)
      ;
  }
#else
  display3 = "No GPS module available";
  display4 = "System is halting";
  displayData();
  ledFlash(50, 50);  // long fast speed flash indicates no GPS device error
  while (1)
    ;
#endif

  display3 = "GPS wait for first fix";
  display4 = "";
  display5 = "";
  displayData();
  delay(2000);

  isWaitingForFirstGpsFix = true;
}

void displayData() {
#ifdef IS_TFT
  tft.fillScreen(ST77XX_BLACK);
  tft.setTextSize(1);
  tft.setFont(NULL);  // Pass NULL to revert to 'classic' fixed-space bitmap font.
  tft.setCursor(0, 0);
  tft.print(display1);
  tft.setCursor(0, 10);
  tft.print(display2);
  tft.setCursor(0, 20);
  tft.print(display3);
  tft.setCursor(0, 30);
  tft.print(display4);
  tft.setCursor(0, 40);
  tft.print(display5);
  tft.setCursor(0, 50);
  tft.print(display6);
  tft.setCursor(0, 60);
  tft.print(display7);
  tft.setCursor(0, 70);
  tft.print(display8);
  tft.setCursor(0, 80);
  tft.print(display9);
  tft.setCursor(0, 90);
  tft.print(display10);
  tft.setCursor(0, 100);
  tft.print(display11);
  tft.setCursor(0, 110);
  tft.print(display12);
  tft.setCursor(0, 120);
  tft.print(display13);
#endif

#ifdef IS_OLED
  display.clear();
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.setFont(Monospaced_plain_9);
  display.drawString(0, 0, display1);
  display.drawString(0, 12, display2);
  display.drawString(0, 24, display3);
  display.drawString(0, 36, display4);
  display.drawString(0, 48, display5);
  display.display();
#endif
}

void displayOledDataLarge() {
#ifdef IS_OLED
  display.clear();
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.setFont(Monospaced_plain_18);
  display.drawString(0, 0, display1);
  display.drawString(0, 20, display2);
  display.drawString(0, 40, display3);
  //display.drawString(0, 36, display4);
  //display.drawString(0, 48, display5);
  display.display();
#endif
  // if a TFT display is attached the regular (small)
  // font size will be used
#ifdef IS_TFT
  displayData();
#endif
}

void clearDisplayData() {
  display1 = "";
  display2 = "";
  display3 = "";
  display4 = "";
  display5 = "";
  display6 = "";
  display7 = "";
  display8 = "";
  display9 = "";
  display10 = "";
  display11 = "";
  display12 = "";
  display13 = "";
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

/*
GPS NEO-6M module response on startup
$GPRMC,170601.00,V,,,,,,,210225,,,N*7A
$GPGGA,170601.00,,,,,0,00,99.99,,,,,,*67
$GPRMC,170602.00,V,,,,,,,210225,,,N*79
$GPGGA,170602.00,,,,,0,00,99.99,,,,,,*64
*/
/*
GPS NEO-7M module response on startup
$GPRMC,114712.00,V,,,,,,,230225,,,N*79
$GPVTG,,,,,,,,,N*30
$GPGGA,114712.00,,,,,0,00,99.99,,,,,,*66
$GPGSA,A,1,,,,,,,,,,,,,99.99,99.99,99.99*30
$GPGSV,3,1,10,06,02,024,,11,25,046,,12,24,100,,18,12,176,*73
$GPGSV,3,2,10,20,12,072,,25,62,102,,28,60,266,20,29,79,199,*75
$GPGSV,3,3,10,31,43,299,,32,09,231,*77
$GPGLL,,,,,114712.00,V,N*4A
*/
/*
GPS NEO-7M module active response
$GPRMC,120226.00,A,5117.63412,N,00652.06871,E,0.967,,230225,,,A*70
$GPVTG,,T,,M,0.967,N,1.791,K,A*25
$GPGGA,120226.00,5117.63412,N,00652.06871,E,1,06,1.67,99.4,M,46.5,M,,*65
$GPGSA,A,3,29,25,11,12,20,26,,,,,,,2.52,1.67,1.89*0E
$GPGSV,3,1,10,11,20,041,17,12,18,103,26,18,18,175,,20,15,067,15*77
$GPGSV,3,2,10,25,55,107,28,26,25,290,19,28,60,252,,29,85,169,20*71
$GPGSV,3,3,10,31,48,292,08,32,03,227,*72
$GPGLL,5117.63412,N,00652.06871,E,120226.00,A,A*65
*/
