/**
 * This program engages with multiple sensors for long-term data logging and 
 * actuate a stepper motor using a driver board (TMC5160 SilentStepStick) when 
 * needed (for automatic acid/base titration, for example). It is designed to 
 * log data locally while also sends them over to an IoT server for real time 
 * storage and rapid analysis.
 * 
 * Author: Jian Gong.
 * MIT, Bosak Lab, Aug 2019
 * 
 * MIT License
 * Copyright (c) 2019 - Jian Gong
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
 
//== Top Level Configurations
bool Back_OLED_Screen = true; // Back_OLED_Screen is connected (debugging only)
bool DEBUG = true;            // Enable DEBUG serial output lines (begins with "::")
int  LOG_INTERVAL = 3;        // Data monitor interval (seconds)
int   pH_INTERVAL = 1;        // Update pH during Calibration (seconds)
bool UPDATE_RTC = false;      // Force update RTC timer
const char *DATAFILESD = "/dataLog_000.csv";  // name of file on SD card

//== IoT Data Logging
#include <WiFi101.h>
#include "iot_config.h"
char ssid[]        = WIFI_SSID;      // your network SSID (name)
char pass[]        = WIFI_PASS;      // your network password (use for WPA, or use as key for WEP)
char io_username[] = IO_USERNAME;    // Adafruit IoT
char io_key[]      = IO_KEY;         // Adafruit IoT
#include <WiFiUdp.h>
#include <ArduinoHttpClient.h>

const char serverIP[] = "192.168.7.220";
int serverPort = 8080;
WiFiClient wifi;
WebSocketClient webSocket = WebSocketClient(wifi, serverIP, serverPort);
int status = WL_IDLE_STATUS;
bool hasNetwork;

//== FlexyStepper Motor Control Library
#include <FlexyStepper.h>
FlexyStepper stepper;

//== TMC5160 Stepper Driver
#include <TMCStepper.h>

#define MOTOR_CS           13 // Chip select
#define MOTOR_DIR          12 // Direction
#define MOTOR_STEP         11 // Step
#define MOTOR_EN           10 // Enable

//#define SW_MOSI          19 // Software Master Out Slave In (MOSI)
//#define SW_MISO          21 // Software Master In Slave Out (MISO)
//#define SW_SCK           20 // Software Slave Clock (SCK)

#define R_SENSE 0.075f      // Specific for TMC5160 Chip

TMC5160Stepper motordriver(MOTOR_CS, R_SENSE);                           // Hardware SPI
//TMC5160Stepper motordriver(MOTOR_CS, R_SENSE, SW_MOSI, SW_MISO, SW_SCK);  // Software SPI

//== Buttons for Control
#define BT_PIN_A         9  // UP
#define BT_PIN_B         6  // DOWN
#define BT_PIN_C         5  // SELECT

//== Display TFT ST7789 (240x240)/ST7735 (128x160) with SD

#include <SPI.h>
#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7735.h> // Hardware-specific library for ST7735
#include <Adafruit_ST7789.h> // Hardware-specific library for ST7789

#define TFT_CS        A1
#define TFT_RST       A2 // Or set to -1 and connect to Arduino RESET pin
#define TFT_DC        A3
#define SD_CS         A4 // Chip select line for SD card

//== Display associated SD card
#include <SdFat.h>           // For accessing onboard SD card
#include <sdios.h>
SdFat sd;    //File system object
SdFile file; // log files

ArduinoOutStream cout(Serial); // used to output sd card directory list
char sd_filename[20];
int sd_fileseq = 0;
bool errorSDCard;

//using hardware SPI (faster)
//Adafruit_ST7735 FrontDisplay = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);
Adafruit_ST7789 FrontDisplay = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);

//using software SPI (any two pins, but slower)
//#define TFT_MOSI 11  // Data out
//#define TFT_SCLK 13  // Clock out
//Adafruit_ST7735 FrontDisplay = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST);
//Adafruit_ST7789 FrontDisplay = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST);

template <unsigned N> // Template of menu system. N: number of menu items to be specified
  struct Menu{
    const char *title;
    const char *name;
    const char *values[N];
  };

String dataHeaderLine = "DateTime, DataIndex, pH, RTD, Temp(C), RelHumidity(%), Pressure(mBar), ";

//== Atlas Scientific Probe Circuits
#include <Wire.h>
#define pH_address      99          // default I2C address for EZO pH Circuit.
#define RTD_address     102         // default I2C address for EZO RTD Circuit.
#define CO2_address     105         // default I2C address for CO2 sensor

//== Environmental Sensor: SGP30
#include <Adafruit_SGP30.h>
#define SGP30_address   0x58        // default I2C address for Adafruit SGP30 Breakout Board

//== Environmental Sensor: BME280
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

//#define BME_SCK 13
//#define BME_MISO 12
//#define BME_MOSI 11
//#define BME_CS 10

#define SEALEVELPRESSURE_HPA (1013.25)
#define BME280_address  0x77        // default I2C address for 4-pin BME280 Breakout Board: 0x77 or 0x76

Adafruit_BME280 bme; // I2C
//Adafruit_BME280 bme(BME_CS); // hardware SPI
//Adafruit_BME280 bme(BME_CS, BME_MOSI, BME_MISO, BME_SCK); // software SPI

//== Adafruit DS3231 RTC I2C
#include "RTClib.h"
RTC_DS3231 rtc;

/*__________________________________________________________SETUP__________________________________________________________*/

void setup() {
  Serial.begin(9600);
//  while(!Serial); // Holds the program until serial connection is established.
//  if(DEBUG) Serial.println("\n:: Serial connection established...");

  //Setting Button Inputs
  pinMode(BT_PIN_A, INPUT_PULLUP);
  pinMode(BT_PIN_B, INPUT_PULLUP);
  pinMode(BT_PIN_C, INPUT_PULLUP);

  //Setting Motor Pins
  pinMode(MOTOR_EN, OUTPUT);
  pinMode(MOTOR_STEP, OUTPUT);
  pinMode(MOTOR_DIR, OUTPUT);
  digitalWrite(MOTOR_EN, LOW);         // Enable driver in hardware

  SPI.begin();                         // SPI drivers

  motordriver.begin();                 // SPI: Init CS pins and possible SW SPI pins
  motordriver.toff(5);                 // Enables driver in software
  motordriver.rms_current(400);        // Set motor RMS current [Nema 11: 400, Nema 17: 1000]
  motordriver.microsteps(16);          // Set microsteps to 1/16th

  motordriver.en_pwm_mode(true);       // Toggle stealthChop on TMC2130/2160/5130/5160
  motordriver.pwm_autoscale(true);     // Needed for stealthChop

  stepper.connectToPins(MOTOR_STEP, MOTOR_DIR);

  //tft.initR(INITR_BLACKTAB);         // Init ST7735S chip, black tab
  FrontDisplay.init(240, 240);         // Init ST7789 240x240
  FrontDisplay.setRotation(2);         // rotate 180 degrees
  FrontDisplay.setTextWrap(false);     // Don't wrap text to next line
  
  //Setting Atlas Probe Circuits
  Wire.begin();                        // Enable I2C port

  //Setting up BME280 Sensor
  unsigned bme_status;
  bme_status = bme.begin();

  //Setting up serial data output
  Serial.println(dataHeaderLine);

  //Setting DS3231/DS1307 RealTimeClock
  if (! rtc.begin()) {
    Serial.println(":: Couldn't find RTC");
  //} else if (rtc.isrunning()) { //DS1307
  } else if (rtc.lostPower()) { //DS3231
    Serial.println(":: RTC lost power, lets set the time!");
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    Serial.println(":: RTC Time Updated. Ready to go.");
  } else if (UPDATE_RTC) { // Manual update flag is on
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    Serial.println(":: RTC Time Updated. Ready to go.");
    // This line sets the RTC with an explicit date & time, for example to set
    // January 21, 2014 at 3am you would call:
    // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
  } else {
    Serial.println(":: RTC timer status is normal.");
  }

  //IoT Data Logging
    //Configure pins for Adafruit ATWINC1500 Feather
  WiFi.setPins(8,7,4,2);

  unsigned long checkTime = millis();
  const int timeOut = 10000;

  hasNetwork = true;
  Serial.print(":: Connecting to wifi network: ");
  while ( WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print('.');
    status = WiFi.begin(ssid, pass);
    if ( (millis() - checkTime) >= timeOut ) {
      Serial.println(":: Cannot connect to network in time. Continue anyways.");
      hasNetwork = false;
      break;
    }
  }
  if (hasNetwork) {
    Serial.print("\r\n");
    Serial.print(":: Connected to ");
    Serial.println(WiFi.SSID());
    Serial.print(":: IP address:\t");
    IPAddress ip = WiFi.localIP();
    Serial.println(ip);
  }

  //SD Card Logging
  if (!sd.begin(SD_CS, SD_SCK_MHZ(50))) {
    Serial.print("SD Card Error!");
    errorSDCard = true;
  } else {
    errorSDCard = false;
    strcpy(sd_filename, DATAFILESD);
    for (uint8_t i = 0; i < 200; i++) {
      sd_filename[9] = '0' + i/100;
      sd_filename[10] = '0' + i/10;
      sd_filename[11] = '0' + i%10;
      // create if does not exist, do not open existing, write, sync after write
      if (! sd.exists(sd_filename)) {
        sd_fileseq = i;
        break;
      }
    }
  
    // list SD card directory
    cout << F("\nList of files on the SD Card:\n");
    sd.ls("/", LS_R);
    
    if (!file.open(sd_filename, O_CREAT | O_WRITE)) {
      Serial.println("Cannot create file on the SD card.");
    } else {
      Serial.print("Writing to "); 
      Serial.println(sd_filename);
      file.println(dataHeaderLine);
      file.flush(); // makes sure the data is written to file
    }
  }
} //SETUP

int id;                  // index of acquired data
String timeString;       // RTC datetime stored in this
char dateTimeBuffer[12];
String dataString1;      // the main string to store all data (line 1)
String dataString2;      // the main string to store all data (line 2)

double pH_value;         // var used to hold the float value of pH
double RTD_value;        // var used to hold the float value of RTD
double T_value;          // var used to hold the float value of Temperature
double H_value;          // var used to hold the float value of Relative Humidity
double P_value;          // var used to hold the float value of Atmospheric Pressure

int  menu_nav[2]  = {1,1};    // menu array for GUI: {level_1_location, level_2_location}, like a coordinate
int  menu_level   = 0;        // keep track of menu level: 0 is root/1st level, 1 is 2nd level
bool need_refresh = true;     // to refresh screen
Menu<5> Menu_root     = { " Bosak Lab SensorBot ", " MENU ",       { "Monitor Mode", 
                                                                     "Auto Titration", 
                                                                     "Log Sensors",
                                                                     "UV/Light Sensing",
                                                                     "Settings" } };

Menu<5> Menu_titration = { " Bosak Lab SensorBot ", " TITRATION ", { "Move Motor/Pump",
                                                                     "Calibrate pH",
                                                                     "Execute Titration",
                                                                     "Beam Data",
                                                                     "Back to MENU" } };
Menu<5> Menu_display;

/*__________________________________________________________LOOP__________________________________________________________*/

void loop() {
  if (need_refresh) {    // only refresh screen once during boot/return from other menus (save energy)
    if (menu_level == 0) // assign menu content based on coordinate
      Menu_display = Menu_root;
    else if ((menu_nav[0] == 2) && (menu_level == 1))
      Menu_display = Menu_titration;
    showMenu(Menu_display);
    need_refresh = false;
  }

  // Managing Menus
  if (!digitalRead(BT_PIN_B) && digitalRead(BT_PIN_C) && digitalRead(BT_PIN_A)){
    menu_nav[menu_level]++;
    showMenu(Menu_display);
    delay(100);
    while (!digitalRead(BT_PIN_B) && digitalRead(BT_PIN_C) && digitalRead(BT_PIN_A)); // Ignore button holds
  }
  if (!digitalRead(BT_PIN_A) && digitalRead(BT_PIN_B) && digitalRead(BT_PIN_C)){
    menu_nav[menu_level]--;
    showMenu(Menu_display);
    delay(100);
    while(!digitalRead(BT_PIN_A) && digitalRead(BT_PIN_B) && digitalRead(BT_PIN_C));  // Ignore button holds
  }
  if (!digitalRead(BT_PIN_C) && digitalRead(BT_PIN_A) && digitalRead(BT_PIN_B)){
    executeAction();
    need_refresh = true;
    delay(100);
    while (!digitalRead(BT_PIN_C) && digitalRead(BT_PIN_A) && digitalRead(BT_PIN_B));  // Ignore button holds
  }
  
  // Web socket interactions
  if (webSocket.connected()) { // waiting to connect to websocket, and listen for incoming
    int msgLength = webSocket.parseMessage();  // get message length
    if (msgLength > 0) {                       // if it's > 0,
      String message = webSocket.readString(); // read it
      Serial.println(message);                 // print it
    }
  }

  if (Serial.available()) { // send text from Serial out to socket server
    String input = Serial.readString();
    if (input == "c") {
      connectToWebSocketServer();
    } else if (input == "x") {
      webSocket.stop();
    } else {
      if (webSocket.connected()) {           // and the webSocket's connected,
        webSocket.beginMessage(TYPE_TEXT);   // message type: text
        webSocket.print(input);
        webSocket.endMessage();
      }
    }
  }
} //LOOP

void showMenu(Menu<5> Menu_display) {
  switch (menu_nav[menu_level]) {
    case 0:
      menu_nav[menu_level] = 1;
      break;
    case 1 ... 5:
      cycleScreen(menu_nav[menu_level], Menu_display);
      break;
      
    case 6:
      menu_nav[menu_level] = 5;
      break;

    default:
      menu_nav[menu_level] = 1;
      break;
  }
}

void cycleScreen(int menu_item, Menu<5> Menu_display) {
        // Line 1: each line is 18 pixels high, 20 characters long (12 pixels wide per character)
      FrontDisplay.fillScreen(ST77XX_BLACK);
      FrontDisplay.setTextSize(2); //18 x 12 pixels per character
      FrontDisplay.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
      FrontDisplay.setCursor(0,0);
      FrontDisplay.println(Menu_display.title);
        // Line 2: each line is 18 pixels high, 20 characters long (12 pixels wide per character)
      FrontDisplay.setCursor((120-strlen(Menu_display.name)*6),18*1); // Center the name
      FrontDisplay.setTextColor(ST77XX_BLACK, ST77XX_WHITE); // 'inverted' text
      FrontDisplay.print(Menu_display.name);
      FrontDisplay.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
        // Line 3: each line is 18 pixels high, 20 characters long (12 pixels wide per character)
        // Line 4: each line is 18 pixels high, 20 characters long (12 pixels wide per character)
      FrontDisplay.setCursor(1,18*2+6);
      if (menu_item == 1) FrontDisplay.print(">"); else FrontDisplay.print(" ");
      FrontDisplay.print(Menu_display.values[0]);
        // Line 5: each line is 18 pixels high, 20 characters long (12 pixels wide per character)
      FrontDisplay.setCursor(1,18*3+6);
      if (menu_item == 2) FrontDisplay.print(">"); else FrontDisplay.print(" ");
      FrontDisplay.print(Menu_display.values[1]);
        // Line 6: each line is 18 pixels high, 20 characters long (12 pixels wide per character)
      FrontDisplay.setCursor(1,18*4+6);
      if (menu_item == 3) FrontDisplay.print(">"); else FrontDisplay.print(" ");
      FrontDisplay.print(Menu_display.values[2]);
        // Line 7: each line is 18 pixels high, 20 characters long (12 pixels wide per character)
      FrontDisplay.setCursor(1,18*5+6);
      if (menu_item == 4) FrontDisplay.print(">"); else FrontDisplay.print(" ");
      FrontDisplay.print(Menu_display.values[3]);
        // Line 8: each line is 18 pixels high, 20 characters long (12 pixels wide per character)
        // Line 9: each line is 18 pixels high, 20 characters long (12 pixels wide per character)
      FrontDisplay.setCursor(1,18*7+6);
      if (menu_item == 5) FrontDisplay.print(">"); else FrontDisplay.print(" ");
      FrontDisplay.print(Menu_display.values[4]);
        // Line 10: each line is 18 pixels high, 20 characters long (12 pixels wide per character)
        // Line 11: each line is 18 pixels high, 20 characters long (12 pixels wide per character)
        // Line 12: each line is 18 pixels high, 20 characters long (12 pixels wide per character)
        // Line 13: each line is 18 pixels high, 20 characters long (12 pixels wide per character)
      if (errorSDCard) {
        FrontDisplay.setCursor(36,18*11+6);
        FrontDisplay.setTextColor(ST77XX_RED, ST77XX_BLACK);
        FrontDisplay.print("!SD Card Error!");
      } else {
        FrontDisplay.setCursor(36,18*11+6);
        FrontDisplay.setTextColor(ST77XX_GREEN, ST77XX_BLACK);
        FrontDisplay.print(sd_filename);
      }
      FrontDisplay.drawRect(0, 37, 240, 197, ST77XX_WHITE);  // Draw a rectangle box around the menu
}

void executeAction() {                   // governs what happens when the "select" button is pressed
  if (menu_level == 0) {                 // if we are at the root menu level
    switch (menu_nav[menu_level]) {
      case 1:                              // monitor all sensors (no sub menus)
        action_1_monitor_sensors();
        break;
      case 2:                              // go to pH titration menu
        menu_level = 1;
        menu_nav[menu_level] = 1;
        break;
      case 3:
        action3_log_sensors();
        break;
      case 4:
        action4_uv_light_log();
        break;
      case 5:
        action5_settings();
        break;
    }
  } else if (menu_level == 1) {          // if we are at the titration menu level
    switch (menu_nav[menu_level]) {
      case 1:
      action21_adjust_pump();              // sub menu 1: Move motor UP/DOWN
      break;
      case 2:
      action22_calibrate_pH();             // sub menu 2: Monitor pH/Temp & Calibrate
      break;
      case 3:
      action23_execute_titration();        // sub menu 3: Execute pH Titration
      break;
      case 4:
      action24_custom();                   // sub menu 4: Custom
      break;
      case 5:
      menu_level = 0;                      // sub menu 5: Return to previous level
      menu_nav[menu_level] = 1;
      break;
    }
  } 
}

void action_1_monitor_sensors() {
  // Monitor/log all sensory data continuously until the next button press
  if(DEBUG) Serial.println(":: Begin logging sensory data.");
  
    // Static Line
    // Line 1: each line is 18 pixels high, 20 characters long (12 pixels wide per character)
  FrontDisplay.fillScreen(ST77XX_BLACK);
  FrontDisplay.setTextSize(2); //18 x 12 pixels per character
  FrontDisplay.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
  FrontDisplay.setCursor(0,0);
  FrontDisplay.println(Menu_display.values[0]);
    // Line 2: each line is 18 pixels high, 20 characters long (12 pixels wide per character)
  FrontDisplay.setCursor(0,18*1+6);
  FrontDisplay.print("_______");
  FrontDisplay.setTextColor(ST77XX_BLACK, ST77XX_WHITE); // 'inverted' text
  FrontDisplay.print(" DATA ");
  FrontDisplay.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
  FrontDisplay.println("_______");

  delay(1000); // This gives time to make sure that the button has been released.

  while( digitalRead(BT_PIN_C) ) { // Exit the loop if the button is pressed again
    // Getting current DateTime: YYYY-MM-DD HH:MM:SS
    DateTime now = rtc.now();

    sprintf(dateTimeBuffer, "%04u-%02u-%02u ", now.year(), now.month(), now.day());
    timeString = dateTimeBuffer;
    sprintf(dateTimeBuffer, "%02u:%02u:%02u,\t", now.hour(), now.minute(), now.second());
    timeString += dateTimeBuffer;
    
    // Setting up dataStrings
    dataString1 = String(id);
    dataString1 += ",\t";
    
    // Acquire pH Probe Data
    if(DEBUG) Serial.println(":: Acquired pH data.");
    pH_value = get_AtlasEZOProbe_I2C_Data(pH_address);
    dataString1 += String(pH_value,3);
    dataString1 += ",\t";
  
    if(DEBUG) Serial.println(":: Acquired RTD data.");
    RTD_value = get_AtlasEZOProbe_I2C_Data(RTD_address);
    dataString1 += String(RTD_value,3);
    dataString1 += ",\t";
    
    if(DEBUG) Serial.println(":: Acquired BME280 sensory data.");
    T_value = bme.readTemperature();
    H_value = bme.readHumidity();
    P_value = bme.readPressure();
    dataString2  = String(T_value,3);
    dataString2 += ",\t";
    dataString2 += String(H_value,3);
    dataString2 += ",\t";
    dataString2 += String(P_value/100,3);
    dataString2 += ",\t";
  
    Serial.print(timeString);
    Serial.print(dataString1);
    Serial.println(dataString2);
    
    // Front Display Update
      // Line 4: each line is 18 pixels high, 20 characters long (12 pixels wide per character)
    FrontDisplay.setCursor(1,18*2+6);
    FrontDisplay.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
    FrontDisplay.print( "#" );
    FrontDisplay.print( String(id) );
    FrontDisplay.print( " " );
    FrontDisplay.println( timeString.substring(5,19) );
      // Line 5: each line is 18 pixels high, 20 characters long (12 pixels wide per character)
    FrontDisplay.setCursor(1,18*3+6);
    FrontDisplay.print( " pH    = " );
    FrontDisplay.println( String(pH_value,3) );
      // Line 6: each line is 18 pixels high, 20 characters long (12 pixels wide per character)
    FrontDisplay.setCursor(1,18*4+6);    
    FrontDisplay.print( " T(lq) = " );
    FrontDisplay.print( String(RTD_value,3) );
    FrontDisplay.println( " C" );
      // Line 7: each line is 18 pixels high, 20 characters long (12 pixels wide per character)
    FrontDisplay.setCursor(1,18*5+6);
    FrontDisplay.print( " T(g)  = ");
    FrontDisplay.print(T_value,3);
    FrontDisplay.println(" C");
      // Line 8: each line is 18 pixels high, 20 characters long (12 pixels wide per character)
    FrontDisplay.setCursor(1,18*6+6);
    FrontDisplay.print( " H(g)  = ");
    FrontDisplay.print(H_value,3);
    FrontDisplay.println(" %");
      // Line 9: each line is 18 pixels high, 20 characters long (12 pixels wide per character)
    FrontDisplay.setCursor(1,18*7+6);
    FrontDisplay.print(" P = ");
    FrontDisplay.print(P_value/100,3);
    FrontDisplay.println(" mBar" );
      // Line 10: each line is 18 pixels high, 20 characters long (12 pixels wide per character)
      // Line 11: each line is 18 pixels high, 20 characters long (12 pixels wide per character)
      // Line 12: each line is 18 pixels high, 20 characters long (12 pixels wide per character)
      // Line 13: each line is 18 pixels high, 20 characters long (12 pixels wide per character)
    FrontDisplay.setCursor(1,18*11+6);
    FrontDisplay.println("< Hold to RETURN" );

    id ++;           // increase one ID next iteration

    delay(LOG_INTERVAL*1000);
  } // while button not pressed, continue to run the code block
}

void action21_adjust_pump() {
  // Adjusting Syringe Pump UP/DOWN
  
    // Static Display Lines
    // Line 1: each line is 18 pixels high, 20 characters long (12 pixels wide per character)
  FrontDisplay.fillScreen(ST77XX_BLACK);
  FrontDisplay.setTextSize(2);
  FrontDisplay.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
  FrontDisplay.setCursor(0,0);
  FrontDisplay.println(Menu_display.values[0]);
    // Line 2: each line is 18 pixels high, 20 characters long (12 pixels wide per character)
  FrontDisplay.setCursor(0,18*1);
  FrontDisplay.print("_______");
  FrontDisplay.setTextColor(ST77XX_BLACK, ST77XX_WHITE); // 'inverted' text
  FrontDisplay.print(" MOTOR ");
  FrontDisplay.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
  FrontDisplay.println("______");
    // Line 3: each line is 18 pixels high, 20 characters long (12 pixels wide per character)
    // Line 4: each line is 18 pixels high, 20 characters long (12 pixels wide per character)
  FrontDisplay.setCursor(1,18*2+6);
  FrontDisplay.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
  FrontDisplay.println(" UP to move up");
    // Line 5: each line is 18 pixels high, 20 characters long (12 pixels wide per character)
  FrontDisplay.setCursor(1,18*3+6);
  FrontDisplay.println(" DOWN to move down" );
    // Line 6: each line is 18 pixels high, 20 characters long (12 pixels wide per character)
    // Line 7: each line is 18 pixels high, 20 characters long (12 pixels wide per character)
    // Line 8: each line is 18 pixels high, 20 characters long (12 pixels wide per character)
  FrontDisplay.setCursor(1,18*6+6);
  FrontDisplay.println("< Hold to RETURN" );

  delay(1000); // This gives time to make sure that the button has been released.

  int revolution_count = 0;

  while( digitalRead(BT_PIN_C) ) { // Exit the loop if the button is pressed again
    // Set Pumping of Syringe
    stepper.setSpeedInStepsPerSecond(6400);
    stepper.setAccelerationInStepsPerSecondPerSecond(6400);

    if (!digitalRead(BT_PIN_A)){
      if(DEBUG) Serial.println(":: Moving Syringe Head UP 1 rev (pumping in)");
      stepper.moveRelativeInSteps(-3200);
      revolution_count++;
      //delay(100);
    }
    if (!digitalRead(BT_PIN_B)){
      if(DEBUG) Serial.println(":: Moving Syringe Head DOWN 1 rev (pumping out)");
      stepper.moveRelativeInSteps(3200);
      revolution_count--;
      //delay(100);
    }

    // Dynamic Display Line
      // Line 6: each line is 18 pixels high, 20 characters long (12 pixels wide per character)
    FrontDisplay.setCursor(1,18*4+6);
    FrontDisplay.print(" UP#REV: " );
    FrontDisplay.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
    FrontDisplay.println( revolution_count );

  } // while button not pressed, continue to run the code block
}

void action22_calibrate_pH() {
  // Monitor pH and Temperature
  
    // Static Line
    // Line 1: each line is 18 pixels high, 20 characters long (12 pixels wide per character)
  FrontDisplay.fillScreen(ST77XX_BLACK);
  FrontDisplay.setTextSize(2);
  FrontDisplay.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
  FrontDisplay.setCursor(0,0);
  FrontDisplay.println(Menu_display.values[1]);
    // Line 2: each line is 18 pixels high, 20 characters long (12 pixels wide per character)
  FrontDisplay.setCursor(0,18*1+6);
  FrontDisplay.print("_______");
  FrontDisplay.setTextColor(ST77XX_BLACK, ST77XX_WHITE); // 'inverted' text
  FrontDisplay.print(" DATA ");
  FrontDisplay.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
  FrontDisplay.println("_______");

  delay(1000); // This gives time to make sure that the button has been released.

  while( digitalRead(BT_PIN_C) ) { // Exit the loop if the button is pressed again
    // Getting current DateTime: YYYY-MM-DD HH:MM:SS
    DateTime now = rtc.now();

    sprintf(dateTimeBuffer, "%04u-%02u-%02u ", now.year(), now.month(), now.day());
    timeString = dateTimeBuffer;
    sprintf(dateTimeBuffer, "%02u:%02u:%02u,\t", now.hour(), now.minute(), now.second());
    timeString += dateTimeBuffer;
    
    // Setting up dataStrings
    dataString1 = String(id);
    dataString1 += ",\t";
    
    // Acquire pH Probe Data
    if(DEBUG) Serial.println(":: Acquired pH data.");
    pH_value = get_AtlasEZOProbe_I2C_Data(pH_address);
    dataString1 += String(pH_value,3);
    dataString1 += ",\t";
  
    if(DEBUG) Serial.println(":: Acquired RTD data.");
    RTD_value = get_AtlasEZOProbe_I2C_Data(RTD_address);
    dataString1 += String(RTD_value,3);
    //dataString1 += ",\t";
  
    Serial.print(timeString);
    Serial.println(dataString1);
    
    // Front Display Update
      // Line 4: each line is 18 pixels high, 20 characters long (12 pixels wide per character)
    FrontDisplay.setCursor(1,18*2+6);
    FrontDisplay.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
    FrontDisplay.print( "#" );
    FrontDisplay.print( String(id) );
    FrontDisplay.print( " " );
    FrontDisplay.println( timeString.substring(5,19) );
      // Line 5: each line is 18 pixels high, 20 characters long (12 pixels wide per character)
    FrontDisplay.setCursor(1,18*3+6);
    FrontDisplay.print( " pH    = " );
    FrontDisplay.println( String(pH_value,3) );
      // Line 6: each line is 18 pixels high, 20 characters long (12 pixels wide per character)
    FrontDisplay.setCursor(1,18*4+6);    
    FrontDisplay.print( " T(lq) = " );
    FrontDisplay.print( String(RTD_value,3) );
    FrontDisplay.println( " C" );
      // Line 7: each line is 18 pixels high, 20 characters long (12 pixels wide per character)
    FrontDisplay.setCursor(1,18*5+6);
      // Line 8: each line is 18 pixels high, 20 characters long (12 pixels wide per character)
      // Line 9: each line is 18 pixels high, 20 characters long (12 pixels wide per character)
      // Line 10: each line is 18 pixels high, 20 characters long (12 pixels wide per character)
      // Line 11: each line is 18 pixels high, 20 characters long (12 pixels wide per character)
      // Line 12: each line is 18 pixels high, 20 characters long (12 pixels wide per character)
      // Line 13: each line is 18 pixels high, 20 characters long (12 pixels wide per character)
    FrontDisplay.setCursor(1,18*11+6);
    FrontDisplay.println("< Hold to RETURN" );

    id ++;           // increase one ID next iteration

    // Button combinations for pH meter calibration
    if (!digitalRead(BT_PIN_A) && !digitalRead(BT_PIN_B)){
      comm_AtlasEZOProbe_I2C_Data(pH_address, "Cal,?");
      delay(100);
      while(!digitalRead(BT_PIN_A) && !digitalRead(BT_PIN_B));  // Ignore button holds
    }
    if (!digitalRead(BT_PIN_B) && !digitalRead(BT_PIN_C)){
      comm_AtlasEZOProbe_I2C_Data(pH_address, "Slope,?");
      delay(100);
      while(!digitalRead(BT_PIN_B) && !digitalRead(BT_PIN_C));  // Ignore button holds
    }
    if (!digitalRead(BT_PIN_A) && !digitalRead(BT_PIN_C)){
      comm_AtlasEZOProbe_I2C_Data(pH_address, "Status");
      delay(100);
      while(!digitalRead(BT_PIN_A) && !digitalRead(BT_PIN_C));  // Ignore button holds
    }
    
    delay(pH_INTERVAL*1000);
  } // while button not pressed, continue to run the code block to update data on the screen
}

void action23_execute_titration() {
  // Monitor necessary sensory data while moving the pump for titration
  if(DEBUG) Serial.println(":: Begin logging sensory data.");
  
    // Line 1: each line is 18 pixels high, 20 characters long (12 pixels wide per character)
  FrontDisplay.fillScreen(ST77XX_BLACK);
  FrontDisplay.setTextSize(2);
  FrontDisplay.setTextColor(ST77XX_WHITE);
  FrontDisplay.setCursor(0,0);
  FrontDisplay.println(Menu_display.values[2]);
  //FrontDisplay.display();
    // Line 2: each line is 18 pixels high, 20 characters long (12 pixels wide per character)
  FrontDisplay.setCursor(0,18*1);
  FrontDisplay.print("____");
  FrontDisplay.setTextColor(ST77XX_BLACK, ST77XX_WHITE); // 'inverted' text
  FrontDisplay.print(" TITRATION ");
  FrontDisplay.setTextColor(ST77XX_WHITE);
  FrontDisplay.println("_____");
  
  delay(1000); // This gives time to make sure that the button has been released.

  while( digitalRead(BT_PIN_C) ) { // Exit the loop if the button is pressed again
    // Getting current DateTime: YYYY-MM-DD HH:MM:SS
    DateTime now = rtc.now();

    sprintf(dateTimeBuffer, "%04u-%02u-%02u ", now.year(), now.month(), now.day());
    timeString = dateTimeBuffer;
    sprintf(dateTimeBuffer, "%02u:%02u:%02u,\t", now.hour(), now.minute(), now.second());
    timeString += dateTimeBuffer;
    
    // Setting up dataStrings
    dataString1 = String(id);
    dataString1 += ",\t";
    
    // Acquire pH Probe Data
    if(DEBUG) Serial.println(":: Acquired pH data.");
    pH_value = get_AtlasEZOProbe_I2C_Data(pH_address);
    dataString1 += String(pH_value,3);
    dataString1 += ",\t";
  
    if(DEBUG) Serial.println(":: Acquired RTD data.");
    RTD_value = get_AtlasEZOProbe_I2C_Data(RTD_address);
    dataString1 += String(RTD_value,3);
    dataString1 += ",\t";
    
    if(DEBUG) Serial.println(":: Acquired BME280 sensory data.");
    T_value = bme.readTemperature();
    H_value = bme.readHumidity();
    P_value = bme.readPressure();
    dataString2  = String(T_value,3);
    dataString2 += ",\t";
    dataString2 += String(H_value,3);
    dataString2 += ",\t";
    dataString2 += String(P_value/100,3);
    dataString2 += ",\t";
  
    Serial.print(timeString);
    Serial.print(dataString1);
    Serial.println(dataString2);
    
    // Front Display Update
      // Line 1: each line is 18 pixels high, 20 characters long (12 pixels wide per character)
    FrontDisplay.fillScreen(ST77XX_BLACK);
    FrontDisplay.setTextSize(2);
    FrontDisplay.setTextColor(ST77XX_WHITE);
    FrontDisplay.setCursor(0,0);
    FrontDisplay.println(Menu_display.values[0]);
      // Line 2: each line is 18 pixels high, 20 characters long (12 pixels wide per character)
    FrontDisplay.setCursor(0,18*1);
    FrontDisplay.print("_____");
    FrontDisplay.setTextColor(ST77XX_BLACK, ST77XX_WHITE); // 'inverted' text
    FrontDisplay.print(" TITRATION ");
    FrontDisplay.setTextColor(ST77XX_WHITE);
    FrontDisplay.println("_____");
      // Line 3: each line is 18 pixels high, 20 characters long (12 pixels wide per character)
      // Line 4: each line is 18 pixels high, 20 characters long (12 pixels wide per character)
    FrontDisplay.setCursor(1,18*2+6);
    FrontDisplay.setTextColor(ST77XX_WHITE);
    FrontDisplay.print( "#" );
    FrontDisplay.print( String(id) );
    FrontDisplay.print( " " );
    FrontDisplay.println( timeString.substring(5,19) );
    // Line 5: each line is 18 pixels high, 20 characters long (12 pixels wide per character)
    FrontDisplay.setCursor(1,18*3+6);
    FrontDisplay.print( " pH=" );
    FrontDisplay.print( String(pH_value,3) );
    FrontDisplay.print( ",   T=" );
    FrontDisplay.print( String(RTD_value,3) );
    FrontDisplay.print( " C" );
      // Line 6: each line is 18 pixels high, 20 characters long (12 pixels wide per character)
    FrontDisplay.setCursor(1,18*4+6);
    FrontDisplay.print(" T="); FrontDisplay.print(T_value,3); FrontDisplay.print  (" C,"); 
    FrontDisplay.print(" H="); FrontDisplay.print(H_value,3); FrontDisplay.println(" %");
      // Line 7: each line is 18 pixels high, 20 characters long (12 pixels wide per character)
    FrontDisplay.setCursor(1,18*5+6);
    FrontDisplay.print(" P="); FrontDisplay.print(P_value/100,3); FrontDisplay.println(" mBar" );
      // Line 8: each line is 18 pixels high, 20 characters long (12 pixels wide per character)
    FrontDisplay.setCursor(1,18*6+6);
    FrontDisplay.println("< Hold to RETURN" );

    id ++;           // increase one ID next iteration
    
    delay(LOG_INTERVAL*1000);

    // Set Pumping of Syringe
    stepper.setSpeedInStepsPerSecond(1600);
    stepper.setAccelerationInStepsPerSecondPerSecond(1600);
  
    if(DEBUG) Serial.println(":: Moving Syringe Head DOWN 1 rev (pumping out)");
    stepper.moveRelativeInSteps(3200);
    delay(2000);
  
    if(DEBUG) Serial.println(":: Moving Syringe Head UP 1 rev (pumping in)");
    stepper.moveRelativeInSteps(-3200);
    delay(2000);
    
  } // while button not pressed, continue to run the code block  
}

void action24_custom() {
  // Custom
  if(DEBUG) Serial.println(":: Begin logging sensory data.");
  
    // Line 1: each line is 18 pixels high, 20 characters long (12 pixels wide per character)
  FrontDisplay.fillScreen(ST77XX_BLACK);
  FrontDisplay.setTextSize(2);
  FrontDisplay.setTextColor(ST77XX_WHITE);
  FrontDisplay.setCursor(0,0);
  FrontDisplay.println(" Custom Log ");
}

void action3_log_sensors() {
  // Monitor/log all sensory data continuously until the next button press
  if(DEBUG) Serial.println(":: Begin logging sensory data.");
  
    // Static Line
    // Line 1: each line is 18 pixels high, 20 characters long (12 pixels wide per character)
  FrontDisplay.fillScreen(ST77XX_BLACK);
  FrontDisplay.setTextSize(2); //18 x 12 pixels per character
  FrontDisplay.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
  FrontDisplay.setCursor(0,0);
  FrontDisplay.println(Menu_display.values[2]);
    // Line 2: each line is 18 pixels high, 20 characters long (12 pixels wide per character)
  FrontDisplay.setCursor(0,18*1+6);
  FrontDisplay.print("_______");
  FrontDisplay.setTextColor(ST77XX_BLACK, ST77XX_WHITE); // 'inverted' text
  FrontDisplay.print(" DATA ");
  FrontDisplay.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
  FrontDisplay.println("_______");

  delay(1000); // This gives time to make sure that the button has been released.

  while( digitalRead(BT_PIN_C) ) { // Exit the loop if the button is pressed again
    // Getting current DateTime: YYYY-MM-DD HH:MM:SS
    DateTime now = rtc.now();

    sprintf(dateTimeBuffer, "%04u-%02u-%02u ", now.year(), now.month(), now.day());
    timeString = dateTimeBuffer;
    sprintf(dateTimeBuffer, "%02u:%02u:%02u,\t", now.hour(), now.minute(), now.second());
    timeString += dateTimeBuffer;
    
    // Setting up dataStrings
    dataString1 = String(id);
    dataString1 += ",\t";
    
    // Acquire pH Probe Data
    if(DEBUG) Serial.println(":: Acquired pH data.");
    pH_value = get_AtlasEZOProbe_I2C_Data(pH_address);
    dataString1 += String(pH_value,3);
    dataString1 += ",\t";
  
    if(DEBUG) Serial.println(":: Acquired RTD data.");
    RTD_value = get_AtlasEZOProbe_I2C_Data(RTD_address);
    dataString1 += String(RTD_value,3);
    dataString1 += ",\t";
    
    if(DEBUG) Serial.println(":: Acquired BME280 sensory data.");
    T_value = bme.readTemperature();
    H_value = bme.readHumidity();
    P_value = bme.readPressure();
    dataString2  = String(T_value,3);
    dataString2 += ",\t";
    dataString2 += String(H_value,3);
    dataString2 += ",\t";
    dataString2 += String(P_value/100,3);
    dataString2 += ",\t";

    Serial.print(timeString);file.print(timeString);
    Serial.print(dataString1);file.print(dataString1);
    Serial.println(dataString2);file.println(dataString2);
    file.flush(); // makes sure that the data is written to file
    
    // Front Display Update
      // Line 4: each line is 18 pixels high, 20 characters long (12 pixels wide per character)
    FrontDisplay.setCursor(1,18*2+6);
    FrontDisplay.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
    FrontDisplay.print( "#" );
    FrontDisplay.print( String(id) );
    FrontDisplay.print( " " );
    FrontDisplay.println( timeString.substring(5,19) );
      // Line 5: each line is 18 pixels high, 20 characters long (12 pixels wide per character)
    FrontDisplay.setCursor(1,18*3+6);
    FrontDisplay.print( " pH    = " );
    FrontDisplay.println( String(pH_value,3) );
      // Line 6: each line is 18 pixels high, 20 characters long (12 pixels wide per character)
    FrontDisplay.setCursor(1,18*4+6);    
    FrontDisplay.print( " T(lq) = " );
    FrontDisplay.print( String(RTD_value,3) );
    FrontDisplay.println( " C" );
      // Line 7: each line is 18 pixels high, 20 characters long (12 pixels wide per character)
    FrontDisplay.setCursor(1,18*5+6);
    FrontDisplay.print( " T(g)  = ");
    FrontDisplay.print(T_value,3);
    FrontDisplay.println(" C");
      // Line 8: each line is 18 pixels high, 20 characters long (12 pixels wide per character)
    FrontDisplay.setCursor(1,18*6+6);
    FrontDisplay.print( " H(g)  = ");
    FrontDisplay.print(H_value,3);
    FrontDisplay.println(" %");
      // Line 9: each line is 18 pixels high, 20 characters long (12 pixels wide per character)
    FrontDisplay.setCursor(1,18*7+6);
    FrontDisplay.print(" P = ");
    FrontDisplay.print(P_value/100,3);
    FrontDisplay.println(" mBar" );
      // Line 10: each line is 18 pixels high, 20 characters long (12 pixels wide per character)
      // Line 11: each line is 18 pixels high, 20 characters long (12 pixels wide per character)
      // Line 12: each line is 18 pixels high, 20 characters long (12 pixels wide per character)
    if (errorSDCard) {
      FrontDisplay.setCursor(36,18*10+6);
      FrontDisplay.setTextColor(ST77XX_RED, ST77XX_BLACK);
      FrontDisplay.print("!SD Card Error!");
    } else {
      FrontDisplay.setCursor(36,18*10+6);
      FrontDisplay.setTextColor(ST77XX_GREEN, ST77XX_BLACK);
      FrontDisplay.print(sd_filename);
    }
      // Line 13: each line is 18 pixels high, 20 characters long (12 pixels wide per character)
    FrontDisplay.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
    FrontDisplay.setCursor(1,18*11+6);
    FrontDisplay.println("< Hold to RETURN" );

    id ++;           // increase one ID next iteration

    delay(LOG_INTERVAL*1000);
  } // while button not pressed, continue to run the code block
}

void action4_uv_light_log() {
  // Monitor UV for long-term timelapse data
  if(DEBUG) Serial.println(":: Begin logging sensory data.");
  
    // Line 1: each line is 18 pixels high, 20 characters long (12 pixels wide per character)
  FrontDisplay.fillScreen(ST77XX_BLACK);
  FrontDisplay.setTextSize(2);
  FrontDisplay.setTextColor(ST77XX_WHITE);
  FrontDisplay.setCursor(0,0);
  FrontDisplay.println(" UV/Light Sensor Log ");
}

void action5_settings() {
  // Settings
  
    // Line 1: each line is 18 pixels high, 20 characters long (12 pixels wide per character)
  FrontDisplay.fillScreen(ST77XX_BLACK);
  FrontDisplay.setTextSize(2);
  FrontDisplay.setTextColor(ST77XX_WHITE);
  FrontDisplay.setCursor(0,0);
  FrontDisplay.println(">Settings            ");
}

double get_AtlasEZOProbe_I2C_Data(int I2C_address) {

  char data[20];       //make a 20 byte character array to hold incoming data from the pH circuit. 
  byte in_char=0;         //used as a 1 byte buffer to store in bound bytes from the pH Circuit.   
  byte i=0;               //counter used for ph_data array. 
  int time_=900;         //used to change the delay needed depending on the command sent to the EZO Class pH Circuit. 

  Wire.beginTransmission(I2C_address);  //call the circuit by its ID number 
  Wire.write('r');              //hard code r to read continually
  Wire.endTransmission();       //end the I2C data transmission. 
  delay(time_);                 //wait the correct amount of time for the circuit to complete its instruction.
  Wire.requestFrom(I2C_address,20,1);  //call the circuit and request 20 bytes (this may be more than we need)
  
  while(Wire.available()) {     //are there bytes to receive
    in_char = Wire.read();      //receive a byte.
    if ((in_char > 31) && (in_char <127)) {  //check if the char is usable (printable)
      data[i]= in_char;      //load this byte into our array.
      i+=1; 
    }
    if(in_char==0) {            //if we see that we have been sent a null command.
      i=0;                      //reset the counter i to 0.
      Wire.endTransmission();   //end the I2C data transmission.
      break;                    //exit the while loop.
    }
  }

  return atof(data);
}

void comm_AtlasEZOProbe_I2C_Data(int I2C_address, const char *str) { // send text command to I2C Channel to control Atlas EZO Chip

  char data[20];          //make a 20 byte character array to hold incoming data from the pH circuit.
  byte in_char = 0;       //used as a 1 byte buffer to store in bound bytes from the pH Circuit.
  byte code = 0;          //used to hold the I2C response code.   
  byte i = 0;             //counter used for ph_data array. 
  int time_ = 900;        //used to change the delay needed depending on the command sent to the EZO Circuit. 

  if (tolower(str[0]) == 'c' || tolower(str[0]) == 'r') time_ = 900; //if a command has been sent to calibrate or take a reading we wait 900ms so that the circuit has time to take the reading.
  else time_ = 300;                                //if any other command has been sent we wait only 300ms.

  Wire.beginTransmission(I2C_address);  //call the circuit by its ID number 
  Wire.write(str);              //hard code r to read continually
  Wire.endTransmission();       //end the I2C data transmission.

  if (strcmp(str, "sleep") != 0) {  //if the command that has been sent is NOT the sleep command, wait the correct amount of time and request data.
                                             //if it is the sleep command, we do nothing. Issuing a sleep command and then requesting data will wake the pH circuit.
    delay(time_);                            //wait the correct amount of time for the circuit to complete its instruction.

    Wire.requestFrom(I2C_address, 20, 1);    //call the circuit and request 20 bytes (this may be more than we need)
    code = Wire.read();                      //the first byte is the response code, we read this separately.

    switch (code) {                    //switch case based on what the response code is.
      case 1:                          //decimal 1.
        Serial.println("Success");     //means the command was successful.
        break;                         //exits the switch case.

      case 2:                          //decimal 2.
        Serial.println("Failed");      //means the command has failed.
        break;                         //exits the switch case.

      case 254:                        //decimal 254.
        Serial.println("Pending");     //means the command has not yet been finished calculating.
        break;                         //exits the switch case.

      case 255:                        //decimal 255.
        Serial.println("No Data");     //means there is no further data to send.
        break;                         //exits the switch case.
    }

    while (Wire.available()) {         //are there bytes to receive.
      in_char = Wire.read();           //receive a byte.
      data[i] = in_char;               //load this byte into our array.
      i += 1;                          //incur the counter for the array element.
      if (in_char == 0) {              //if we see that we have been sent a null command.
        i = 0;                         //reset the counter i to 0.
        Wire.endTransmission();        //end the I2C data transmission.
        break;                         //exit the while loop.
      }
    }

    Serial.println(data);
//    return data;                       //return text string
  }
}

void connectToWebSocketServer() {
  Serial.println(":: Connecting to webSocket server...");
  boolean error = webSocket.begin();   // attempt to connect
  if (error) {
    Serial.println(":: Cannot connect");
  } else {
    Serial.println(":: Connected.");
  }
}
