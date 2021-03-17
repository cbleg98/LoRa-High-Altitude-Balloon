//Feather Receiver Code
//Author: Jonathan Chen
//Modifications by: Cameron Blegen

//Used for the LoRa 900 MHz Radio Feather M0 being used as a receiver. 
//Make sure to plug the uFl cable into an antenna and the usb port into a computer to save data.
#include <SPI.h>
#include <Wire.h>
#include <RH_RF95.h>
#include <Adafruit_SSD1306.h>

//from MSP430 firmware
#define PACKET_LEN 0x79

//for feather m0 RFM9x
#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 3

// Change to 434.0 or other frequency, must match RX's freq!
#define RF95_FREQ 912.0

#define RF95_TXPOWER 20
#define RF95_SPREADINGFACTOR 10
#define RF95_SIGNALBW 500000 //doesn't do anything
#define THISADDRESS 0xFC
#define ACCEPTEDADDRESS 0xFB

// Blink on receipt
#define LED 13

#define BUTTON_A  9
#define BUTTON_B  6
#define BUTTON_C  5

RH_RF95 rf95(RFM95_CS, RFM95_INT); // Singleton instance of the radio driver
Adafruit_SSD1306 display = Adafruit_SSD1306(128, 32, &Wire);

//store telemtry data
  uint8_t hour;             ///< GMT hours
  uint8_t minute;           ///< GMT minutes
  uint8_t seconds;          ///< GMT seconds
  uint8_t year;             ///< GMT year
  uint8_t month;            ///< GMT month
  uint8_t day;              ///< GMT day

  float latitude = 0;   ///< Floating point latitude value in degrees/minutes as received from the GPS (DDMM.MMMM)
  float longitude = 0;  ///< Floating point longitude value in degrees/minutes as received from the GPS (DDDMM.MMMM)

  float altitude;           ///< Altitude in meters above MSL
  float spd;                ///< Current speed over ground in knots
  float angle;              ///< Course in degrees from true north

  char lat;                 ///< N/S
  char lon;                 ///< E/W

  boolean fix = 0;
  uint8_t fixquality = 0;   ///< Fix quality (0, 1, 2 = Invalid, GPS, DGPS)
  uint8_t satellites;       ///< Number of satellites in use

  float OWT = 0;            ///< OneWire Temperature Reading(degrees C)
  float AT = 0;             ///< Analog Temperature Reading(degrees C)
  float RBT = 0;            ///< Red Board Temperature Reading(degrees C)

  float X = 0;            ///< Acceleration X-Axis
  float Y = 0;            ///< Acceleration Y-Axis
  float Z = 0;            ///< Acceleration Z-Axis

bool dataComplete = false;  ///< Flags when all values have been read to update OLED
int displayMode = 0;        ///< Screen display mode(0 = RSSI, Time, Date, Fix, Quality, Satellites;
                            /// 1 = Lat, Lon, Speed, Angle, Altitude; 2 = OWT, AT, RBT, Acceleration)
int timeOfLastSignal = 0;   ///< Helps to indicate how long ago the last transmission was
uint32_t timer = millis();  ///< Keeps track of when to refresh screen

// for message from radio
uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
uint8_t len = sizeof(buf);
  
void setup()
{
  pinMode(LED, OUTPUT);
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

  //--Serial Setup--//
  Serial.begin(115200);
  //while (!Serial) {
  //  delay(1);
  //}
  delay(100);
  
  //Serial.println("Feather LoRa RX Test!");
  //--END Serial Setup--//

  //--Radio Setup--//
  // manual reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  while (!rf95.init()) {
    //Serial.println("LoRa radio init failed");
    //Serial.println("Uncomment '#define SERIAL_DEBUG' in RH_RF95.cpp for detailed debug info");
    while (1);
  }
  //Serial.println("LoRa radio init OK!");

  // Defaults after init are 434.0MHz, modulation GFSK_Rb250Fd250, +13dbM
  if (!rf95.setFrequency(RF95_FREQ)) {
    //Serial.print("setFrequency failed");
    while (1);
  }
  //Serial.print("Set Freq to: "); Serial.println(RF95_FREQ);
  rf95.setSpreadingFactor(RF95_SPREADINGFACTOR); //sets spreading factor
  //rf95.setSignalBandwidth(RF95_SIGNALBW); //sets bandwidth

  // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on

  // The default transmitter power is 13dBm, using PA_BOOST.
  // If you are using RFM95/96/97/98 modules which uses the PA_BOOST transmitter pin, then
  // you can set transmitter powers from 5 to 23 dBm:
  rf95.setTxPower(RF95_TXPOWER, false);
  rf95.setPromiscuous(false);
  rf95.setThisAddress(THISADDRESS); //sets unique address for radio so it doesn't get unintended data
  rf95.setHeaderTo(ACCEPTEDADDRESS);
  rf95.setHeaderFrom(THISADDRESS);

  //--END Radio Setup--//
  
  //--OLED Setup--//
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C); // Address 0x3C for 128x32
  //Serial.println("OLED begun");
  display.display();

  delay(1000);
 
  // Clear the buffer.
  display.clearDisplay();
  display.display();

  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0,0);

  pinMode(BUTTON_A, INPUT_PULLUP);
  pinMode(BUTTON_B, INPUT_PULLUP);
  pinMode(BUTTON_C, INPUT_PULLUP);

  //--END OLED Setup--//
}

void loop()
{
  unsigned int i;
  //--Serial Loop--//--send message
  /*char input[RH_RF95_MAX_MESSAGE_LEN];
  if (Serial.available() > 0) {
    digitalWrite(LED, HIGH);
    int bytesRead = Serial.readBytes(input, RH_RF95_MAX_MESSAGE_LEN);
    //--Radio Loop--//
    rf95.send((uint8_t*)input, bytesRead);
    digitalWrite(LED, LOW);
    //--END Radio Loop--//
  }*/
  //--END Serial Loop--//
  
  //--Radio Loop--//--receive message
  if (rf95.available())
  {
    // Store new message
    //uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    //uint8_t len = sizeof(buf);

    if (rf95.recv(buf, &len))
    {
      if (rf95.headerFrom() == ACCEPTEDADDRESS) {
        //make sure the from radio has the correct address
        digitalWrite(LED, HIGH);
        Serial.print('['); //start character
        for(i=0;i<len;i++){
          Serial.print(buf[i]);
          Serial.print(','); 
        }
        Serial.print("RSSI");
        Serial.print(rf95.lastRssi(), DEC);
        Serial.print(',');
        Serial.print("SNR");
        Serial.print(rf95.lastSNR(), DEC);
        Serial.print(']');
        Serial.println("");

        //RH_RF95::printBuffer("", buf, len);
        //RH_RF95::printBuffer("Received: ", buf, len);
        //Serial.print("Got: ");
        //Serial.println((char*)buf);
        /*if (buf[0] == 'Z') {    //newline for readability in serial
          Serial.println();
        }
        if (buf[0] != 'Z') {
          Serial.print("RSSI:");
          Serial.println(rf95.lastRssi(), DEC);
          Serial.print("SNR: ");
          Serial.println(rf95.lastSNR());
        }*/
        digitalWrite(LED, LOW);
  
        //processData((char*)buf);
        timeOfLastSignal = millis();
      //}
    }
    else
    {
      //Serial.println("Receive failed");
    }
  }
}
  //--END Radio Loop--//
  
  //--OLED Display Loop--//
  if(!digitalRead(BUTTON_A)) {
    displayMode = 0;
    updateScreen();
  }
  if(!digitalRead(BUTTON_B)) {
    displayMode = 1;
    updateScreen();
  }
  if(!digitalRead(BUTTON_C)) {
    displayMode = 2;
    updateScreen();
  }

  //if millis() wraps around, reset timer
  if (timer > millis()) timer = millis();
  //refresh screen every ~2 seconds
  if (millis() - timer > 2000) {
    timer = millis();
    updateScreen();
  }
  //if (dataComplete) {
  //  dataComplete = false;
  //}
  //--END OLED DisplayLoop--//
}

/*//processes buffer and assigns to the appropriate variables
void processData(char* buf) {
  char* value;
  //decision tree for which variable to assign
  if (buf[0] == 'T') { //time
    value = (char*)strtok(buf, ":");
    value = (char*)strtok(NULL, ":"); //skip time label
    int i = 0; //index hour, minute, seconds
    while (value != NULL) {
      if (i == 0) {
        hour = atoi(value);
      }
      else if (i == 1) {
        minute = atoi(value);
      }
      else if (i == 2) {
        seconds = atoi(value);
      }
      value = strtok (NULL, ":");
      i += 1;
    }
  }
  else if (buf[0] == 'D') { //date
    value = (char*)strtok(buf, ":");
    value = (char*)strtok(NULL, ":");
    int i = 0;
    while (value != NULL) {
      if (i == 0) {
        day = atoi(value);
      }
      else if (i == 1) {
        month = atoi(value);
      }
      else if (i == 2) {
        year = atoi(value);
      }
      value = strtok (NULL, ":");
      i += 1;
    }
  }
  else if (buf[0] == 'F') { //fix/quality data
    value = (char*)strtok(buf, ":");
    value = (char*)strtok(NULL, ":"); //fix
    fix = (int)(value[0]-'0');
    value = (char*)strtok(NULL, ":"); //quality
    fixquality = atoi(value);
  }
  else if (buf[0] == 'L') { //GPS location data
    if (buf[1] == 'a') { //latitude
      value = (char*)strtok(buf, ":");
      value = (char*)strtok(NULL, ":");
      latitude = atof(value);
      value = (char*)strtok(NULL, ":");
      lat = value[0];
    }
    else if (buf[1] == 'o') { //longitude
      value = (char*)strtok(buf, ":");
      value = (char*)strtok(NULL, ":");
      longitude = atof(value);
      value = (char*)strtok(NULL, ":");
      lon = value[0];
    }
  }
  else if (buf[0] == 'S') {//GPS speed and satellites
    if (buf[1] == 'p') {
      value = (char*)strtok(buf, ":");
      value = (char*)strtok(NULL, ":");
      spd = atof(value);
    }
    else if (buf[1] == 'a') {
      value = (char*)strtok(buf, ":");
      value = (char*)strtok(NULL, ":");
      satellites = atoi(value);
    }
  }
  else if (buf[0] == 'A') {//Angle, Altitude, and Analog sensor
    if (buf[1] == 'n') {
      if (buf[2] == 'g') {//Angle
        value = (char*)strtok(buf, ":");
        value = (char*)strtok(NULL, ":");
        angle = atof(value);
      }
      else if (buf[2] == 'a') {//Analog
        value = (char*)strtok(buf, ":");
        value = (char*)strtok(NULL, ":");
        AT = atof(value);
      }
    }
    else if (buf[1] == 'l') {//Altitude
      value = (char*)strtok(buf, ":");
      value = (char*)strtok(NULL, ":");
      altitude = atof(value);
    }
  }
  else if (buf[0] == 'O') {//OneWire sensor
    value = (char*)strtok(buf, ":");
    value = (char*)strtok(NULL, ":");
    OWT = atof(value);
  }
  else if (buf[0] == 'R') {//Red Board sensor
    value = (char*)strtok(buf, ":");
    value = (char*)strtok(NULL, ":");
    RBT = atof(value);
  }
  else if (buf[0] == 'X') {//Accelerometer X
    value = (char*)strtok(buf, ":");
    value = (char*)strtok(NULL, ":");
    X = atof(value);
  }
  else if (buf[0] == 'Y') {//Accelerometer Y
    value = (char*)strtok(buf, ":");
    value = (char*)strtok(NULL, ":");
    Y = atof(value);
  }
  else if (buf[0] == 'Z') {//Accelerometer Z
    value = (char*)strtok(buf, ":");
    value = (char*)strtok(NULL, ":");
    Z = atof(value);
    //signal end of transmission
    dataComplete = true;
  }
}
*/
void updateScreen() {
  display.clearDisplay();
  display.setCursor(0,0);
  switch(displayMode) {
    case 0:
    updateScreen0(); break;
    case 1:
    updateScreen1(); break;
    case 2:
    updateScreen2(); break;
    default: break;//Serial.println("You messed up your display modes somehow");
  }
  display.display();
}

void updateScreen0() {
  int i;
  //RSSI, Time, Date, Fix, Quality, Satellites
  display.print("RSSI:");
  display.print(rf95.lastRssi(), DEC);
  display.println(" TSLF:" + (String)((millis()-timeOfLastSignal)/1000));
  //format time to have 0's for single digit numbers
  //SNR
  display.print("SNR:");
  display.println(rf95.lastSNR());
  display.println("");
  display.print("Buffer:");
  display.print((char*)buf);    

}

void updateScreen1() {
  int i;
  display.print("Data Buf:");
        for(i=0;i<20;i++){
          display.print(buf[i]);
          display.print(',');
        }
  display.println("");
  display.println("");
  display.print("Data says: The earth is flat");
  }
  
void updateScreen2() {
  int i;
  display.print("GPS Data:");
        for(i=20;i<len;i++){
          display.print((char)buf[i]);
        }
  }
  
