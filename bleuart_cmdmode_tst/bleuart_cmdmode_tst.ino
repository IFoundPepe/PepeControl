#include <Adafruit_NeoPixel.h>

/*********************************************************************
 This is an example for our nRF51822 based Bluefruit LE modules

 Pick one up today in the adafruit shop!

 Adafruit invests time and resources providing this open source code,
 please support Adafruit and open-source hardware by purchasing
 products from Adafruit!

 MIT license, check LICENSE for more information
 All text above, and the splash screen below must be included in
 any redistribution
*********************************************************************/

#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include <Adafruit_VS1053.h>

#define PIN 19
#define LASERPIN 18

Adafruit_NeoPixel strip = Adafruit_NeoPixel(2, PIN, NEO_GRB + NEO_KHZ800);

// These are the pins used
#define VS1053_RESET   -1     // VS1053 reset pin (not used!)

#if not defined (_VARIANT_ARDUINO_DUE_X_) && not defined (_VARIANT_ARDUINO_ZERO_)
  #include <SoftwareSerial.h>
#endif

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"

#include "BluefruitConfig.h"

// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
// you can also call it with a different address you want
//Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x41);

// Feather M0 or 32u4
  #define VS1053_CS       6     // VS1053 chip select pin (output)
  #define VS1053_DCS     10     // VS1053 Data/command select pin (output)
  #define CARDCS          5     // Card chip select pin
  // DREQ should be an Int pin *if possible* (not possible on 32u4)
  #define VS1053_DREQ     9     // VS1053 Data request, ideally an Interrupt pin

Adafruit_VS1053_FilePlayer musicPlayer = Adafruit_VS1053_FilePlayer(VS1053_RESET, VS1053_CS, VS1053_DCS, VS1053_DREQ, CARDCS);

// Depending on your servo make, the pulse width min and max may vary, you 
// want these to be as small/large as possible without hitting the hard stop
// for max range. You'll have to tweak them as necessary to match the servos you
// have!
#define SERVOMIN  215 // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  465 // this is the 'maximum' pulse length count (out of 4096)
#define MAXFILELENGTH 100

#define FLAPMIN   325
#define FLAPMAX   600

#define TURNMIN 215
#define TURNMAX 500

#define RIGHTFLAPMIN 160
#define RIGHTFLAPMAX 330

#define LEFTFLAPMIN 320
#define LEFTFLAPMAX 490

#define BLINKLEFTMIN 125
#define BLINKLEFTMAX 280

#define BLINKRIGHTMIN 340
#define BLINKRIGHTMAX 480

#define TAILMIN 200
#define TAILMAX 400

bool firstTime = true;

/*=========================================================================
    APPLICATION SETTINGS

    FACTORYRESET_ENABLE       Perform a factory reset when running this sketch
   
                              Enabling this will put your Bluefruit LE module
                              in a 'known good' state and clear any config
                              data set in previous sketches or projects, so
                              running this at least once is a good idea.
   
                              When deploying your project, however, you will
                              want to disable factory reset by setting this
                              value to 0.  If you are making changes to your 
                              Bluefruit LE device via AT commands, and those
                              changes aren't persisting across resets, this
                              is the reason why.  Factory reset will erase
                              the non-volatile memory where config data is
                              stored, setting it back to factory default
                              values.
       
                              Some sketches that require you to bond to a
                              central device (HID mouse, keyboard, etc.)
                              won't work at all with this feature enabled
                              since the factory reset will clear all of the
                              bonding data stored on the chip, meaning the
                              central device won't be able to reconnect.
    MINIMUM_FIRMWARE_VERSION  Minimum firmware version to have some new features
    MODE_LED_BEHAVIOUR        LED activity, valid options are
                              "DISABLE" or "MODE" or "BLEUART" or
                              "HWUART"  or "SPI"  or "MANUAL"
    -----------------------------------------------------------------------*/
    #define FACTORYRESET_ENABLE         0
    #define MINIMUM_FIRMWARE_VERSION    "0.6.6"
    #define MODE_LED_BEHAVIOUR          "MODE"
/*=========================================================================*/

// GLOBAL VARIABLES CUZ I IZ BAD CODER!!!! At least I initialized them...
int look = 128;
int turn = 0;
int flapLeft = 0;
int flapRight = 0;
int blinkLeft = 0;
int blinkRight = 0;
int tail = 0;
int key = 0;
int laser = 0;
int eyeRight = 0;
int eyeLeft = 0;
int tweet = 0;

int laser_count = 0;

char* soundArray[MAXFILELENGTH]={"Pepe1.mp3", "Pepe2.mp3", "Pepe3.mp3", "Pepe4.mp3", "Pepe5.mp3",
"Pepe6.mp3","Pepe7.mp3", "Pepe8.mp3", "Pepe9.mp3", "Pepe10.mp3","whistle.mp3"};

// Create the bluefruit object, either software serial...uncomment these lines
/*
SoftwareSerial bluefruitSS = SoftwareSerial(BLUEFRUIT_SWUART_TXD_PIN, BLUEFRUIT_SWUART_RXD_PIN);

Adafruit_BluefruitLE_UART ble(bluefruitSS, BLUEFRUIT_UART_MODE_PIN,
                      BLUEFRUIT_UART_CTS_PIN, BLUEFRUIT_UART_RTS_PIN);
*/

/* ...or hardware serial, which does not need the RTS/CTS pins. Uncomment this line */
// Adafruit_BluefruitLE_UART ble(Serial1, BLUEFRUIT_UART_MODE_PIN);

/* ...hardware SPI, using SCK/MOSI/MISO hardware SPI pins and then user selected CS/IRQ/RST */
Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

/* ...software SPI, using SCK/MOSI/MISO user-defined SPI pins and then user selected CS/IRQ/RST */
//Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_SCK, BLUEFRUIT_SPI_MISO,
//                             BLUEFRUIT_SPI_MOSI, BLUEFRUIT_SPI_CS,
//                             BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

// A small helper
void error(const __FlashStringHelper*err) {
  Serial.println(err);
  while (1);
}

/**************************************************************************/
/*!
    @brief  Sets up the HW an the BLE module (this function is called
            automatically on startup)
*/
/**************************************************************************/
void setup(void)
{
  uint32_t color  = 0x00ffbb; // aqua
  strip.begin();
  strip.show(); // Initialize all pixels to 'off'
  strip.setPixelColor(0,color);
  strip.setPixelColor(1,color);  
  strip.show();
  
  pinMode(LASERPIN,OUTPUT);
  digitalWrite(LASERPIN,HIGH);
  pinMode(LASERPIN-1,OUTPUT);
  digitalWrite(LASERPIN-1,HIGH);
  
/*  while (!Serial);  // required for Flora & Micro*/
  delay(500);

  Serial.begin(115200);
  Serial.println(F("Adafruit Bluefruit Command Mode Example"));
  Serial.println(F("---------------------------------------"));

  /* Initialise the module */
  Serial.print(F("Initialising the Bluefruit LE module: "));

  if ( !ble.begin(VERBOSE_MODE) )
  {
    error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
  }
  Serial.println( F("OK!") );

  if ( FACTORYRESET_ENABLE )
  {
    /* Perform a factory reset to make sure everything is in a known state */
    Serial.println(F("Performing a factory reset: "));
    if ( ! ble.factoryReset() ){
      error(F("Couldn't factory reset"));
    }
  }

  /* Disable command echo from Bluefruit */
  ble.echo(false);
  
  Serial.println("Requesting Bluefruit info:");
  /* Print Bluefruit information */
  ble.info();

  Serial.println(F("Please use Adafruit Bluefruit LE app to connect in UART mode"));
  Serial.println(F("Then Enter characters to send to Bluefruit"));
  Serial.println();

  ble.verbose(false);  // debug info is a little annoying after this point!

  /* Wait for connection */
  while (! ble.isConnected()) {
      delay(500);
  }

  // LED Activity command is only supported from 0.6.6
  if ( ble.isVersionAtLeast(MINIMUM_FIRMWARE_VERSION) )
  {
    // Change Mode LED Activity
    Serial.println(F("******************************"));
    Serial.println(F("Change LED activity to " MODE_LED_BEHAVIOUR));
    ble.sendCommandCheckOK("AT+HWModeLED=" MODE_LED_BEHAVIOUR);
    Serial.println(F("******************************"));
  }

  // Servo stuff
  pwm.begin();
  pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz updates

  if (! musicPlayer.begin()) { // initialise the music player
     Serial.println(F("Couldn't find VS1053, do you have the right pins defined?"));
     while (1);
  }
  
  if (!SD.begin(CARDCS)) {
    Serial.println(F("SD failed, or not present"));
    //while (1);  // don't do anything more
  }
  Serial.println("SD OK!");
  
  // list files
  //printDirectory(SD.open("/"), 0);
  
  // Set volume for left, right channels. lower numbers == louder volume!
  musicPlayer.setVolume(1,1);

  //
 
}

/**************************************************************************/
/*!
    @brief  Constantly poll for new command or response data
*/
/**************************************************************************/
void loop(void)
{
  
  // Check for user input
  char inputs[BUFSIZE+1];

  if ( getUserInput(inputs, BUFSIZE) )
  {
    // Send characters to Bluefruit
    Serial.print("[Send] ");
    Serial.println(inputs);

    ble.print("AT+BLEUARTTX=");
    ble.println(inputs);

    // check response stastus
    if (! ble.waitForOK() ) {
      Serial.println(F("Failed to send?"));
    }
  }

  // Check for incoming characters from Bluefruit
  ble.println("AT+BLEUARTRX");
  ble.readline();
  if (strcmp(ble.buffer, "OK") == 0) {
    // no data
    return;
  }
  // Some data was found, its in the buffer
  Serial.print(F("[Recv] "));

//  String tmp = "";
//  
// for(int i = 0; ble.buffer[i] != '%'; i++)
// { 
//    tmp += ble.buffer[i];
// }
//  //parse the data
//  char packet[tmp.length()+1];
//  
//  tmp.toCharArray(packet, tmp.length()+1);
  
  parseDataPacket(ble.buffer);

  //update servo positions based on new data
  if (!firstTime) {
  setServoPositions();
  } else {
    firstTime = false;
  }

  if(tweet > 0 && tweet < (MAXFILELENGTH-1) )
  {
      musicPlayer.setVolume(1,1);
      musicPlayer.playFullFile(soundArray[tweet - 1]);
  }
  
  //Serial.println(tmp);
//  tmp = "";
   /*not mirroring back the text*/
  //ble.print("AT+BLEUARTTX=");
  //ble.println(ble.buffer);

  // check response stastus
  //if (! ble.waitForOK() ) {
  //  Serial.println(F("Failed to send?"));
  //}
  
  ble.waitForOK();

  //if(laser_count++ > 50)
  //{
    //digitalWrite(LASERPIN,HIGH);
    //if(laser_count > 100)
    //{
      //laser_count = 0;
    //}
  //}
  //else
  //{
    //digitalWrite(LASERPIN,LOW);
  //}
  
  //pixels.setPixelColor(0,color);
  //pixels.setPixelColor(1,color);  
  //pixels.show();
  //delay(500);
}

/**************************************************************************/
/*!
    @brief  Checks for user input (via the Serial Monitor)
*/
/**************************************************************************/
bool getUserInput(char buffer[], uint8_t maxSize)
{
  // timeout in 100 milliseconds
  TimeoutTimer timeout(100);

  memset(buffer, 0, maxSize);
  while( (!Serial.available()) && !timeout.expired() ) { delay(1); }

  if ( timeout.expired() ) return false;

  delay(2);
  uint8_t count=0;
  do
  {
    count += Serial.readBytes(buffer+count, maxSize);
    delay(2);
  } while( (count < maxSize) && (Serial.available()) && buffer[count] != '#' );

  return true;
}

bool parseDataPacket(const char* input)
{
  Serial.println(input);
  int i = 0;
  look =0;
  turn = 0;
  flapLeft = 0;
  flapRight = 0;
  blinkLeft = 0;
  blinkRight = 0;
  tail = 0;
  key = 0;
  tweet = 0;
  laser = 0;
  eyeRight = 0;
  eyeLeft = 0;
//  look =  input[i++] | (input[i++] << 8) | (input[i++] << 16) | (input[i++] << 24);
//  turn =  input[i++] | (input[i++] << 8) | (input[i++] << 16) | (input[i++] << 24);
//  flapLeft =  input[i++] | (input[i++] << 8) | (input[i++] << 16) | (input[i++] << 24);
//  flapRight =  input[i++] | (input[i++] << 8) | (input[i++] << 16) | (input[i++] << 24);
//  blinkLeft =  input[i++] | (input[i++] << 8) | (input[i++] << 16) | (input[i++] << 24);
//  blinkRight =  input[i++] | (input[i++] << 8) | (input[i++] << 16) | (input[i++] << 24);
//  tail =  input[i++] | (input[i++] << 8) | (input[i++] << 16) | (input[i++] << 24);
//  key =  input[i++] | (input[i++] << 8) | (input[i++] << 16) | (input[i++] << 24);
//  tweet =  input[i++] | (input[i++] << 8) | (input[i++] << 16) | (input[i++] << 24);


  look =  input[i++];
//  look =  input[i++] | (input[i++] << 8);
  turn =  input[i++];
//  turn =  input[i++] | (input[i++] << 8);
  flapLeft =  input[i++];
  flapRight =  input[i++];
//  flapLeft =  input[i++] | (input[i++] << 8);
//  flapRight =  input[i++] | (input[i++] << 8);
//  blinkLeft =  input[i++] | (input[i++] << 8);
//  blinkRight =  input[i++] | (input[i++] << 8);
  blinkLeft =  input[i++];
  blinkRight =  input[i++];
//  tail =  input[i++] | (input[i++] << 8);
//  key =  input[i++] | (input[i++] << 8);
//  tweet =  input[i++] | (input[i++] << 8);
  tail =  input[i++];
  key =  input[i++];
  tweet =  input[i++];
  eyeRight = input[i++];
  eyeLeft = input[i++];
  laser = input[i++];


  
//  turn =  input[0] + (input[1] << 8) + (input[2] << 16) + (input[3] << 24);
//  turn = (input[0] << 24) + (input[1] << 16) + (input[2] << 8) + input[3];
  Serial.println(look);
  Serial.println(turn);
  Serial.println(flapLeft);
  Serial.println(flapRight);
  Serial.println(blinkLeft);
  Serial.println(blinkRight);
  Serial.println(tail);
  Serial.println(key);
  Serial.println(tweet);
  Serial.println(eyeRight);
  Serial.println(eyeLeft);
  Serial.println(laser);
  
  return true;
}

int pulselengthLook = 0;
int prevPulseLengthLook = 0;

int pulselengthTurn = 0;
int prevPulseLengthTurn = 0;

//int pulseBlinkRight = 0;
//int pulseBlinkRightPrev = 0;
//
//int pulseBlinkLeft = 0;
//int pulseBlinkLeftPrev = 0;

bool setServoPositions()
{  
//  pwm.setPWM(1, 0, turn);
  pwm.setPWM(1, 0, map(flapRight, 0, 90, RIGHTFLAPMIN, RIGHTFLAPMAX));
  pwm.setPWM(2, 0, map(tail, 0, 90, TAILMIN, TAILMAX));
//  pwm.setPWM(4, 0, tweet);
//  pwm.setPWM(5, 0, key);

//  pulselengthLook = map(flapLeft, 0, 180, SERVOMIN, SERVOMAX);
  pwm.setPWM(6, 0, map(flapLeft, 0, 90, LEFTFLAPMIN, LEFTFLAPMAX));
  
  pwm.setPWM(4, 0, map(blinkLeft, 0, 180, BLINKLEFTMIN, BLINKLEFTMAX));
  pwm.setPWM(5, 0, map(blinkRight, 0, 180, BLINKRIGHTMIN, BLINKRIGHTMAX));

  pulselengthLook = map(look, 0, 180, SERVOMIN, SERVOMAX);
  if (pulselengthLook != prevPulseLengthLook ) {
    updateServo(pulselengthLook, prevPulseLengthLook, 0);
  }
  prevPulseLengthLook = pulselengthLook;

  pulselengthTurn = map(turn, 0, 180, TURNMIN, TURNMAX);
  if (pulselengthTurn != prevPulseLengthTurn) {
    updateServo(pulselengthTurn, prevPulseLengthTurn, 3);
  }
  prevPulseLengthTurn = pulselengthTurn;

//  pulseBlinkRight = map(blinkRight, 0, 180, SERVOMIN, SERVOMAX);
//  updateServo(pulseBlinkRight, pulseBlinkRightPrev, 7);
//  pulseBlinkRightPrev = pulseBlinkRight;
//
//  pulseBlinkLeft = map(blinkLeft, 0, 180, SERVOMIN, SERVOMAX);
//  updateServo(pulseBlinkLeft, pulseBlinkLeftPrev, 7);
//  pulseBlinkLeftPrev = pulseBlinkLeft;

  if ( laser >= 1 ) {
    digitalWrite(LASERPIN,HIGH);
    digitalWrite(LASERPIN-1,HIGH);
  }
  else {
    digitalWrite(LASERPIN,LOW);
    digitalWrite(LASERPIN-1,LOW);
  }
  
  return true;
}

void updateServo(int current, int previous, int pin)
{
  if (current > previous) {
    for (uint16_t pulselen = previous; pulselen < current; pulselen++) {
      pwm.setPWM(pin, 0, pulselen);
    }
  }
  else 
  {
    for (uint16_t pulselen = previous; pulselen > current; pulselen--) {
      pwm.setPWM(pin, 0, pulselen); 
    }
  }
}

/// File listing helper
void printDirectory(File dir, int numTabs) {
   while(true) {
     
     File entry =  dir.openNextFile();
     if (! entry) {
       // no more files
       //Serial.println("**nomorefiles**");
       break;
     }
     for (uint8_t i=0; i<numTabs; i++) {
       Serial.print('\t');
     }
     Serial.print(entry.name());
     if (entry.isDirectory()) {
       Serial.println("/");
       printDirectory(entry, numTabs+1);
     } else {
       // files have sizes, directories do not
       Serial.print("\t\t");
       Serial.println(entry.size(), DEC);
     }
     entry.close();
   }
}

void loadSoundArray()
{
  

}

