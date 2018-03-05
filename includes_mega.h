
//==============================================//
// Include the various libraries                //
//==============================================//

#include <PID_v1.h>
#include <PID_AutoTune_v0.h>
#include <MCUFRIEND_kbv.h> // changed this line to to 0x3A, 1, 0x55,      //Interlace Pixel Format [XX]
// Due to incorrect colour scheme using 0x3A, 1, 0x66,      //Interlace Pixel Format [XX]
#include <TouchScreen.h>
#include <SPI.h>
#include <EEPROM.h>
#include <elapsedMillis.h>
#include <Adafruit_MAX31865.h> // https://joshuawoehlke.com/daisy-chaining-spi-arduino-mega-2560/
#include <avr/wdt.h>
#include <Adafruit_GFX.h> // Hardware-specific library
//#include <ILI9488_kbv.h>
//#include <TimerFive.h>
//#include <SD.h> // https://joshuawoehlke.com/daisy-chaining-spi-arduino-mega-2560/
//#include <AltSoftSerial.h>
