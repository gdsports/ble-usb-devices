/*
 * USB Mouse to BLE mouse converter for the Adafruit Bluefruit M0 LE
 */

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

/*
  MIT License

  Copyright (c) 2019 gdsports625@gmail.com

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  SOFTWARE.
*/

#define DEBUG_ON 0

#if DEBUG_ON

/* Serial1 is the Tx/Rx pins on the Feather M0. Connect a USB to serial board
 * such as a CP2104 board to get debug output. When the USB port is USB host
 * mode for the keyboard, it cannot be used for USB serial. In other words,
 * Serial does not work when using USB host mode so use Serial1.
 */

#define DEBUG_SERIAL      Serial1
#define dbbegin(...)      DEBUG_SERIAL.begin(__VA_ARGS__)
#define dbprint(...)      DEBUG_SERIAL.print(__VA_ARGS__)
#define dbprintln(...)    DEBUG_SERIAL.println(__VA_ARGS__)
#define dbavailable(...)  DEBUG_SERIAL.available()
#define dbreadBytes(...)  DEBUG_SERIAL.readBytes(__VA_ARGS__)
#else
#define dbbegin(...)
#define dbprintln(...)
#define dbprint(...)
#define dbavailable(...)
#define dbreadBytes(...)
#endif

#include <Arduino.h>
#include <SPI.h>
#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"

#include "BluefruitConfig.h"

#if SOFTWARE_SERIAL_AVAILABLE
  #include <SoftwareSerial.h>
#endif

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
    -----------------------------------------------------------------------*/
    #define FACTORYRESET_ENABLE         0
    #define MINIMUM_FIRMWARE_VERSION    "0.6.6"
/*=========================================================================*/


// Create the bluefruit object, either software serial...uncomment these lines
/*
SoftwareSerial bluefruitSS = SoftwareSerial(BLUEFRUIT_SWUART_TXD_PIN, BLUEFRUIT_SWUART_RXD_PIN);

Adafruit_BluefruitLE_UART ble(bluefruitSS, BLUEFRUIT_UART_MODE_PIN,
                      BLUEFRUIT_UART_CTS_PIN, BLUEFRUIT_UART_RTS_PIN);
*/

/* ...or hardware serial, which does not need the RTS/CTS pins. Uncomment this line */
// Adafruit_BluefruitLE_UART ble(BLUEFRUIT_HWSERIAL_NAME, BLUEFRUIT_UART_MODE_PIN);

/* ...hardware SPI, using SCK/MOSI/MISO hardware SPI pins and then user selected CS/IRQ/RST */
Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

/* ...software SPI, using SCK/MOSI/MISO user-defined SPI pins and then user selected CS/IRQ/RST */
//Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_SCK, BLUEFRUIT_SPI_MISO,
//                             BLUEFRUIT_SPI_MOSI, BLUEFRUIT_SPI_CS,
//                             BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

// A small helper
void error(const __FlashStringHelper*err) {
  dbprintln(err);
}

/*************************************************************************/
#include <hidboot.h>
#include <usbhub.h>

class MouseRptParser : public MouseReportParser
{
protected:
	void OnMouseMove	(MOUSEINFO *mi);
	void OnLeftButtonUp	(MOUSEINFO *mi);
	void OnLeftButtonDown	(MOUSEINFO *mi);
	void OnRightButtonUp	(MOUSEINFO *mi);
	void OnRightButtonDown	(MOUSEINFO *mi);
	void OnMiddleButtonUp	(MOUSEINFO *mi);
	void OnMiddleButtonDown	(MOUSEINFO *mi);
};
void MouseRptParser::OnMouseMove(MOUSEINFO *mi)
{
  char atcmd[32];
  snprintf(atcmd, sizeof(atcmd), "AT+BLEHIDMOUSEMOVE=%d,%d", mi->dX, mi->dY);
  dbprint(F("Move "));
  dbprintln(atcmd);
  if (! ble.sendCommandCheckOK(atcmd)) {
    error(F("Failed to move"));
  }
};
void MouseRptParser::OnLeftButtonUp	(MOUSEINFO *mi)
{
  dbprintln(F("L Butt Up"));
  if (! ble.sendCommandCheckOK(F( "AT+BLEHIDMOUSEBUTTON=0"  ))) {
    error(F("Failed to L Butt Up"));
  }
};
void MouseRptParser::OnLeftButtonDown	(MOUSEINFO *mi)
{
  dbprintln(F("L Butt Dn"));
  if (! ble.sendCommandCheckOK(F( "AT+BLEHIDMOUSEBUTTON=L,press"  ))) {
    error(F("Failed to L Butt Dn"));
  }
};
void MouseRptParser::OnRightButtonUp	(MOUSEINFO *mi)
{
  dbprintln(F("R Butt Up"));
  if (! ble.sendCommandCheckOK(F( "AT+BLEHIDMOUSEBUTTON=0"  ))) {
    error(F("Failed to R Butt Up"));
  }
};
void MouseRptParser::OnRightButtonDown	(MOUSEINFO *mi)
{
  dbprintln(F("R Butt Dn"));
  if (! ble.sendCommandCheckOK(F( "AT+BLEHIDMOUSEBUTTON=R,press"  ))) {
    error(F("Failed to R Butt Dn"));
  }
};
void MouseRptParser::OnMiddleButtonUp	(MOUSEINFO *mi)
{
  dbprintln(F("M Butt Up"));
  if (! ble.sendCommandCheckOK(F( "AT+BLEHIDMOUSEBUTTON=0"  ))) {
    error(F("Failed to M Butt Up"));
  }
};
void MouseRptParser::OnMiddleButtonDown	(MOUSEINFO *mi)
{
  dbprintln(F("M Butt Dn"));
  if (! ble.sendCommandCheckOK(F( "AT+BLEHIDMOUSEBUTTON=M,press"  ))) {
    error(F("Failed to M Butt Dn"));
  }
};

USBHost     UsbH;
USBHub     Hub(&UsbH);
HIDBoot<HID_PROTOCOL_MOUSE>    HidMouse(&UsbH);

MouseRptParser                               Prs;
/*************************************************************************/


/**************************************************************************/
/*!
    @brief  Sets up the HW an the BLE module (this function is called
            automatically on startup)
*/
/**************************************************************************/
void setup(void)
{
  dbbegin(115200);
  dbprintln(F("Adafruit Bluefruit HID Mouse Example"));
  dbprintln(F("---------------------------------------"));

  /* Initialise the module */
  dbprint(F("Initialising the Bluefruit LE module: "));

  if ( !ble.begin(VERBOSE_MODE) )
  {
    error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
  }
  dbprintln( F("OK!") );

  if ( FACTORYRESET_ENABLE )
  {
    /* Perform a factory reset to make sure everything is in a known state */
    dbprintln(F("Performing a factory reset: "));
    if ( ! ble.factoryReset() ){
      error(F("Couldn't factory reset"));
    }
  }

  /* Disable command echo from Bluefruit */
  ble.echo(false);

  dbprintln(F("Requesting Bluefruit info:"));
  /* Print Bluefruit information */
  ble.info();

  // This demo only available for firmware from 0.6.6
  if ( !ble.isVersionAtLeast(MINIMUM_FIRMWARE_VERSION) )
  {
    error(F("This sketch requires firmware version " MINIMUM_FIRMWARE_VERSION " or higher!"));
  }

  /* Change the device name to make it easier to find */
  dbprintln(F("Setting device name : "));
  if (! ble.sendCommandCheckOK(F( "AT+GAPDEVNAME=BLE Mouse" )) ) {
    error(F("Could not set device name?"));
  }

  /* Enable HID Service (including Mouse) */
  dbprintln(F("Enable HID Service (including Mouse): "));
  if (! ble.sendCommandCheckOK(F( "AT+BleHIDEn=On"  ))) {
    error(F("Failed to enable HID (firmware >=0.6.6?)"));
  }

  /* Add or remove service requires a reset */
  dbprintln(F("Performing a SW reset (service changes require a reset): "));
  if (! ble.reset() ) {
    error(F("Could not reset??"));
  }

  dbprintln();
  dbprintln(F("Go to your phone's Bluetooth settings to pair your device"));
  dbprintln(F("then open an application that accepts mouse input"));
  dbprintln();

  HidMouse.SetReportParser(0, &Prs);
}

/**************************************************************************/
/*!
    @brief  Constantly poll for new command or response data
*/
/**************************************************************************/
void loop(void)
{
  UsbH.Task();
}
