/*
 * USB Keyboard to BLE Keyboard converter
 *
 * Plug in a USB keyboard and convert it to a BLE keyboard.  You can buy a
 * Bluetooth keyboard that is cheaper than a Feather M0 Bluefruit LE so this
 * only makes sense for specialty USB keyboard such as mechanical keyboards
 * with Cherry MX switches which are not available with Bluetooth.
 *
 * Adafruit Feather M0 Bluefruit LE
 * USB OTG to USB host cable
 * Adafruit CP2104 break out board
 *
 * The CP2104 is used to get debug information so is not needed in normal
 * operation.  However, the Feather board must be powered using 5V because it
 * powers the USB keyboard via the micro USB connector. The lithium battery
 * voltage is too low for USB devices.
 *
 * USB host is not available on the 32u4 or the nRF52840 so this only works on
 * the M0 Bluefruit.
 *
 * The BLE code is from an Adafruit Bluefruit example program. The USB keyboard
 * code is from me.
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
#define DEBUG_SERIAL    Serial1
#define dbbegin(...)    DEBUG_SERIAL.begin(__VA_ARGS__)
#define dbprint(...)    DEBUG_SERIAL.print(__VA_ARGS__)
#define dbprintln(...)  DEBUG_SERIAL.println(__VA_ARGS__)
#else
#define dbbegin(...)
#define dbprintln(...)
#define dbprint(...)
#endif

/*
  This example shows how to send HID reports (keyboard/mouse/etc) data via BLE UART
  to another BLE device. This does not use Bluetooth Classic or BLE HID.
  The goal is to simply transfer the raw HID report to the other BLE device.
*/

#include <SPI.h>
#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"

#include "BluefruitConfig.h"

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
  while (1);
}

/************************************************************************/

//#define DEBUG_KEYBOARD_RAW 1

#include <hidboot.h>

class KeyboardRaw : public KeyboardReportParser {
public:
  KeyboardRaw(USBHost &usb) : hostKeyboard(&usb) {
    hostKeyboard.SetReportParser(0, this);
  };

  void Parse(HID *hid, uint32_t is_rpt_id, uint32_t len, uint8_t *buf);

private:
  HIDBoot<HID_PROTOCOL_KEYBOARD> hostKeyboard;
};

void KeyboardRaw::Parse(HID *hid, uint32_t is_rpt_id, uint32_t len, uint8_t *buf)
{
#ifdef DEBUG_KEYBOARD_RAW
  dbprint(F("KeyboardRaw::Parse"));
  // Show USB HID keyboard report
  for (uint8_t i = 0; i < len ; i++) {
    dbprint(' '); dbprint(buf[i], HEX);
  }
  dbprintln();
#endif

  // Call parent/super method
  KeyboardReportParser::Parse(hid, is_rpt_id, len, buf);

  // On error - return
  if (buf[2] == 1)
    return;

  if (len == 8) {
    char atcmd[64];
    snprintf(atcmd, sizeof(atcmd), "AT+BLEKEYBOARDCODE=%02x-%02x-%02x-%02x-%02x-%02x-%02x-%02x",
        buf[0],buf[1],buf[2],buf[3],buf[4],buf[5],buf[6],buf[7]);
    ble.sendCommandCheckOK(atcmd);
  }
}

// Initialize USB Controller
USBHost usb;

// Attach keyboard controller to USB
KeyboardRaw keyboard(usb);

void setup()
{
  dbbegin( 115200 );
  // Wait for serial port to connect - used on M0, Leonardo, Teensy and other boards with built-in USB CDC serial connection
  while (!SERIAL_PORT_MONITOR && millis() < 2000) delay(10);
  dbprintln(F("USB keyboard/mouse to BLE started"));

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

  /* Change the device name to make it easier to find */
  dbprintln(F("Setting device name : "));
  if (! ble.sendCommandCheckOK(F( "AT+GAPDEVNAME=BLE Keyboard" )) ) {
    error(F("Could not set device name?"));
  }

  /* Enable HID Service */
  Serial.println(F("Enable HID Service (including Keyboard): "));
  if ( ble.isVersionAtLeast(MINIMUM_FIRMWARE_VERSION) )
  {
    if ( !ble.sendCommandCheckOK(F( "AT+BleHIDEn=On" ))) {
      error(F("Could not enable Keyboard"));
    }
  }else
  {
    if (! ble.sendCommandCheckOK(F( "AT+BleKeyboardEn=On"  ))) {
      error(F("Could not enable Keyboard"));
    }
  }

  /* Add or remove service requires a reset */
  Serial.println(F("Performing a SW reset (service changes require a reset): "));
  if (! ble.reset() ) {
    error(F("Couldn't reset??"));
  }

  ble.verbose(false);  // debug info is a little annoying after this point!

  /* Wait for connection */
  while (! ble.isConnected()) {
      delay(500);
  }

  dbprintln(F("******************************"));

  // LED Activity command is only supported from 0.6.6
  if ( ble.isVersionAtLeast(MINIMUM_FIRMWARE_VERSION) )
  {
    // Change Mode LED Activity
    dbprintln(F("Change LED activity to " MODE_LED_BEHAVIOUR));
    ble.sendCommandCheckOK("AT+HWModeLED=" MODE_LED_BEHAVIOUR);
  }

  dbprintln(F("******************************"));

  dbprintln(F("Start USB host controller"));
  if (usb.Init())
    dbprintln(F("USB host controller did not start."));

  delay( 20 );

  dbprintln(F("Feather USB Kbd BLE Test!"));
  dbprintln();

}

void loop()
{
  // Process USB tasks
  usb.Task();
}
