/*
   MIDI USB host to BLE MIDI converter for the Adafruit Bluefruit M0 LE
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
#define DEBUG_SERIAL    Serial1
#define dbbegin(...)    DEBUG_SERIAL.begin(__VA_ARGS__)
#define dbprint(...)    DEBUG_SERIAL.print(__VA_ARGS__)
#define dbprintln(...)  DEBUG_SERIAL.println(__VA_ARGS__)
#else
#define dbbegin(...)
#define dbprint(...)
#define dbprintln(...)
#endif

#include <SPI.h>
#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"
#include "Adafruit_BLEMIDI.h"
#if SOFTWARE_SERIAL_AVAILABLE
  #include <SoftwareSerial.h>
#endif

#include "BluefruitConfig.h"

#include <usbh_midi.h>
#include <usbhub.h>

USBHost UsbH;
USBHub  Hub1(&UsbH);
USBHub  Hub2(&UsbH);
USBHub  Hub3(&UsbH);
USBHub  Hub4(&UsbH);
USBH_MIDI MIDIUSBH1(&UsbH);
USBH_MIDI MIDIUSBH2(&UsbH);
USBH_MIDI MIDIUSBH3(&UsbH);
USBH_MIDI MIDIUSBH4(&UsbH);

void USBHost_to_BLE(USBH_MIDI &MIDIUSBH)
{
  uint8_t recvBuf[MIDI_EVENT_PACKET_SIZE];
  uint8_t rcode = 0;     //return code
  uint16_t rcvd;
  uint8_t readCount = 0;

  rcode = MIDIUSBH.RecvData( &rcvd, recvBuf);

  //data check
  if (rcode != 0 || rcvd == 0) return;
  if ( recvBuf[0] == 0 && recvBuf[1] == 0 && recvBuf[2] == 0 && recvBuf[3] == 0 ) {
    return;
  }

  uint8_t *p = recvBuf;
  while (readCount < rcvd)  {
    if (*p == 0 && *(p + 1) == 0) break; //data end
    dbprint(F("USB "));
    dbprint(p[0], DEC);
    dbprint(' ');
    dbprint(p[1], DEC);
    dbprint(' ');
    dbprint(p[2], DEC);
    dbprint(' ');
    dbprintln(p[3], DEC);
    uint8_t header = *p & 0x0F;
    p++;
    switch (header) {
      case 0x00:  // Misc. Reserved for future extensions.
        break;
      case 0x01:  // Cable events. Reserved for future expansion.
        break;
      case 0x02:  // Two-byte System Common messages
      case 0x0C:  // Program Change
      case 0x0D:  // Channel Pressure
        writeBLE(p, 2);
        break;
      case 0x03:  // Three-byte System Common messages
      case 0x08:  // Note-off
      case 0x09:  // Note-on
      case 0x0A:  // Poly-KeyPress
      case 0x0B:  // Control Change
      case 0x0E:  // PitchBend Change
        writeBLE(p, 3);
        break;

      case 0x04:  // SysEx starts or continues
        writeBLE(p, 3);
        break;
      case 0x05:  // Single-byte System Common Message or SysEx ends with the following single byte
        writeBLE(p, 1);
        break;
      case 0x06:  // SysEx ends with the following two bytes
        writeBLE(p, 2);
        break;
      case 0x07:  // SysEx ends with the following three bytes
        writeBLE(p, 3);
        break;
      case 0x0F:  // Single Byte, TuneRequest, Clock, Start, Continue, Stop, etc.
        writeBLE(p, 1);
        break;
    }
    p += 3;
    readCount += 4;
  }
}

void BLE_to_USBHost()
{
#if 0
  if (MIDIUART.read()) {
    midi::MidiType msgType = MIDIUART.getType();
    dbprint(F("UART "));
    dbprint(msgType, HEX);
    dbprint(' ');
    dbprint(MIDIUART.getData1(), HEX);
    dbprint(' ');
    dbprintln(MIDIUART.getData2(), HEX);
    switch (msgType) {
      case midi::InvalidType:
        break;
      case midi::NoteOff:
      case midi::NoteOn:
      case midi::AfterTouchPoly:
      case midi::ControlChange:
      case midi::ProgramChange:
      case midi::AfterTouchChannel:
      case midi::PitchBend:
        {
          uint8_t tx[4] = {
            (byte)(msgType >> 4),
            (byte)((msgType & 0xF0) | ((MIDIUART.getChannel() - 1) & 0x0F)), /* getChannel() returns values from 1 to 16 */
            MIDIUART.getData1(),
            MIDIUART.getData2()
          };
          MIDIUSBH.SendRawData(sizeof(tx), tx);
          break;
        }
      case midi::SystemExclusive:
        MIDIUSBH.SendSysEx((uint8_t *)MIDIUART.getSysExArray(),
            MIDIUART.getSysExArrayLength(), 0);
        dbprint("sysex size ");
        dbprintln(MIDIUART.getSysExArrayLength());
        break;
      case midi::TuneRequest:
      case midi::Clock:
      case midi::Start:
      case midi::Continue:
      case midi::Stop:
      case midi::ActiveSensing:
      case midi::SystemReset:
        {
          uint8_t tx[4] = { 0x0F, (byte)(msgType), 0, 0 };
          MIDIUSBH.SendRawData(sizeof(tx), tx);
          break;
        }
      case midi::TimeCodeQuarterFrame:
      case midi::SongSelect:
        {
          uint8_t tx[4] = { 0x02, (byte)(msgType), MIDIUART.getData1(), 0 };
          MIDIUSBH.SendRawData(sizeof(tx), tx);
          break;
        }
      case midi::SongPosition:
        {
          uint8_t tx[4] = { 0x03, (byte)(msgType), MIDIUART.getData1(), MIDIUART.getData2() };
          MIDIUSBH.SendRawData(sizeof(tx), tx);
          break;
        }
      default:
        break;
    }
  }
#endif
}

/*******************************************************************/

#define FACTORYRESET_ENABLE         0
#define MINIMUM_FIRMWARE_VERSION    "0.7.0"

// This app was tested on iOS with the following apps:
//
// https://itunes.apple.com/us/app/midimittr/id925495245?mt=8
// https://itunes.apple.com/us/app/igrand-piano-free-for-ipad/id562914032?mt=8
//
// To test:
// - Run this sketch and open the Serial Monitor
// - Open the iGrand Piano Free app
// - Open the midimittr app on your phone and under Clients select "Adafruit Bluefruit LE"
// - When you see the 'Connected' label switch to the Routing panel
// - Set the Destination to 'iGrand Piano'
// - Switch to the iGrand Piano Free app and you should see notes playing one by one

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

Adafruit_BLEMIDI blemidi(ble);

/* Up to 4 full MIDI events
 * AT+BLEMIDITX=90-3C-7F
 * 1 full MIDI event with up 7 events without status
 * AT+BLEMIDITX=90-3C-7F-3C-7F
 */
void writeBLE(uint8_t *p, size_t len)
{
  blemidi.send(p);
}

bool isConnected = false;
int current_note = 60;

// A small helper
void error(const __FlashStringHelper*err) {
  dbprintln(err);
  while (1);
}

// callback
void connected(void)
{
  isConnected = true;

  dbprintln(F(" CONNECTED!"));
  delay(1000);

}

void disconnected(void)
{
  dbprintln("disconnected");
  isConnected = false;
}

void BleMidiRX(uint16_t timestamp, uint8_t status, uint8_t byte1, uint8_t byte2)
{
  dbprint("[MIDI ");
  dbprint(timestamp);
  dbprint(" ] ");

  dbprint(status, HEX); dbprint(" ");
  dbprint(byte1 , HEX); dbprint(" ");
  dbprint(byte2 , HEX); dbprint(" ");

  dbprintln();
}

void setup(void)
{
  dbbegin(115200);
  //while (!Serial);  // required for Flora & Micro
  dbprintln(F("Adafruit Bluefruit USB MIDI to BLE"));
  dbprintln(F("---------------------------------------"));

  /* Initialise the module */
  dbprint(F("Initialising the Bluefruit LE module: "));

  if ( !ble.begin(false) )
  {
    error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
  }
  dbprintln( F("OK!") );

  if ( FACTORYRESET_ENABLE )
  {
    /* Perform a factory reset to make sure everything is in a known state */
    dbprintln(F("Performing a factory reset: "));
    if ( ! ble.factoryReset() ) {
      error(F("Couldn't factory reset"));
    }
  }

  //ble.sendCommandCheckOK(F("AT+uartflow=off"));
  ble.echo(false);

  dbprintln("Requesting Bluefruit info:");
  /* Print Bluefruit information */
  ble.info();

  /* Change the device name to make it easier to find */
  dbprintln(F("Setting device name : "));
  if (! ble.sendCommandCheckOK(F( "AT+GAPDEVNAME=BLE MIDI" )) ) {
    error(F("Could not set device name?"));
  }

  /* Set BLE callbacks */
  ble.setConnectCallback(connected);
  ble.setDisconnectCallback(disconnected);

  // Set MIDI RX callback
  blemidi.setRxCallback(BleMidiRX);

  dbprintln(F("Enable MIDI: "));
  if ( ! blemidi.begin(true) )
  {
    error(F("Could not enable MIDI"));
  }

  ble.verbose(true);
  dbprint(F("Waiting for a connection..."));

  if (UsbH.Init())
    SERIAL_PORT_MONITOR.println(F("USB host controller did not start."));
}

void loop(void)
{
  UsbH.Task();

  // interval for each scanning ~ 1ms (non blocking)
  ble.update(1);

  // bail if not connected
  if (! isConnected)
    return;

  if (MIDIUSBH1) {
    /* MIDI BLE -> MIDI USB Host */
    //BLE_to_USBHost();

    /* MIDI USB Host -> MIDI BLE */
    USBHost_to_BLE(MIDIUSBH1);
  }
  if (MIDIUSBH2) {
    /* MIDI BLE -> MIDI USB Host */
    //BLE_to_USBHost();

    /* MIDI USB Host -> MIDI BLE */
    USBHost_to_BLE(MIDIUSBH2);
  }
  if (MIDIUSBH3) {
    /* MIDI BLE -> MIDI USB Host */
    //BLE_to_USBHost();

    /* MIDI USB Host -> MIDI BLE */
    USBHost_to_BLE(MIDIUSBH3);
  }
  if (MIDIUSBH4) {
    /* MIDI BLE -> MIDI USB Host */
    //BLE_to_USBHost();

    /* MIDI USB Host -> MIDI BLE */
    USBHost_to_BLE(MIDIUSBH4);
  }
}
