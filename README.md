# Convert USB keyboard, mouse, and MIDI to Bluetooth LE

## Hardware

* [Adafruit Feather M0 Bluefruit LE](https://www.adafruit.com/product/2995)
* [USB OTG to USB host cable](https://www.adafruit.com/product/1099)
* [Adafruit CP2104 break out board](https://www.adafruit.com/product/3309)

The CP2104 is used to get debug information so is not needed in normal
operation. However, the Feather board must be powered using 5V because it
powers the USB keyboard via the micro USB connector. Lithium battery voltage is
too low to power USB devices.

USB host is not available on the 32u4 or the nRF52840 so this only works on the
M0 Bluefruit. When the micro USB port is in USB host mode, it cannot be used
for serial console. The sketches use Serial1 as the serial console. Also the
Arduino IDE auto upload feature do not work. Just double click on the Feather
reset button to put the board in upload mode.

Be sure to follow [Adafruit's
tutorial](https://learn.adafruit.com/adafruit-feather-m0-bluefruit-le/overview)
to make sure the boards work and the libraries are installed.

## Required library

The [USB Host Library for
SAMD](https://github.com/gdsports/USB_Host_Library_SAMD) must be installed
because it adds USB host drivers for USB keyboard, mouse, and MIDI devices.

## Connection

```
USB         USB OTG Host
Keyboard -> cable/adapter -> Feather M0 BLE
                             GND  USB  Tx  Rx
                              |    ^   |   ^
                              |    |   |   |
                              |    |   v   |
                             GND  5V   RxD TxD
                             CP2104 USB to serial -> Computer or 5V power supply
```

Substitute keyboard with mouse or MIDI device and use the corresponding sketch.


## USBKbdBle -- USB Keyboard to BLE

Plug in a USB keyboard and convert it to a BLE keyboard. A Bluetooth keyboard
is cheaper than a Feather M0 Bluefruit LE so this only makes sense for a
specialty USB keyboard. For example, a mechanical keyboard with Cherry MX
switches which is not available with Bluetooth.

If a USB barcode scanner emulates a USB keyboard, this project may be able to
BLE-ify it. This may also work for USB RFID scanners.

## USBMouseBle -- USB Mouse to BLE

This works for USB mouse with left, right, and optional middle buttons. 12
button MMO gaming mice need not apply.

## USBMidiBLE -- USB MIDI to BLE

Tested on a MIDI keyboard and an iPad using the free "midimittr" and "iGrand
Piano" apps. Auto connect after power off/on does not work but forgetting the
Bluefruit then re-pairing it works. Also close and opening the midimittr app is
also required.
