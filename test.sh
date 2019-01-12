#!/bin/bash
IDEVER="1.8.8"
WORKDIR="/tmp/autobuild_$$"
mkdir -p ${WORKDIR}
# Install Ardino IDE in work directory
TARFILE="${HOME}/Downloads/arduino-${IDEVER}-linux64.tar.xz"
if [ -f ${TARFILE} ]
then
    tar xf ${TARFILE} -C ${WORKDIR}
else
    exit -1
fi
# Create portable sketchbook and library directories
IDEDIR="${WORKDIR}/arduino-${IDEVER}"
LIBDIR="${IDEDIR}/portable/sketchbook/libraries"
mkdir -p "${LIBDIR}"
export PATH="${IDEDIR}:${PATH}"
cd ${IDEDIR}
which arduino
# Configure board package
arduino --install-boards "arduino:samd"
arduino --pref "boardsmanager.additional.urls=https://adafruit.github.io/arduino-board-index/package_adafruit_index.json" --save-prefs
arduino --install-boards "adafruit:samd"
BOARD="adafruit:samd:adafruit_feather_m0"
arduino --board "${BOARD}" --save-prefs
CC="arduino --verify --board ${BOARD}"
arduino --pref "compiler.warning_level=default" \
		--pref "update.check=false" \
        --pref "editor.external=true" --save-prefs
cd $IDEDIR/portable/sketchbook
if [ -d ${HOME}/Sync/ble-usb-devices/ ]
then
    ln -s ${HOME}/Sync/ble-usb-devices/
else
    git clone https://github.com/gdsports/ble-usb-devices
fi
# Install libraries
arduino --install-library "Adafruit BluefruitLE nRF51"
cd $LIBDIR
if [ -d ${HOME}/Sync/USB_Host_Library_SAMD/ ]
then
    ln -s ${HOME}/Sync/USB_Host_Library_SAMD/
else
    git clone https://github.com/gdsports/USB_Host_Library_SAMD
fi
cd ../ble-usb-devices
find . -name '*.ino' -print0 | xargs -0 -n 1 $CC
