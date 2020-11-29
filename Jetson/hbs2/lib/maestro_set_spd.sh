#!/bin/bash
# Sets the speed of a Simple Motor Controller via its virtual serial port.
# Usage: smc-set-speed.sh DEVICE SPEED
# Linux example: bash smc-set-speed.sh /dev/ttyACM0 3200
# Mac OS X example: bash smc-set-speed.sh /dev/cu.usbmodemfa121 3200
# Windows example: bash smc-set-speed.sh '\\.\USBSER000' 3200
# Windows example: bash smc-set-speed.sh '\\.\COM6' 3200
# DEVICE is the name of the virtual COM port device.
# SPEED is a number between 1 and 100

DEVICE=$1
CHANNEL=$2
SPEED=$3

byte() {
  printf "\\x$(printf "%x" $1)"
}

{
  byte 0x87
  byte $CHANNEL
  byte $((SPEED & 0x7F))
  byte $((SPEED >> 7 & 0x7F))
} > $DEVICE
