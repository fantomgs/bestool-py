# BES programming tool

Based on the latest existing commit of python version of Ralim's bestool (https://github.com/Ralim/bestool/commit/49abe04f6aa717b72b6a50731462530acc282970)

Rough around the edges minimal python script to load code into the BES2300 over the uart.

This is built by capturing the traffic from the windows programming tool.
The `programmer.bin` is just from a uart capture of the payload the tool sends.
This file is obviously copyright BES.
The rest of this code & notes is released under MIT licence.

## Usage

Power off your device (if it is not already in bootloader mode), then run corresponding action.

**Warning!** Tested on the Xiaomi L05B smart speaker with FW version v1.62.28 only (flash start address is 0x2C000000). For other devices to burn correctly you should edit FLASH_START and BURN_OFFSET variables to set appropriate start address of the flash and the app offset. Don't forget to make the full flash dump backup of your device before experiments.

This will wait for sync, program the device, then drop to a uart monitor:
`./bestool.py burn-watch /path/to/firmware.bin /dev/ttyUSB0`

To list available com ports:
`./bestool.py list_ports`

Dump full flash to file:
`./bestool.py dump /dev/ttyUSB0 -w full_flash.bin -a <address> -s <size> [-p]`
for example `./bestool.py dump /dev/ttyUSB0 -w full_flash.bin -a 0x2C000000 -s 0x01000000 -p`
-p argument implies the use of the prorammer binary, it will speed up reading x2, omit it to dumping by bootloader 

Memory display:
`./bestool.py md <address> <size> /dev/ttyUSB0 [-s]`
-s - to wait for sync

Patch Xiaomi L05B/C to skip login DSA signature verification (only for FW version 1.62.28):
`./bestool.py patch /dev/ttyUSB0`
