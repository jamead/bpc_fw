# ALS-U Bipolar Power Converter Firmware

David Bergman (dbergman@bnl.gov)


The bipolar power converter (BPC) developed for the Advanced Light Source Upgrade project (ALS-U) is a family of 4-quadrant power converters with rated power ranging from 720 W to 3.2 kW. BPCs
are switch-mode type, voltage-mode power amplifiers conceived, designed, and built by
Brookhaven National Laboratory (BNL) to meet the tight physical space requirements of the ALS-U
accelerator complex.


The controller board inside the BPC uses an SD card to configure the BPC. The file “config.txt” on the SD card is read once at startup. Some firmware upgrades may require that the contents of the config.txt file be changed, but the SD card does not contain any firmware code, only configuration data. The SD card does not need to be physically accessed to upgrade the firmware.

Firmware included with the BPCs shipped to LBNL for the AR was version 092324. Firmware beginning with version 061025 adds capability to allow for DHCP IP address assignment. 

## Requires

* Arduino IDE 1.8.19
* Arduino Mega 
* python3

## Firmware Updates

* See the docs folder for step by step instructions
