ARDUINO_DIR   = /Applications/Arduino.app/Contents/Java
ARDUINO_PACKAGE_DIR = $(HOME)/Library/Arduino15/packages
ALTERNATE_CORE_PATH = $(HOME)/Library/Arduino15/packages/adafruit/hardware/samd/1.2.9
CMSIS_DIR = $(ARDUINO_PACKAGE_DIR)/arduino/tools/CMSIS/4.5.0/CMSIS
CMSIS_ATMEL_DIR = $(ARDUINO_PACKAGE_DIR)/arduino/tools/CMSIS-Atmel/1.2.0/CMSIS
ARCHITECTURE = sam
BOARD_TAG     = adafruit_feather_m4
#BOARD_TAG     = adafruit_feather_m0
CXXFLAGS_STD = -DARDUINO_ARCH_SAMD

#ARDUINO_PORT = /dev/tty.usbmodem14501

ARDUINO_LIBS += Audio SPI Adafruit_NeoPixel \
				Adafruit_ZeroDMA \
				Adafruit_LIS3DH Wire Adafruit_Sensor 

VERSION=$(shell git describe --tags --always --dirty 2> /dev/null)

include /usr/local/opt/arduino-mk/Sam.mk

#CC = arm-none-eabi-g++
CXX = arm-none-eabi-g++


.PHONY: upload
upload: all
	/usr/local/bin/ard-reset-arduino --zero $(DEVICE_PATH)
	../BOSSA/bin/bossac --erase --write --verify --info --reset \
		                --port=$(DEVICE_PATH) --offset=0x4000 \
						build-$(BOARD_TAG)/arduino_lightsaber.bin
	sleep 3
	screen $(DEVICE_PATH) $(MONITOR_BAUDRATE)
