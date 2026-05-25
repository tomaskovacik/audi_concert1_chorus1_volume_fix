SKETCH     := $(abspath audi_volume_fix_avrPB.ino)
BUILD_PATH := /tmp/arduino_build_audi_volume_fix_avr
FQBN       := arduino:avr:nano:cpu=atmega328
PORT       := /dev/ttyUSB0
BINARY     := $(BUILD_PATH)/audi_volume_fix_avrPB.ino.hex

BUILDER  := /opt/arduino-nightly/arduino-builder
AVRDUDE  := /opt/arduino-nightly/hardware/tools/avr/bin/avrdude
AVRCONF  := /opt/arduino-nightly/hardware/tools/avr/etc/avrdude.conf
DECODER  := $(abspath ../decoder.py)

BFLAGS := \
	-hardware /opt/arduino-nightly/hardware \
	-hardware /home/nail/.arduino15/packages \
	-hardware /home/nail/Arduino/hardware \
	-tools /opt/arduino-nightly/tools-builder \
	-tools /opt/arduino-nightly/hardware/tools/avr \
	-tools /home/nail/.arduino15/packages \
	-built-in-libraries /opt/arduino-nightly/libraries \
	-libraries /home/nail/Arduino/libraries \
	-fqbn=$(FQBN) \
	-ide-version=10814 \
	-build-path $(BUILD_PATH) \
	-warnings=all \
	-prefs=build.warn_data_percentage=75

.PHONY: all build upload serial clean

all: clean build upload serial

build: $(BUILD_PATH)
	$(BUILDER) -compile -logger=human $(BFLAGS) $(SKETCH)

$(BUILD_PATH):
	mkdir -p $@

upload: build
	$(AVRDUDE) -C$(AVRCONF) -v -patmega328p -carduino -P$(PORT) -b115200 -D -Uflash:w:$(BINARY):i

serial:
	python3 $(DECODER) $(PORT)

clean:
	rm -rf $(BUILD_PATH)
