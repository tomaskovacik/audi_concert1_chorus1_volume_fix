SKETCH     := $(abspath audi_volume_fix_stm32.ino)
BUILD_PATH := /tmp/arduino_build_audi_volume_fix
FQBN       := stm32duino:STM32F1:genericSTM32F103C6:upload_method=STLinkMethod,cpu_speed=speed_72mhz,opt=osstd
PORT       := /dev/ttyUSB0
BINARY     := $(BUILD_PATH)/audi_volume_fix_stm32.ino.bin
FLASH_ADDR := 0x8000000

BUILDER  := /opt/arduino-nightly/arduino-builder
ST_FLASH := /home/nail/.arduino15/packages/stm32duino/tools/stm32tools/2022.9.26/linux/stlink/st-flash
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
	-vid-pid=10C4_EA60 \
	-ide-version=10814 \
	-build-path $(BUILD_PATH) \
	-warnings=all \
	-prefs=build.warn_data_percentage=75 \
	-prefs=build.upload_flags=-DCONFIG_MAPLE_MINI_NO_DISABLE_DEBUG=1 \
	-prefs=runtime.tools.stm32tools.path=/home/nail/.arduino15/packages/stm32duino/tools/stm32tools/2022.9.26 \
	-prefs=runtime.tools.stm32tools-2022.9.26.path=/home/nail/.arduino15/packages/stm32duino/tools/stm32tools/2022.9.26 \
	-prefs=runtime.tools.arm-none-eabi-gcc.path=/home/nail/.arduino15/packages/arduino/tools/arm-none-eabi-gcc/4.8.3-2014q1 \
	-prefs=runtime.tools.arm-none-eabi-gcc-4.8.3-2014q1.path=/home/nail/.arduino15/packages/arduino/tools/arm-none-eabi-gcc/4.8.3-2014q1

.PHONY: all build upload serial clean

all: clean build upload serial

build: $(BUILD_PATH)
	$(BUILDER) -compile -logger=human $(BFLAGS) $(SKETCH)

$(BUILD_PATH):
	mkdir -p $@

upload: build
	$(ST_FLASH) --reset write $(BINARY) $(FLASH_ADDR)

# open serial monitor with Python decoder
serial:
	python3 $(DECODER) $(PORT)

clean:
	rm -rf $(BUILD_PATH)
