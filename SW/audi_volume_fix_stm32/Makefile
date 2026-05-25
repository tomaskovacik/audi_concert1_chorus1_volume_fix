SKETCH     := $(abspath audi_volume_fix_stm32.ino)
BUILD_PATH := /tmp/arduino_build_audi_volume_fix
FQBN       := stm32duino:STM32F1:genericSTM32F103C6:upload_method=STLinkMethod,cpu_speed=speed_72mhz,opt=osstd
PORT       := /dev/ttyUSB0

BUILDER := /opt/arduino-nightly/arduino-builder

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
	-prefs=runtime.tools.stm32tools.path=/home/nail/.arduino15/packages/stm32duino/tools/stm32tools/2022.9.26 \
	-prefs=runtime.tools.stm32tools-2022.9.26.path=/home/nail/.arduino15/packages/stm32duino/tools/stm32tools/2022.9.26 \
	-prefs=runtime.tools.arm-none-eabi-gcc.path=/home/nail/.arduino15/packages/arduino/tools/arm-none-eabi-gcc/4.8.3-2014q1 \
	-prefs=runtime.tools.arm-none-eabi-gcc-4.8.3-2014q1.path=/home/nail/.arduino15/packages/arduino/tools/arm-none-eabi-gcc/4.8.3-2014q1

.PHONY: all upload serial clean

# compile + upload (arduino-builder with STLinkMethod does both in one shot)
all upload: $(BUILD_PATH)
	$(BUILDER) -compile -logger=human $(BFLAGS) $(SKETCH)

$(BUILD_PATH):
	mkdir -p $@

# open serial monitor (Ctrl-A Ctrl-X to quit)
serial:
	picocom -b 115200 $(PORT)

clean:
	rm -rf $(BUILD_PATH)
