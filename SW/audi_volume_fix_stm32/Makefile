SKETCH     := $(abspath audi_volume_fix_stm32.ino)
BUILD_PATH := /tmp/arduino_build_audi_volume_fix
FQBN       := STMicroelectronics:stm32:GenF1:pnum=BLUEPILL_F103C6,upload_method=swdMethod,opt=osstd
PORT       := /dev/ttyUSB0
BINARY     := $(BUILD_PATH)/audi_volume_fix_stm32.ino.bin
FLASH_ADDR := 0x8000000

BUILDER  := /opt/arduino-nightly/arduino-builder
DECODER  := $(abspath ../decoder.py)

STM32_CORE_PATH := /home/nail/.arduino15/packages/STMicroelectronics/hardware/stm32/2.12.0
GCC_PATH        := /home/nail/.arduino15/packages/STMicroelectronics/tools/xpack-arm-none-eabi-gcc/14.2.1-1.1
STM32TOOLS_PATH := /home/nail/.arduino15/packages/STMicroelectronics/tools/STM32Tools/2.4.0
OPENOCD         := /home/nail/.arduino15/packages/STMicroelectronics/tools/xpack-openocd/0.12.0-6/bin/openocd
OPENOCD_SCRIPTS := /home/nail/.arduino15/packages/STMicroelectronics/tools/xpack-openocd/0.12.0-6/openocd/scripts
OPENOCD_CFG     := $(OPENOCD_SCRIPTS)/board/stm32f103c8_blue_pill.cfg

BFLAGS := \
	-hardware /opt/arduino-nightly/hardware \
	-hardware /home/nail/.arduino15/packages \
	-hardware /home/nail/Arduino/hardware \
	-tools /opt/arduino-nightly/tools-builder \
	-tools /home/nail/.arduino15/packages \
	-built-in-libraries /opt/arduino-nightly/libraries \
	-libraries /home/nail/Arduino/libraries \
	-fqbn=$(FQBN) \
	-vid-pid=0000_0000 \
	-ide-version=10814 \
	-build-path $(BUILD_PATH) \
	-warnings=all \
	-prefs=build.warn_data_percentage=75 \
	-prefs=runtime.tools.xpack-arm-none-eabi-gcc.path=$(GCC_PATH) \
	-prefs=runtime.tools.xpack-arm-none-eabi-gcc-14.2.1-1.1.path=$(GCC_PATH) \
	-prefs=runtime.tools.STM32Tools.path=$(STM32TOOLS_PATH) \
	-prefs=runtime.tools.STM32Tools-2.4.0.path=$(STM32TOOLS_PATH)

.PHONY: all build upload serial clean

all: clean build upload serial

build: $(BUILD_PATH)
	$(BUILDER) -compile -logger=human $(BFLAGS) $(SKETCH)

$(BUILD_PATH):
	mkdir -p $@

upload: build
	$(OPENOCD) -s $(OPENOCD_SCRIPTS) \
	  -f interface/stlink.cfg \
	  -f $(OPENOCD_CFG) \
	  -c "program $(BINARY) $(FLASH_ADDR) verify reset exit"

# open serial monitor with Python decoder
serial:
	python3 $(DECODER) $(PORT)

clean:
	rm -rf $(BUILD_PATH)
