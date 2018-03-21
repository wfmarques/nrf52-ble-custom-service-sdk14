UNAME := $(shell uname)

ifeq ($(UNAME),MINGW32_NT-6.1)
	OS=windows
	CC=cc
endif
ifeq ($(UNAME),Darwin)
	OS=macosx
	CC=gcc
endif
ifeq ($(UNAME),Linux)
	OS=linux
	CC=gcc
endif

PROJECT_NAME := infinity_ble_s132_pca10040

export PATH := ..\Min:$(PATH)
#Add to the PATH variable C:\MingW\bin;C:\MingW\msys\1.0\bin 
SDK_ROOT := /Users/wesley/Developer/projeto_infinity/nRF5_SDK_14.0.0_3bcc1f7
GNU_INSTALL_ROOT := /Users/wesley/Developer/arm_linux_toolchain/arm-none-eabi-4.9
nrfjprog := /Users/wesley/Developer/arm_linux_toolchain/nordic/nRF5x-Command-Line-Tools_9_0_0_OSX/nrfjprog/nrfjprog




GNU_VERSION := 4.9.3
GNU_PREFIX := arm-none-eabi
VERBOSE := 0

export OUTPUT_FILENAME
#MAKEFILE_NAME := $(CURDIR)/$(word $(words $(MAKEFILE_LIST)),$(MAKEFILE_LIST))
MAKEFILE_NAME := $(MAKEFILE_LIST)
MAKEFILE_DIR := $(dir $(MAKEFILE_NAME) ) 

TEMPLATE_PATH = $(SDK_ROOT)/components/toolchain/gcc

MK := mkdir
RM := rm -rf

#echo suspend
ifeq ("$(VERBOSE)","1")
NO_ECHO := 
else
NO_ECHO := @
endif

# Toolchain commands
CC              := '$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-gcc'
AS              := '$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-as'
AR              := '$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-ar' -r
LD              := '$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-ld'
NM              := '$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-nm'
OBJDUMP         := '$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-objdump'
OBJCOPY         := '$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-objcopy'
SIZE            := '$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-size'

#function for removing duplicates in a list
remduplicates = $(strip $(if $1,$(firstword $1) $(call remduplicates,$(filter-out $(firstword $1),$1))))

#source common to all targets
C_SOURCE_FILES += \
$(SDK_ROOT)/components/libraries/experimental_log/src/nrf_log_backend_rtt.c \
  $(SDK_ROOT)/components/libraries/experimental_log/src/nrf_log_backend_serial.c \
  $(SDK_ROOT)/components/libraries/experimental_log/src/nrf_log_backend_uart.c \
  $(SDK_ROOT)/components/libraries/experimental_log/src/nrf_log_default_backends.c \
  $(SDK_ROOT)/components/libraries/experimental_log/src/nrf_log_frontend.c \
  $(SDK_ROOT)/components/libraries/experimental_log/src/nrf_log_str_formatter.c \
  $(SDK_ROOT)/components/libraries/button/app_button.c \
  $(SDK_ROOT)/components/libraries/util/app_error.c \
  $(SDK_ROOT)/components/libraries/util/app_error_weak.c \
  $(SDK_ROOT)/components/libraries/scheduler/app_scheduler.c \
  $(SDK_ROOT)/components/libraries/timer/app_timer.c \
  $(SDK_ROOT)/components/libraries/util/app_util_platform.c \
  $(SDK_ROOT)/components/libraries/crc16/crc16.c \
  $(SDK_ROOT)/components/libraries/fds/fds.c \
  $(SDK_ROOT)/components/libraries/hardfault/hardfault_implementation.c \
  $(SDK_ROOT)/components/libraries/util/nrf_assert.c \
  $(SDK_ROOT)/components/libraries/atomic_fifo/nrf_atfifo.c \
  $(SDK_ROOT)/components/libraries/balloc/nrf_balloc.c \
  $(SDK_ROOT)/external/fprintf/nrf_fprintf.c \
  $(SDK_ROOT)/external/fprintf/nrf_fprintf_format.c \
  $(SDK_ROOT)/components/libraries/fstorage/nrf_fstorage.c \
  $(SDK_ROOT)/components/libraries/fstorage/nrf_fstorage_sd.c \
  $(SDK_ROOT)/components/libraries/experimental_memobj/nrf_memobj.c \
  $(SDK_ROOT)/components/libraries/pwr_mgmt/nrf_pwr_mgmt.c \
  $(SDK_ROOT)/components/libraries/experimental_section_vars/nrf_section_iter.c \
  $(SDK_ROOT)/components/libraries/strerror/nrf_strerror.c \
  $(SDK_ROOT)/components/libraries/util/sdk_mapped_flags.c \
  $(SDK_ROOT)/components/libraries/sensorsim/sensorsim.c \
  $(SDK_ROOT)/components/boards/boards.c \
  $(SDK_ROOT)/components/drivers_nrf/clock/nrf_drv_clock.c \
  $(SDK_ROOT)/components/drivers_nrf/common/nrf_drv_common.c \
  $(SDK_ROOT)/components/drivers_nrf/gpiote/nrf_drv_gpiote.c \
  $(SDK_ROOT)/components/drivers_nrf/uart/nrf_drv_uart.c \
  $(SDK_ROOT)/external/segger_rtt/RTT_Syscalls_GCC.c \
  $(SDK_ROOT)/external/segger_rtt/SEGGER_RTT.c \
  $(SDK_ROOT)/external/segger_rtt/SEGGER_RTT_printf.c \
  $(SDK_ROOT)/components/ble/common/ble_advdata.c \
  $(SDK_ROOT)/components/ble/ble_advertising/ble_advertising.c \
  $(SDK_ROOT)/components/ble/common/ble_conn_params.c \
  $(SDK_ROOT)/components/ble/common/ble_conn_state.c \
  $(SDK_ROOT)/components/ble/common/ble_srv_common.c \
  $(SDK_ROOT)/components/ble/peer_manager/gatt_cache_manager.c \
  $(SDK_ROOT)/components/ble/peer_manager/gatts_cache_manager.c \
  $(SDK_ROOT)/components/ble/peer_manager/id_manager.c \
  $(SDK_ROOT)/components/ble/nrf_ble_gatt/nrf_ble_gatt.c \
  $(SDK_ROOT)/components/ble/peer_manager/peer_data_storage.c \
  $(SDK_ROOT)/components/ble/peer_manager/peer_database.c \
  $(SDK_ROOT)/components/ble/peer_manager/peer_id.c \
  $(SDK_ROOT)/components/ble/peer_manager/peer_manager.c \
  $(SDK_ROOT)/components/ble/peer_manager/pm_buffer.c \
  $(SDK_ROOT)/components/ble/peer_manager/pm_mutex.c \
  $(SDK_ROOT)/components/ble/peer_manager/security_dispatcher.c \
  $(SDK_ROOT)/components/ble/peer_manager/security_manager.c \
  $(SDK_ROOT)/components/toolchain/system_nrf52.c \
  $(SDK_ROOT)/components/softdevice/common/nrf_sdh.c \
  $(SDK_ROOT)/components/softdevice/common/nrf_sdh_ble.c \
  $(SDK_ROOT)/components/softdevice/common/nrf_sdh_soc.c \
  ./mycustom_service.c \
  ./main.c 



#assembly files common to all targets
ASM_SOURCE_FILES  = $(abspath $(SDK_ROOT)/components/toolchain/gcc/gcc_startup_nrf52.S)

#includes common to all targets
INC_PATHS += -I.
INC_PATHS += -I./config/ble_app_template_s132_pca10040
INC_PATHS += -I./config
INC_PATHS += -I$(SDK_ROOT)/components/drivers_nrf/comp 
INC_PATHS += -I$(SDK_ROOT)/components/libraries/experimental_cli 
INC_PATHS += -I$(SDK_ROOT)/components/ble/ble_services/ble_ancs_c 
INC_PATHS += -I$(SDK_ROOT)/components/ble/ble_services/ble_ias_c 
INC_PATHS += -I$(SDK_ROOT)/components/libraries/pwm 
INC_PATHS += -I$(SDK_ROOT)/components/softdevice/s132/headers/nrf52 
INC_PATHS += -I$(SDK_ROOT)/components/libraries/usbd/class/cdc/acm 
INC_PATHS += -I$(SDK_ROOT)/components/libraries/usbd/class/hid/generic 
INC_PATHS += -I$(SDK_ROOT)/components/libraries/usbd/class/msc 
INC_PATHS += -I$(SDK_ROOT)/components/libraries/usbd/class/hid 
INC_PATHS += -I$(SDK_ROOT)/components/ble/ble_services/ble_lbs 
INC_PATHS += -I$(SDK_ROOT)/components/ble/ble_services/ble_gls 
INC_PATHS += -I$(SDK_ROOT)/components/libraries/fstorage 
INC_PATHS += -I$(SDK_ROOT)/components/drivers_nrf/i2s 
INC_PATHS += -I$(SDK_ROOT)/components/libraries/mutex 
INC_PATHS += -I$(SDK_ROOT)/components/libraries/gpiote 
INC_PATHS += -I$(SDK_ROOT)/components/libraries/experimental_log/src 
INC_PATHS += -I$(SDK_ROOT)/components/drivers_nrf/gpiote 
INC_PATHS += -I$(SDK_ROOT)/components/boards 
INC_PATHS += -I$(SDK_ROOT)/components/libraries/experimental_memobj 
INC_PATHS += -I$(SDK_ROOT)/components/drivers_nrf/common 
INC_PATHS += -I$(SDK_ROOT)/components/ble/ble_advertising 
INC_PATHS += -I$(SDK_ROOT)/components/ble/ble_services/ble_bas_c 
INC_PATHS += -I$(SDK_ROOT)/components/ble/ble_services/ble_hrs_c 
INC_PATHS += -I$(SDK_ROOT)/components/libraries/queue 
INC_PATHS += -I$(SDK_ROOT)/components/libraries/pwr_mgmt 
INC_PATHS += -I$(SDK_ROOT)/components/ble/ble_dtm 
INC_PATHS += -I$(SDK_ROOT)/components/toolchain/cmsis/include 
INC_PATHS += -I$(SDK_ROOT)/components/ble/ble_services/ble_rscs_c 
INC_PATHS += -I$(SDK_ROOT)/components/drivers_nrf/uart 
INC_PATHS += -I$(SDK_ROOT)/components/ble/common 
INC_PATHS += -I$(SDK_ROOT)/components/ble/ble_services/ble_lls 
INC_PATHS += -I$(SDK_ROOT)/components/drivers_nrf/wdt 
INC_PATHS += -I$(SDK_ROOT)/components/ble/ble_services/ble_bas 
INC_PATHS += -I$(SDK_ROOT)/components/libraries/experimental_section_vars 
INC_PATHS += -I$(SDK_ROOT)/components/softdevice/s132/headers 
INC_PATHS += -I$(SDK_ROOT)/components/ble/ble_services/ble_ans_c 
INC_PATHS += -I$(SDK_ROOT)/components/libraries/slip 
INC_PATHS += -I$(SDK_ROOT)/components/libraries/mem_manager 
INC_PATHS += -I$(SDK_ROOT)/external/segger_rtt 
INC_PATHS += -I$(SDK_ROOT)/components/libraries/usbd/class/cdc 
INC_PATHS += -I$(SDK_ROOT)/components/drivers_nrf/hal 
INC_PATHS += -I$(SDK_ROOT)/components/ble/ble_services/ble_nus_c 
INC_PATHS += -I$(SDK_ROOT)/components/drivers_nrf/rtc 
INC_PATHS += -I$(SDK_ROOT)/components/softdevice/common 
INC_PATHS += -I$(SDK_ROOT)/components/ble/ble_services/ble_ias 
INC_PATHS += -I$(SDK_ROOT)/components/libraries/usbd/class/hid/mouse 
INC_PATHS += -I$(SDK_ROOT)/components/libraries/ecc 
INC_PATHS += -I$(SDK_ROOT)/components/drivers_nrf/ppi 
INC_PATHS += -I$(SDK_ROOT)/components/ble/ble_services/ble_dfu 
INC_PATHS += -I$(SDK_ROOT)/external/fprintf 
INC_PATHS += -I$(SDK_ROOT)/components/libraries/atomic 
INC_PATHS += -I$(SDK_ROOT)/components 
INC_PATHS += -I$(SDK_ROOT)/components/libraries/scheduler 
INC_PATHS += -I$(SDK_ROOT)/components/libraries/experimental_log 
INC_PATHS += -I$(SDK_ROOT)/components/ble/ble_services/ble_hts 
INC_PATHS += -I$(SDK_ROOT)/components/drivers_nrf/delay 
INC_PATHS += -I$(SDK_ROOT)/components/libraries/crc16 
INC_PATHS += -I$(SDK_ROOT)/components/drivers_nrf/timer 
INC_PATHS += -I$(SDK_ROOT)/components/libraries/util 
INC_PATHS += -I$(SDK_ROOT)/components/drivers_nrf/pwm 
INC_PATHS += -I$(SDK_ROOT)/components/libraries/csense_drv 
INC_PATHS += -I$(SDK_ROOT)/components/libraries/csense 
INC_PATHS += -I$(SDK_ROOT)/components/libraries/balloc 
INC_PATHS += -I$(SDK_ROOT)/components/libraries/low_power_pwm 
INC_PATHS += -I$(SDK_ROOT)/components/libraries/hardfault 
INC_PATHS += -I$(SDK_ROOT)/components/ble/ble_services/ble_cscs 
INC_PATHS += -I$(SDK_ROOT)/components/libraries/uart 
INC_PATHS += -I$(SDK_ROOT)/components/libraries/hci 
INC_PATHS += -I$(SDK_ROOT)/components/libraries/usbd/class/hid/kbd 
INC_PATHS += -I$(SDK_ROOT)/components/drivers_nrf/spi_slave 
INC_PATHS += -I$(SDK_ROOT)/components/drivers_nrf/lpcomp 
INC_PATHS += -I$(SDK_ROOT)/components/libraries/timer 
INC_PATHS += -I$(SDK_ROOT)/components/drivers_nrf/rng 
INC_PATHS += -I$(SDK_ROOT)/components/drivers_nrf/power 
INC_PATHS += -I$(SDK_ROOT)/components/libraries/usbd/config 
INC_PATHS += -I$(SDK_ROOT)/components/toolchain 
INC_PATHS += -I$(SDK_ROOT)/components/libraries/led_softblink 
INC_PATHS += -I$(SDK_ROOT)/components/drivers_nrf/qdec 
INC_PATHS += -I$(SDK_ROOT)/components/ble/ble_services/ble_cts_c 
INC_PATHS += -I$(SDK_ROOT)/components/drivers_nrf/spi_master 
INC_PATHS += -I$(SDK_ROOT)/components/ble/ble_services/ble_nus 
INC_PATHS += -I$(SDK_ROOT)/components/ble/ble_services/ble_hids 
INC_PATHS += -I$(SDK_ROOT)/components/libraries/strerror 
INC_PATHS += -I$(SDK_ROOT)/components/drivers_nrf/pdm 
INC_PATHS += -I$(SDK_ROOT)/components/libraries/crc32 
INC_PATHS += -I$(SDK_ROOT)/components/libraries/usbd/class/audio 
INC_PATHS += -I$(SDK_ROOT)/components/libraries/sensorsim 
INC_PATHS += -I$(SDK_ROOT)/components/ble/peer_manager 
INC_PATHS += -I$(SDK_ROOT)/components/drivers_nrf/swi 
INC_PATHS += -I$(SDK_ROOT)/components/ble/ble_services/ble_tps 
INC_PATHS += -I$(SDK_ROOT)/components/ble/ble_services/ble_dis 
INC_PATHS += -I$(SDK_ROOT)/components/device 
INC_PATHS += -I$(SDK_ROOT)/components/ble/nrf_ble_gatt 
INC_PATHS += -I$(SDK_ROOT)/components/ble/nrf_ble_qwr 
INC_PATHS += -I$(SDK_ROOT)/components/libraries/button 
INC_PATHS += -I$(SDK_ROOT)/components/libraries/usbd 
INC_PATHS += -I$(SDK_ROOT)/components/drivers_nrf/saadc 
INC_PATHS += -I$(SDK_ROOT)/components/libraries/atomic_fifo 
INC_PATHS += -I$(SDK_ROOT)/components/ble/ble_services/ble_lbs_c 
INC_PATHS += -I$(SDK_ROOT)/components/ble/ble_racp 
INC_PATHS += -I$(SDK_ROOT)/components/toolchain/gcc 
INC_PATHS += -I$(SDK_ROOT)/components/libraries/fds 
INC_PATHS += -I$(SDK_ROOT)/components/drivers_nrf/clock 
INC_PATHS += -I$(SDK_ROOT)/components/ble/ble_services/ble_rscs 
INC_PATHS += -I$(SDK_ROOT)/components/drivers_nrf/usbd 
INC_PATHS += -I$(SDK_ROOT)/components/ble/ble_services/ble_hrs 
INC_PATHS += -I$(SDK_ROOT)/components/softdevice/s132/headers
INC_PATHS += -I$(SDK_ROOT)/components/drivers_nrf/gpiote
INC_PATHS += -I$(SDK_ROOT)/components/drivers_nrf/hal


OBJECT_DIRECTORY = _build
LISTING_DIRECTORY = $(OBJECT_DIRECTORY)
OUTPUT_BINARY_DIRECTORY = $(OBJECT_DIRECTORY)

# Sorting removes duplicates
BUILD_DIRECTORIES := $(sort $(OBJECT_DIRECTORY) $(OUTPUT_BINARY_DIRECTORY) $(LISTING_DIRECTORY) )

#flags common to all targets
CFLAGS  = -DNRF52
CFLAGS += -DNRF_LOG_USES_RTT=1
CFLAGS += -DSOFTDEVICE_PRESENT
CFLAGS += -DBOARD_PCA10040
CFLAGS += -DNRF52_PAN_12
CFLAGS += -DNRF52_PAN_15
CFLAGS += -DNRF52_PAN_58
CFLAGS += -DNRF52_PAN_55
CFLAGS += -DNRF52_PAN_54
CFLAGS += -DNRF52_PAN_31
CFLAGS += -DNRF52_PAN_30
CFLAGS += -DNRF52_PAN_51
CFLAGS += -DNRF52_PAN_36
CFLAGS += -DNRF52_PAN_53
CFLAGS += -DNRF_LOG_USES_UART=1
CFLAGS += -DS132
CFLAGS += -DCONFIG_GPIO_AS_PINRESET
CFLAGS += -DBLE_STACK_SUPPORT_REQD
CFLAGS += -DNRF_SD_BLE_API_VERSION=5
CFLAGS += -DSWI_DISABLE0
CFLAGS += -DNRF52_PAN_20
CFLAGS += -DNRF52_PAN_64
CFLAGS += -DNRF52_PAN_62
CFLAGS += -DNRF52_PAN_63
CFLAGS += -mcpu=cortex-m4
CFLAGS += -mthumb -mabi=aapcs --std=c99
CFLAGS += -O3 -g3 
#-Wall -Werror
CFLAGS += -mfloat-abi=hard -mfpu=fpv4-sp-d16
# keep every function in separate section. This will allow linker to dump unused functions
CFLAGS += -ffunction-sections -fdata-sections -fno-strict-aliasing
CFLAGS += -fno-builtin --short-enums 

#clock
CFLAGS += -DNRF_SDH_CLOCK_LF_SRC=0
CFLAGS += -DNRF_SDH_CLOCK_LF_RC_CTIV=16
CFLAGS += -DNRF_SDH_CLOCK_LF_RC_TEMP_CTIV=2
CFLAGS += -DNRF_SDH_CLOCK_LF_XTAL_ACCURACY=7


# keep every function in separate section. This will allow linker to dump unused functions
LDFLAGS += -Xlinker -Map=$(LISTING_DIRECTORY)/$(OUTPUT_FILENAME).map
LDFLAGS += -mthumb -mabi=aapcs -L $(TEMPLATE_PATH) -T$(LINKER_SCRIPT)
LDFLAGS += -mcpu=cortex-m4
LDFLAGS += -mfloat-abi=hard -mfpu=fpv4-sp-d16
# let linker to dump unused sections
LDFLAGS += -Wl,--gc-sections
# use newlib in nano version
LDFLAGS += --specs=nano.specs -lc -lnosys


# Assembler flags
ASMFLAGS += -x assembler-with-cpp
ASMFLAGS += -DNRF52
ASMFLAGS += -DSOFTDEVICE_PRESENT
ASMFLAGS += -DBOARD_PCA10040
ASMFLAGS += -DNRF52_PAN_12
ASMFLAGS += -DNRF52_PAN_15
ASMFLAGS += -DNRF52_PAN_58
ASMFLAGS += -DNRF52_PAN_55
ASMFLAGS += -DNRF52_PAN_54
ASMFLAGS += -DNRF52_PAN_31
ASMFLAGS += -DNRF52_PAN_30
ASMFLAGS += -DNRF52_PAN_51
ASMFLAGS += -DNRF52_PAN_36
ASMFLAGS += -DNRF52_PAN_53
ASMFLAGS += -DNRF_LOG_USES_UART=1
ASMFLAGS += -DS132
ASMFLAGS += -DCONFIG_GPIO_AS_PINRESET
ASMFLAGS += -DBLE_STACK_SUPPORT_REQD
ASMFLAGS += -DSWI_DISABLE0
ASMFLAGS += -DNRF52_PAN_20
ASMFLAGS += -DNRF52_PAN_64
ASMFLAGS += -DNRF52_PAN_62
ASMFLAGS += -DNRF52_PAN_63

#clock
ASMFLAGS += -DNRF_SDH_CLOCK_LF_SRC=0
ASMFLAGS += -DNRF_SDH_CLOCK_LF_RC_CTIV=16
ASMFLAGS += -DNRF_SDH_CLOCK_LF_RC_TEMP_CTIV=2
ASMFLAGS += -DNRF_SDH_CLOCK_LF_XTAL_ACCURACY=7

#default target - first one defined
default: nrf52832_xxaa_s132

#building all targets
all: 
	$(NO_ECHO)$(MAKE) -f $(MAKEFILE_NAME) -C $(MAKEFILE_DIR) -e nrf52832_xxaa_s132

#target for printing all targets
help:
	@echo following targets are available:
	@echo 	nrf52832_xxaa_s132
	@echo 	flash_softdevice

C_SOURCE_FILE_NAMES = $(notdir $(C_SOURCE_FILES))
C_PATHS = $(call remduplicates, $(dir $(C_SOURCE_FILES) ) )
C_OBJECTS = $(addprefix $(OBJECT_DIRECTORY)/, $(C_SOURCE_FILE_NAMES:.c=.o) )

ASM_SOURCE_FILE_NAMES = $(notdir $(ASM_SOURCE_FILES))
ASM_PATHS = $(call remduplicates, $(dir $(ASM_SOURCE_FILES) ))
ASM_OBJECTS = $(addprefix $(OBJECT_DIRECTORY)/, $(ASM_SOURCE_FILE_NAMES:.S=.o) )

vpath %.c $(C_PATHS)
vpath %.S $(ASM_PATHS)

OBJECTS = $(C_OBJECTS) $(ASM_OBJECTS)

nrf52832_xxaa_s132: OUTPUT_FILENAME := nrf52832_xxaa_s132
nrf52832_xxaa_s132: LINKER_SCRIPT=infinity_ble_gcc_nrf52.ld

nrf52832_xxaa_s132: $(BUILD_DIRECTORIES) $(OBJECTS)
	@echo Linking target: $(OUTPUT_FILENAME).out
	$(NO_ECHO)$(CC) $(LDFLAGS) $(OBJECTS) $(LIBS) -lm -o $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out
	$(NO_ECHO)$(MAKE) -f $(MAKEFILE_NAME) -C $(MAKEFILE_DIR) -e finalize

## Create build directories
$(BUILD_DIRECTORIES):
	echo $(MAKEFILE_NAME)
	$(MK) $@

# Create objects from C SRC files
$(OBJECT_DIRECTORY)/%.o: %.c
	@echo Compiling file: $(notdir $<)
	$(NO_ECHO)$(CC) $(CFLAGS) $(INC_PATHS) -c -o $@ $<

# Assemble files
$(OBJECT_DIRECTORY)/%.o: %.S
	@echo Assembly file: $(notdir $<)
	$(NO_ECHO)$(CC) $(ASMFLAGS) $(INC_PATHS) -c -o $@ $<
# Link
$(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out: $(BUILD_DIRECTORIES) $(OBJECTS)
	@echo Linking target: $(OUTPUT_FILENAME).out
	$(NO_ECHO)$(CC) $(LDFLAGS) $(OBJECTS) $(LIBS) -lm -o $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out
## Create binary .bin file from the .out file
$(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).bin: $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out
	@echo Preparing: $(OUTPUT_FILENAME).bin
	$(NO_ECHO)$(OBJCOPY) -O binary $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).bin

## Create binary .hex file from the .out file
$(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).hex: $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out
	@echo Preparing: $(OUTPUT_FILENAME).hex
	$(NO_ECHO)$(OBJCOPY) -O ihex $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).hex

finalize: genbin genhex echosize

genbin:
	@echo Preparing: $(OUTPUT_FILENAME).bin
	$(NO_ECHO)$(OBJCOPY) -O binary $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).bin

## Create binary .hex file from the .out file
genhex: 
	@echo Preparing: $(OUTPUT_FILENAME).hex
	$(NO_ECHO)$(OBJCOPY) -O ihex $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).hex
echosize:
	-@echo ''
	$(NO_ECHO)$(SIZE) $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out
	-@echo ''

clean:
	$(RM) $(BUILD_DIRECTORIES)

cleanobj:
	$(RM) $(BUILD_DIRECTORIES)/*.o
flash: nrf52832_xxaa_s132
	@echo Flashing: $(OUTPUT_BINARY_DIRECTORY)/$<.hex
	$(nrfjprog) --program $(OUTPUT_BINARY_DIRECTORY)/$<.hex -f nrf52  --sectorerase
	$(nrfjprog) --reset -f nrf52

## Flash softdevice
flash_softdevice:
	@echo Flashing: s132_nrf52_5.0.0_softdevice.hex
	nrfjprog -f nrf52 --program $(SDK_ROOT)/components/softdevice/s132/hex/s132_nrf52_5.0.0_softdevice.hex --sectorerase
	nrfjprog -f nrf52 --reset
