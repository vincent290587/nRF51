PROJECT_NAME := ble_ant_ancs_app_s310_pca10028

export OUTPUT_FILENAME
#MAKEFILE_NAME := $(CURDIR)/$(word $(words $(MAKEFILE_LIST)),$(MAKEFILE_LIST))
MAKEFILE_NAME := $(MAKEFILE_LIST)
MAKEFILE_DIR := $(dir $(MAKEFILE_NAME) ) 

TEMPLATE_PATH = ../../../../../../components/toolchain/gcc
ifeq ($(OS),Windows_NT)
include $(TEMPLATE_PATH)/Makefile.windows
else
include $(TEMPLATE_PATH)/Makefile.posix
endif

MK := mkdir
RM := rm -rf

#echo suspend
ifeq ("$(VERBOSE)","1")
NO_ECHO := 
else
NO_ECHO := @
endif


# Toolchain commands
CC       		:= "$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-gcc.exe"
AS       		:= "$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-as.exe"
AR       		:= "$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-ar.exe" -r
LD       		:= "$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-ld.exe"
NM       		:= "$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-nm.exe"
OBJDUMP  		:= "$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-objdump.exe"
OBJCOPY  		:= "$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-objcopy.exe"
FROMELF         := 'C:/Keil_v5/ARM/ARMCC/bin/fromelf'
NRFUTIL         := '$(PROGFILES)/Nordic Semiconductor/Master Control Panel/3.10.0.14/nrf/nrfutil'
SIZE            := '$(GNU_INSTALL_ROOT)/bin/$(GNU_PREFIX)-size'
NRFJPROG        := 'C:/Program Files (x86)/Nordic Semiconductor/nrf5x/bin/nrfjprog'

#function for removing duplicates in a list
remduplicates = $(strip $(if $1,$(firstword $1) $(call remduplicates,$(filter-out $(firstword $1),$1))))

#source common to all targets
C_SOURCE_FILES += \
$(abspath ../../../../../../components/libraries/button/app_button.c) \
$(abspath ../../../../../../components/libraries/util/app_error.c) \
$(abspath ../../../../../../components/libraries/fifo/app_fifo.c) \
$(abspath ../../../../../../components/libraries/scheduler/app_scheduler.c) \
$(abspath ../../../../../../components/libraries/timer/app_timer.c) \
$(abspath ../../../../../../components/libraries/timer/app_timer_appsh.c) \
$(abspath ../../../../../../components/libraries/trace/app_trace.c) \
$(abspath ../../../../../../components/libraries/pwm/app_pwm.c) \
$(abspath ../../../../../../components/libraries/util/nrf_assert.c) \
$(abspath ../../../../../../components/libraries/uart/retarget.c) \
$(abspath ../../../../../../components/libraries/bootloader_dfu/bootloader_util.c) \
$(abspath ../../../../../../components/libraries/bootloader_dfu/dfu_app_handler.c) \
$(abspath ../../../../../../components/drivers_nrf/uart/app_uart_fifo.c) \
$(abspath ../../../../../../components/drivers_nrf/uart/nrf_drv_uart.c) \
$(abspath ../../../../../../components/drivers_nrf/delay/nrf_delay.c) \
$(abspath ../../../../../../components/drivers_nrf/common/nrf_drv_common.c) \
$(abspath ../../../../../../components/drivers_nrf/gpiote/nrf_drv_gpiote.c) \
$(abspath ../../../../../../components/drivers_nrf/pstorage/pstorage.c) \
$(abspath ../../../../../../components/drivers_nrf/wdt/nrf_drv_wdt.c) \
$(abspath ../../../../../../components/drivers_nrf/ppi/nrf_drv_ppi.c) \
$(abspath ../../../../../../components/drivers_nrf/timer/nrf_drv_timer.c) \
$(abspath ../../../../../../components/ant/ant_stack_config/ant_stack_config.c) \
$(abspath ../../../../../../components/ant/ant_state_indicator/ant_state_indicator.c) \
$(abspath ../../../../../../components/ant/ant_channel_config/ant_channel_config.c) \
$(abspath ../../../../../../components/ant/ant_profiles/ant_glasses/ant_glasses.c) \
$(abspath ../../../../../../components/ant/ant_profiles/ant_bsc/ant_bsc.c) \
$(abspath ../../../../../../components/ant/ant_profiles/ant_bsc/pages/ant_bsc_page_0.c) \
$(abspath ../../../../../../components/ant/ant_profiles/ant_bsc/pages/ant_bsc_page_1.c) \
$(abspath ../../../../../../components/ant/ant_profiles/ant_bsc/pages/ant_bsc_page_2.c) \
$(abspath ../../../../../../components/ant/ant_profiles/ant_bsc/pages/ant_bsc_page_3.c) \
$(abspath ../../../../../../components/ant/ant_profiles/ant_bsc/pages/ant_bsc_page_4.c) \
$(abspath ../../../../../../components/ant/ant_profiles/ant_bsc/pages/ant_bsc_page_5.c) \
$(abspath ../../../../../../components/ant/ant_profiles/ant_bsc/pages/ant_bsc_combined_page_0.c) \
$(abspath ../../../../../../components/ant/ant_profiles/ant_hrm/ant_hrm.c) \
$(abspath ../../../../../../components/ant/ant_profiles/ant_hrm/pages/ant_hrm_page_0.c) \
$(abspath ../../../../../../components/ant/ant_profiles/ant_hrm/pages/ant_hrm_page_1.c) \
$(abspath ../../../../../../components/ant/ant_profiles/ant_hrm/pages/ant_hrm_page_2.c) \
$(abspath ../../../../../../components/ant/ant_profiles/ant_hrm/pages/ant_hrm_page_3.c) \
$(abspath ../../../../../../components/ant/ant_profiles/ant_hrm/pages/ant_hrm_page_4.c) \
$(abspath ../../../../../bsp/bsp.c) \
$(abspath ../../../../../bsp/bsp_btn_ble.c) \
$(abspath ../../../main.c) \
$(abspath ../../../../../../components/ble/common/ble_advdata.c) \
$(abspath ../../../../../../components/ble/ble_advertising/ble_advertising.c) \
$(abspath ../../../../../../components/ble/ble_services/ble_bas/ble_bas.c) \
$(abspath ../../../../../../components/ble/ble_services/ble_dfu/ble_dfu.c) \
$(abspath ../../../../../../components/ble/ble_services/ble_nus/ble_nus.c) \
$(abspath ../../../../../../components/ble/ble_services/ble_hrs/ble_hrs.c) \
$(abspath ../../../../../../components/ble/common/ble_conn_params.c) \
$(abspath ../../../../../../components/ble/ble_db_discovery/ble_db_discovery.c) \
$(abspath ../../../../../../components/ble/common/ble_srv_common.c) \
$(abspath ../../../../../../components/ble/device_manager/device_manager_peripheral.c) \
$(abspath ../../../../../../components/toolchain/system_nrf51.c) \
$(abspath ../../../../../../components/softdevice/common/softdevice_handler/softdevice_handler.c) \
$(abspath ../../../../../../external/segger_rtt/SEGGER_RTT_printf.c) \
$(abspath ../../../../../../external/segger_rtt/SEGGER_RTT.c) \

#assembly files common to all targets
ASM_SOURCE_FILES  = $(abspath ../../../../../../components/toolchain/gcc/gcc_startup_nrf51.s)

#includes common to all targets
INC_PATHS  = -I$(abspath ../../../config)
INC_PATHS += -I$(abspath ../../../../../../components/softdevice/s310/headers)
INC_PATHS += -I$(abspath ../../../../../../components/libraries/scheduler)
INC_PATHS += -I$(abspath ../../../../../../components/drivers_nrf/config)
INC_PATHS += -I$(abspath ../../../../../bsp)
INC_PATHS += -I$(abspath ../../../../../../components/libraries/dfu)
INC_PATHS += -I$(abspath ../../../../../../components/libraries/uart)
INC_PATHS += -I$(abspath ../../../../../../components/libraries/fifo)
INC_PATHS += -I$(abspath ../../../../../../components/libraries/util)
INC_PATHS += -I$(abspath ../../../../../../components/libraries/button)
INC_PATHS += -I$(abspath ../../../../../../components/libraries/trace)
INC_PATHS += -I$(abspath ../../../../../../components/libraries/bootloader_dfu)
INC_PATHS += -I$(abspath ../../../../../../components/libraries/pwm)
INC_PATHS += -I$(abspath ../../../../../../components/libraries/timer)
INC_PATHS += -I$(abspath ../../../../../../components/drivers_nrf/pstorage)
INC_PATHS += -I$(abspath ../../../../../../components/drivers_nrf/uart)
INC_PATHS += -I$(abspath ../../../../../../components/drivers_nrf/ppi)
INC_PATHS += -I$(abspath ../../../../../../components/drivers_nrf/hal)
INC_PATHS += -I$(abspath ../../../../../../components/drivers_nrf/delay)
INC_PATHS += -I$(abspath ../../../../../../components/drivers_nrf/timer)
INC_PATHS += -I$(abspath ../../../../../../components/drivers_nrf/wdt)
INC_PATHS += -I$(abspath ../../../../../../components/drivers_nrf/gpiote)
INC_PATHS += -I$(abspath ../../../../../../components/ble/common)
INC_PATHS += -I$(abspath ../../../../../../components/ble/device_manager)
INC_PATHS += -I$(abspath ../../../../../../components/ble/ble_services/ble_dfu)
INC_PATHS += -I$(abspath ../../../../../../components/ble/ble_services/ble_dis)
INC_PATHS += -I$(abspath ../../../../../../components/ble/ble_services/ble_nus)
INC_PATHS += -I$(abspath ../../../../../../components/ble/ble_services/ble_hrs)
INC_PATHS += -I$(abspath ../../../../../../components/device)
INC_PATHS += -I$(abspath ../../../../../../components/ble/ble_db_discovery)
INC_PATHS += -I$(abspath ../../../../../../components/ant/ant_stack_config)
INC_PATHS += -I$(abspath ../../../../../../components/ant/ant_state_indicator)
INC_PATHS += -I$(abspath ../../../../../../components/ant/ant_channel_config)
INC_PATHS += -I$(abspath ../../../../../../components/ant/ant_profiles/ant_glasses)
INC_PATHS += -I$(abspath ../../../../../../components/ant/ant_profiles/ant_bsc)
INC_PATHS += -I$(abspath ../../../../../../components/ant/ant_profiles/ant_bsc/utils)
INC_PATHS += -I$(abspath ../../../../../../components/ant/ant_profiles/ant_bsc/pages)
INC_PATHS += -I$(abspath ../../../../../../components/ant/ant_profiles/ant_bsc/pages/logger)
INC_PATHS += -I$(abspath ../../../../../../components/ant/ant_profiles/ant_hrm)
INC_PATHS += -I$(abspath ../../../../../../components/ant/ant_profiles/ant_hrm/utils)
INC_PATHS += -I$(abspath ../../../../../../components/ant/ant_profiles/ant_hrm/pages)
INC_PATHS += -I$(abspath ../../../../../../components/ant/ant_profiles/ant_hrm/pages/logger)
INC_PATHS += -I$(abspath ../../../../../../components/drivers_nrf/gpiote)
INC_PATHS += -I$(abspath ../../../../../../components/drivers_nrf/hal)
INC_PATHS += -I$(abspath ../../../../../../components/toolchain/gcc)
INC_PATHS += -I$(abspath ../../../../../../components/toolchain)
INC_PATHS += -I$(abspath ../../../../../../components/drivers_nrf/common)
INC_PATHS += -I$(abspath ../../../../../../components/ble/ble_advertising)
INC_PATHS += -I$(abspath ../../../../../../components/ble/ble_services/ble_bas)
INC_PATHS += -I$(abspath ../../../../../../components/softdevice/common/softdevice_handler)
INC_PATHS += -I$(abspath ../../../../../../external/segger_rtt)


OBJECT_DIRECTORY = _build
LISTING_DIRECTORY = $(OBJECT_DIRECTORY)
OUTPUT_BINARY_DIRECTORY = $(OBJECT_DIRECTORY)

# Sorting removes duplicates
BUILD_DIRECTORIES := $(sort $(OBJECT_DIRECTORY) $(OUTPUT_BINARY_DIRECTORY) $(LISTING_DIRECTORY) )

#flags common to all targets
CFLAGS  = -DSWI_DISABLE0
CFLAGS += -DBOARD_PCA10028
CFLAGS += -DBUTTONS_NUMBER=5 -DLEDS_NUMBER=0
CFLAGS += -DSOFTDEVICE_PRESENT
CFLAGS += -DNRF51
CFLAGS += -DANT_STACK_SUPPORT_REQD
CFLAGS += -DBLE_STACK_SUPPORT_REQD
CFLAGS += -DBLE_DFU_APP_SUPPORT
CFLAGS += -DUSE_TUNES
#CFLAGS += -DDEBUG
CFLAGS += -DS310
CFLAGS += -mcpu=cortex-m0
CFLAGS += -mthumb -mabi=aapcs --std=gnu99
CFLAGS += -Wall -O2
CFLAGS += -mfloat-abi=soft
# keep every function in separate section. This will allow linker to dump unused functions
CFLAGS += -ffunction-sections -fdata-sections -fno-strict-aliasing
CFLAGS += -fno-builtin --short-enums

# keep every function in separate section. This will allow linker to dump unused functions
LDFLAGS += -Xlinker -Map=$(LISTING_DIRECTORY)/$(OUTPUT_FILENAME).map
LDFLAGS += -mthumb -mabi=aapcs -L $(TEMPLATE_PATH) -T$(LINKER_SCRIPT)
LDFLAGS += -mcpu=cortex-m0
# let linker to dump unused sections
LDFLAGS += -Wl,--gc-sections -ffunction-sections -fdata-sections -O2
# use newlib in nano version
LDFLAGS += --specs=nano.specs -lc -lnosys

# Assembler flags
ASMFLAGS += -x assembler-with-cpp
ASMFLAGS += -DSWI_DISABLE0
ASMFLAGS += -DBOARD_PCA10028
ASMFLAGS += -DBUTTONS_NUMBER=5 -DLEDS_NUMBER=0
ASMFLAGS += -DUSE_TUNES
#ASMFLAGS += -DDEBUG
ASMFLAGS += -DSOFTDEVICE_PRESENT
ASMFLAGS += -DNRF51
ASMFLAGS += -DANT_STACK_SUPPORT_REQD
ASMFLAGS += -DBLE_STACK_SUPPORT_REQD
ASMFLAGS += -DBLE_DFU_APP_SUPPORT
ASMFLAGS += -DS310
#default target - first one defined
default: nrf51422_xxaa

xxaa: nrf51422_xxaa

xxac: nrf51422_xxac

#building all targets
all: clean
	$(NO_ECHO)$(MAKE) -f $(MAKEFILE_NAME) -C $(MAKEFILE_DIR) -e cleanobj
	$(NO_ECHO)$(MAKE) -f $(MAKEFILE_NAME) -C $(MAKEFILE_DIR) -e nrf51422_xxaa
	
cll: clean
	$(NO_ECHO)$(MAKE) -f $(MAKEFILE_NAME) -C $(MAKEFILE_DIR) -e cleanobj
	$(NO_ECHO)$(MAKE) -f $(MAKEFILE_NAME) -C $(MAKEFILE_DIR) -e nrf51422_xxac

#target for printing all targets
help:
	@echo following targets are available:
	@echo 	nrf51422_xxaa nrf51422_xxac
	@echo 	flash_softdevice


C_SOURCE_FILE_NAMES = $(notdir $(C_SOURCE_FILES))
C_PATHS = $(call remduplicates, $(dir $(C_SOURCE_FILES) ) )
C_OBJECTS = $(addprefix $(OBJECT_DIRECTORY)/, $(C_SOURCE_FILE_NAMES:.c=.o) )

ASM_SOURCE_FILE_NAMES = $(notdir $(ASM_SOURCE_FILES))
ASM_PATHS = $(call remduplicates, $(dir $(ASM_SOURCE_FILES) ))
ASM_OBJECTS = $(addprefix $(OBJECT_DIRECTORY)/, $(ASM_SOURCE_FILE_NAMES:.s=.o) )

vpath %.c $(C_PATHS)
vpath %.s $(ASM_PATHS)

OBJECTS = $(C_OBJECTS) $(ASM_OBJECTS)

nrf51422_xxaa: OUTPUT_FILENAME := nrf51422_xxax
nrf51422_xxaa: LINKER_SCRIPT=ble_strava_gcc_nrf51xxaa.ld
nrf51422_xxaa: $(BUILD_DIRECTORIES) $(OBJECTS)
	@echo Linking target: $(OUTPUT_FILENAME).out
	$(NO_ECHO)$(CC) $(LDFLAGS) $(OBJECTS) $(LIBS) -o $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out
	$(NO_ECHO)$(MAKE) -f $(MAKEFILE_NAME) -C $(MAKEFILE_DIR) -e finalize

nrf51422_xxac: OUTPUT_FILENAME := nrf51422_xxax
nrf51422_xxac: LINKER_SCRIPT=ble_strava_gcc_nrf51xxac.ld
nrf51422_xxac: $(BUILD_DIRECTORIES) $(OBJECTS)
	@echo Linking target: $(OUTPUT_FILENAME).out
	$(NO_ECHO)$(CC) $(LDFLAGS) $(OBJECTS) $(LIBS) -o $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out
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
$(OBJECT_DIRECTORY)/%.o: %.s
	@echo Compiling file: $(notdir $<)
	$(NO_ECHO)$(CC) $(ASMFLAGS) $(INC_PATHS) -c -o $@ $<


# Link
$(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out: $(BUILD_DIRECTORIES) $(OBJECTS)
	@echo Linking target: $(OUTPUT_FILENAME).out
	$(NO_ECHO)$(CC) $(LDFLAGS) $(OBJECTS) $(LIBS) -o $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out


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
	$(NRFUTIL) --verbose dfu genpkg $(OUTPUT_FILENAME).zip --application $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).bin --application-version 0xff --dev-revision 1 --dev-type 1 --sd-req 0xfffe

## Create binary .hex file from the .out file
genhex: 
	@echo Preparing: $(OUTPUT_FILENAME).hex
	$(NO_ECHO)$(OBJCOPY) -O ihex $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).hex

echosize:
	-@echo ""
	$(NO_ECHO)$(SIZE) $(OUTPUT_BINARY_DIRECTORY)/$(OUTPUT_FILENAME).out
	-@echo ""

clean:
	$(RM) $(BUILD_DIRECTORIES)

cleanobj:
	$(RM) $(BUILD_DIRECTORIES)/*.o

flash:
	@echo Flashing: $(OUTPUT_BINARY_DIRECTORY)/nrf51422_xxax.hex
	$(NRFJPROG) --program $(OUTPUT_BINARY_DIRECTORY)/nrf51422_xxax.hex -f nrf51  --sectorerase
	$(NRFJPROG) --reset

## Flash softdevice
flash_softdevice: 
	@echo Flashing: s310_softdevice.hex
	$(NRFJPROG) --program ../../../../../../components/softdevice/s310/hex/s310_softdevice.hex
	$(NRFJPROG) --reset