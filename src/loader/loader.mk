###############################################################################
# Wingflight loader for the STM32H730 (XIP targets), see src/loader/main.c
#
# Included by the main Makefile when XIP=yes. Builds $(LOADER_HEX), which is
# merged with the firmware into $(TARGET_HEX).

LOADER_DIR      := $(ROOT)/src/loader
LOADER_OBJ_DIR  := $(OBJECT_DIR)/$(TARGET)/loader
LOADER_ELF      := $(LOADER_OBJ_DIR)/$(FORKNAME)_loader_$(TARGET).elf
LOADER_HEX      := $(LOADER_OBJ_DIR)/$(FORKNAME)_loader_$(TARGET).hex
LOADER_LD       := $(LOADER_DIR)/stm32h730_loader.ld
LOADER_MK       := $(LOADER_DIR)/loader.mk

LOADER_HAL_DIR  := $(ROOT)/lib/main/STM32H7/Drivers/STM32H7xx_HAL_Driver
LOADER_CMSIS    := $(ROOT)/lib/main/STM32H7/Drivers/CMSIS/Device/ST/STM32H7xx
# ST's unmodified USB device library: the H7 copy is changed for the firmware's composite device
LOADER_USB_DIR  := $(ROOT)/lib/main/STM32F4/Middlewares/ST/STM32_USB_Device_Library

LOADER_SRC := \
    $(LOADER_DIR)/main.c \
    $(LOADER_DIR)/ospi_flash.c \
    $(LOADER_DIR)/usbd_conf.c \
    $(LOADER_DIR)/usbd_desc.c \
    $(LOADER_CMSIS)/Source/Templates/gcc/startup_stm32h730xx.s \
    $(addprefix $(LOADER_HAL_DIR)/Src/stm32h7xx_, \
        hal.c hal_cortex.c hal_gpio.c hal_mdma.c hal_ospi.c hal_pcd.c hal_pcd_ex.c \
        hal_pwr.c hal_pwr_ex.c hal_rcc.c hal_rcc_ex.c ll_usb.c) \
    $(addprefix $(LOADER_USB_DIR)/, \
        Core/Src/usbd_core.c Core/Src/usbd_ctlreq.c Core/Src/usbd_ioreq.c \
        Class/DFU/Src/usbd_dfu.c)

LOADER_OBJS := $(patsubst $(ROOT)/%,$(LOADER_OBJ_DIR)/%.o,$(LOADER_SRC))

LOADER_INCLUDES := \
    $(LOADER_DIR) \
    $(SRC_DIR) \
    $(LOADER_HAL_DIR)/Inc \
    $(LOADER_CMSIS)/Include \
    $(ROOT)/lib/main/CMSIS/Core/Include \
    $(LOADER_USB_DIR)/Core/Inc \
    $(LOADER_USB_DIR)/Class/DFU/Inc

LOADER_CFLAGS := \
    -mthumb -mcpu=cortex-m7 -mfloat-abi=hard -mfpu=fpv5-sp-d16 \
    -DSTM32H730xx -DUSE_HAL_DRIVER \
    -Os -g -std=gnu17 -Wall \
    -ffunction-sections -fdata-sections -MMD -MP \
    $(addprefix -I,$(LOADER_INCLUDES))

LOADER_LDFLAGS := \
    -mthumb -mcpu=cortex-m7 -mfloat-abi=hard -mfpu=fpv5-sp-d16 \
    --specs=nano.specs --specs=nosys.specs \
    -Wl,--gc-sections -Wl,-Map,$(LOADER_OBJ_DIR)/loader.map \
    -T$(LOADER_LD)

# Rebuilt when the flags here change
$(LOADER_OBJ_DIR)/%.c.o: $(ROOT)/%.c $(LOADER_MK)
	@echo "%% (loader) $(notdir $<)" "$(STDOUT)"
	$(V1) mkdir -p $(dir $@)
	$(V1) $(CROSS_CC) -c -o $@ $(LOADER_CFLAGS) $<

$(LOADER_OBJ_DIR)/%.s.o: $(ROOT)/%.s $(LOADER_MK)
	@echo "%% (loader) $(notdir $<)" "$(STDOUT)"
	$(V1) mkdir -p $(dir $@)
	$(V1) $(CROSS_CC) -c -o $@ $(LOADER_CFLAGS) -x assembler-with-cpp $<

# Header dependencies, written by -MMD next to the objects
-include $(LOADER_OBJS:.o=.d)

$(LOADER_ELF): $(LOADER_OBJS) $(LOADER_LD) $(LOADER_MK)
	@echo "Linking loader $@" "$(STDOUT)"
	$(V1) $(CROSS_CC) -o $@ $(LOADER_OBJS) $(LOADER_LDFLAGS)
	$(V1) $(SIZE) $@

$(LOADER_HEX): $(LOADER_ELF)
	$(V1) $(OBJCOPY) -O ihex $< $@
