
INCLUDE_DIRS    := $(INCLUDE_DIRS)

# build/atomic.c provides the emulated atomic_BASEPRI storage used by
# ATOMIC_BLOCK() under UNIT_TEST/SIMULATOR_BUILD; it's not part of the
# common source list since real hardware targets use inline BASEPRI asm
# instead and never reference the variable.
MCU_COMMON_SRC  := $(ROOT)/src/main/build/atomic.c

#Flags
ARCH_FLAGS      =
DEVICE_FLAGS    =
LD_SCRIPT       = src/main/target/SITL/pg.ld
STARTUP_SRC     =

TARGET_FLAGS    = -D$(TARGET)
MCU_FLASH_SIZE  := 2048

# On MinGW/MSYS (native Windows) toolchains, GCC defaults to MS-style bitfield
# layout, which breaks the packed wire-format structs (SBUS/FBUS/etc.) that
# assume GNU/System V bitfield packing, as used on the other (ARM/Linux)
# targets. Force GNU-style bitfield layout so the STATIC_ASSERTs on those
# struct sizes hold on Windows too. This flag is a no-op on Linux/macOS gcc.
ifeq ($(OSFAMILY),windows)
ARCH_FLAGS      := $(ARCH_FLAGS) -mno-ms-bitfields
endif

# SITL is a native (non-ARM) build, so the ARM cross toolchain doesn't apply here.
# On Windows, prefer the vendored MinGW-w64 GCC (see 'make mingw_sdk_install') when
# present under tools/mingw64, so TARGET=SITL doesn't depend on a system-wide
# compiler; otherwise fall back to whatever "gcc" is on PATH.
MINGW_SDK_DIR   ?= $(TOOLS_DIR)/mingw64
ifeq ($(OSFAMILY),windows)
  ifeq ($(shell [ -x "$(MINGW_SDK_DIR)/bin/gcc.exe" ] && echo "exists"), exists)
    ARM_SDK_PREFIX := $(MINGW_SDK_DIR)/bin/
  else
    ARM_SDK_PREFIX =
  endif
else
ARM_SDK_PREFIX  =
endif

MCU_EXCLUDES = \
            drivers/adc.c \
            drivers/bus_i2c.c \
            drivers/bus_i2c_config.c \
            drivers/bus_spi.c \
            drivers/bus_spi_config.c \
            drivers/bus_spi_pinconfig.c \
            drivers/dma.c \
            drivers/pwm_output.c \
            drivers/timer.c \
            drivers/system.c \
            drivers/rcc.c \
            drivers/serial_escserial.c \
            drivers/serial_uart.c \
            drivers/serial_uart_init.c \
            drivers/serial_uart_pinconfig.c \
            drivers/rx/rx_xn297.c \
            drivers/display_ug2864hsweg01.c \
            telemetry/crsf.c \
            telemetry/ghst.c \
            telemetry/srxl.c \
            io/displayport_oled.c

TARGET_MAP  = $(OBJECT_DIR)/$(FORKNAME)_$(TARGET).map

# MinGW/MSYS (native Windows) toolchains link libc implicitly and have no
# separate librt (clock_gettime/nanosleep are provided via winpthreads /
# msvcrt instead), so -lc/-lrt must be omitted there; they're required on
# Linux/macOS.
# -static avoids depending on libwinpthread-1.dll/libgcc_s_seh-1.dll being
# reachable on PATH at runtime (otherwise the exe fails to start with
# STATUS_DLL_NOT_FOUND (0xC0000135) unless run from a directory/PATH that
# includes the toolchain's bin dir).
ifeq ($(OSFAMILY),windows)
LD_FLAGS    := \
              -lm \
              -lpthread \
              -lws2_32 \
              -static \
              $(ARCH_FLAGS) \
              $(LTO_FLAGS) \
              $(DEBUG_FLAGS) \
              -Wl,-gc-sections,-Map,$(TARGET_MAP) \
              -Wl,-L$(LINKER_DIR) \
              -Wl,--cref \
              -T$(LD_SCRIPT)
else
LD_FLAGS    := \
              -lm \
              -lpthread \
              -lc \
              -lrt \
              $(ARCH_FLAGS) \
              $(LTO_FLAGS) \
              $(DEBUG_FLAGS) \
              -Wl,-gc-sections,-Map,$(TARGET_MAP) \
              -Wl,-L$(LINKER_DIR) \
              -Wl,--cref \
              -T$(LD_SCRIPT)
endif

ifneq ($(filter SITL_STATIC,$(OPTIONS)),)
LD_FLAGS     += \
              -static \
              -static-libgcc
endif

ifneq ($(DEBUG),GDB)
OPTIMISE_DEFAULT    := -Ofast
OPTIMISE_SPEED      := -Ofast
OPTIMISE_SIZE       := -Os

LTO_FLAGS           := $(OPTIMISATION_BASE) $(OPTIMISE_SPEED)
endif
