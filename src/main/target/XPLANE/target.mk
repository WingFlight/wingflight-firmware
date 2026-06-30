# X-Plane 11/12 Hardware-in-Loop (HITL) Target Build Configuration
#
# This build configuration enables Wingflight firmware to run in a Software-in-the-Loop
# environment connected to X-Plane flight simulator via UDP protocol.

SITL_TARGETS += $(TARGET)
FEATURES       += #SDCARD_SPI VCP

# X-Plane specific source files
TARGET_SRC = \
            drivers/accgyro/accgyro_fake.c \
            drivers/barometer/barometer_fake.c \
            drivers/compass/compass_fake.c \
            drivers/serial_tcp.c

# X-Plane protocol header directory
INCLUDE_DIRS += target/XPLANE
