/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>
#include <limits.h>
#include <ctype.h>

#include "platform.h"

#include "blackbox/blackbox.h"
#include "blackbox/blackbox_io.h"

#include "build/build_config.h"
#include "build/debug.h"
#include "build/version.h"


#include "common/axis.h"
#include "common/bitarray.h"
#include "common/color.h"
#include "common/huffman.h"
#include "common/maths.h"
#include "common/printf.h"
#include "common/streambuf.h"
#include "common/utils.h"

#include "config/config.h"
#include "config/config_eeprom.h"
#include "config/custom_defaults.h"
#include "config/feature.h"

#include "drivers/accgyro/accgyro.h"
#include "drivers/bus_i2c.h"
#include "drivers/bus_spi.h"
#include "drivers/compass/compass.h"
#include "drivers/display.h"
#include "drivers/dshot.h"
#include "drivers/dshot_command.h"
#include "drivers/fbus_master.h"
#include "drivers/fbus_mux_fpga.h"
#include "drivers/rx_input_backup.h"
#include "drivers/fbus_sensor.h"
#include "drivers/fbus_xact.h"
#include "drivers/crsf_sensors.h"
#include "drivers/flash.h"
#include "drivers/io.h"
#include "drivers/motor.h"
#include "drivers/pwm_output.h"
#include "drivers/sdcard.h"
#include "drivers/serial.h"
#include "drivers/serial_escserial.h"
#include "drivers/system.h"
#include "drivers/usb_msc.h"
#include "drivers/freq.h"

#include "fc/board_info.h"
#include "fc/rc_rates.h"
#include "fc/core.h"
#include "fc/dispatch.h"
#include "fc/rc.h"
#include "fc/rc_adjustments.h"
#include "fc/rc_controls.h"
#include "fc/rc_modes.h"
#include "fc/runtime_config.h"

#include "flight/failsafe.h"
#include "flight/gps_rescue.h"
#include "flight/imu.h"
#include "flight/mixer.h"
#include "flight/logic_condition.h"
#include "flight/pid.h"
#include "flight/tv_hold.h"
#include "flight/tv_pid.h"
#include "flight/position.h"
#include "flight/rpm_filter.h"
#include "flight/servos.h"
#include "flight/tune_advisor.h"

#include "io/asyncfatfs/asyncfatfs.h"
#include "io/beeper.h"
#include "io/flashfs.h"
#include "io/gps.h"
#include "io/ledstrip.h"
#include "io/serial.h"
#include "io/serial_4way.h"
#include "io/servos.h"
#include "io/usb_msc.h"

#include "msp/msp_box.h"
#include "msp/msp_catalogue.h"
#include "msp/msp_param.h"
#include "msp/msp_runtime.h"
#include "msp/msp_protocol.h"
#include "msp/msp_protocol_v2_betaflight.h"
#include "msp/msp_protocol_v2_rotorflight.h"
#include "msp/msp_protocol_v2_common.h"
#include "msp/msp_serial.h"

#include "pg/beeper.h"
#include "pg/board.h"
#include "pg/dyn_notch.h"
#include "pg/gyrodev.h"
#include "pg/gps_nav.h"
#include "pg/governor.h"
#include "pg/motor.h"
#include "pg/rx.h"
#include "pg/rx_spi.h"
#include "pg/stats.h"
#include "pg/usb.h"
#include "pg/battery.h"
#include "pg/sbus_output.h"
#include "pg/fbus_master.h"
#include "pg/rx_input_backup.h"
#include "pg/bus_servo.h"
#include "pg/logic_condition.h"
#ifdef USE_SRXL2_ESC
#include "pg/srxl2_esc.h"
#endif

#include "rx/rx.h"
#include "rx/rx_bind.h"
#include "rx/msp.h"

#include "scheduler/scheduler.h"

#include "sensors/acceleration.h"
#include "sensors/barometer.h"
#include "sensors/battery.h"
#include "sensors/smartfuel.h"
#include "sensors/boardalignment.h"
#include "sensors/boardalignment_auto.h"
#include "sensors/boardmounttrim_auto.h"
#include "sensors/compass.h"
#include "sensors/esc_sensor.h"
#include "sensors/gyro.h"
#include "sensors/gyro_init.h"

#include "telemetry/msp_shared.h"
#include "telemetry/telemetry.h"

#ifdef USE_HARDWARE_REVISION_DETECTION
#include "hardware_revision.h"
#endif

#include "msp.h"


static const char * const flightControllerIdentifier = FC_FIRMWARE_IDENTIFIER; // 4 UPPER CASE alpha numeric characters that identify the flight controller.

enum {
    MSP_REBOOT_FIRMWARE = 0,
    MSP_REBOOT_BOOTLOADER_ROM,
    MSP_REBOOT_MSC,
    MSP_REBOOT_MSC_UTC,
    MSP_REBOOT_BOOTLOADER_FLASH,
    MSP_REBOOT_COUNT,
};

static uint8_t rebootMode;

typedef enum {
    MSP_SDCARD_STATE_NOT_PRESENT = 0,
    MSP_SDCARD_STATE_FATAL       = 1,
    MSP_SDCARD_STATE_CARD_INIT   = 2,
    MSP_SDCARD_STATE_FS_INIT     = 3,
    MSP_SDCARD_STATE_READY       = 4
} mspSDCardState_e;

typedef enum {
    MSP_SDCARD_FLAG_SUPPORTED   = 1
} mspSDCardFlags_e;

typedef enum {
    MSP_FLASHFS_FLAG_READY       = 1,
    MSP_FLASHFS_FLAG_SUPPORTED  = 2
} mspFlashFsFlags_e;

typedef enum {
    MSP_PASSTHROUGH_ESC_SIMONK = PROTOCOL_SIMONK,
    MSP_PASSTHROUGH_ESC_BLHELI = PROTOCOL_BLHELI,
    MSP_PASSTHROUGH_ESC_KISS = PROTOCOL_KISS,
    MSP_PASSTHROUGH_ESC_KISSALL = PROTOCOL_KISSALL,
    MSP_PASSTHROUGH_ESC_CASTLE = PROTOCOL_CASTLE,

    MSP_PASSTHROUGH_SERIAL_ID = 0xFD,
    MSP_PASSTHROUGH_SERIAL_FUNCTION_ID = 0xFE,

    MSP_PASSTHROUGH_ESC_4WAY = 0xFF,
} mspPassthroughType_e;

#define RATEPROFILE_MASK (1 << 7)

#define RTC_NOT_SUPPORTED 0xff

typedef enum {
    DEFAULTS_TYPE_BASE = 0,
    DEFAULTS_TYPE_CUSTOM,
} defaultsType_e;


static int mspDescriptor = 0;

mspDescriptor_t mspDescriptorAlloc(void)
{
    return (mspDescriptor_t)mspDescriptor++;
}

static uint32_t mspArmingDisableFlags = 0;

static void mspArmingDisableByDescriptor(mspDescriptor_t desc)
{
    mspArmingDisableFlags |= (1 << desc);
}

static void mspArmingEnableByDescriptor(mspDescriptor_t desc)
{
    mspArmingDisableFlags &= ~(1 << desc);
}

static bool mspIsMspArmingEnabled(void)
{
    return mspArmingDisableFlags == 0;
}

#define MSP_PASSTHROUGH_ESC_4WAY 0xff

static uint8_t mspPassthroughMode;
static uint8_t mspPassthroughArgument;

#ifdef USE_ESCSERIAL
static void mspEscPassthroughFn(serialPort_t *serialPort)
{
    escEnablePassthrough(serialPort, &motorConfig()->dev, mspPassthroughArgument, mspPassthroughMode);
}
#endif

static serialPort_t *mspFindPassthroughSerialPort(void)
{
    serialPortUsage_t *portUsage = NULL;

    switch (mspPassthroughMode) {
    case MSP_PASSTHROUGH_SERIAL_ID:
    {
        portUsage = findSerialPortUsageByIdentifier(mspPassthroughArgument);
        break;
    }
    case MSP_PASSTHROUGH_SERIAL_FUNCTION_ID:
    {
        const serialPortConfig_t *portConfig = findSerialPortConfig(1 << mspPassthroughArgument);
        if (portConfig) {
            portUsage = findSerialPortUsageByIdentifier(portConfig->identifier);
        }
        break;
    }
    }
    return portUsage ? portUsage->serialPort : NULL;
}

static void mspSerialPassthroughFn(serialPort_t *serialPort)
{
    serialPort_t *passthroughPort = mspFindPassthroughSerialPort();
    if (passthroughPort && serialPort) {
        serialPassthrough(passthroughPort, serialPort, NULL, NULL);
    }
}

static void mspFcSetPassthroughCommand(sbuf_t *dst, sbuf_t *src, mspPostProcessFnPtr *mspPostProcessFn)
{
    const unsigned int dataSize = sbufBytesRemaining(src);
    if (dataSize == 0) {
        // Legacy format
        mspPassthroughMode = MSP_PASSTHROUGH_ESC_4WAY;
    } else {
        mspPassthroughMode = sbufReadU8(src);
        mspPassthroughArgument = sbufReadU8(src);
    }

    switch (mspPassthroughMode) {
    case MSP_PASSTHROUGH_SERIAL_ID:
    case MSP_PASSTHROUGH_SERIAL_FUNCTION_ID:
        if (mspFindPassthroughSerialPort()) {
            if (mspPostProcessFn) {
                *mspPostProcessFn = mspSerialPassthroughFn;
            }
            sbufWriteU8(dst, 1);
        } else {
            sbufWriteU8(dst, 0);
        }
        break;
#ifdef USE_SERIAL_4WAY_BLHELI_INTERFACE
    case MSP_PASSTHROUGH_ESC_4WAY:
        // get channel number
        // switch all motor lines HI
        // reply with the count of ESC found
        sbufWriteU8(dst, esc4wayInit());

        if (mspPostProcessFn) {
            *mspPostProcessFn = esc4wayProcess;
        }
        break;

#ifdef USE_ESCSERIAL
    case MSP_PASSTHROUGH_ESC_SIMONK:
    case MSP_PASSTHROUGH_ESC_BLHELI:
    case MSP_PASSTHROUGH_ESC_KISS:
    case MSP_PASSTHROUGH_ESC_KISSALL:
    case MSP_PASSTHROUGH_ESC_CASTLE:
        if (mspPassthroughArgument < getMotorCount() || (mspPassthroughMode == MSP_PASSTHROUGH_ESC_KISS && mspPassthroughArgument == ALL_MOTORS)) {
            sbufWriteU8(dst, 1);

            if (mspPostProcessFn) {
                *mspPostProcessFn = mspEscPassthroughFn;
            }

            break;
        }
        FALLTHROUGH;
#endif // USE_ESCSERIAL
#endif //USE_SERIAL_4WAY_BLHELI_INTERFACE
    default:
        sbufWriteU8(dst, 0);
    }
}

// TODO: Remove the pragma once this is called from unconditional code
#pragma GCC diagnostic ignored "-Wunused-function"
static void configRebootUpdateCheckU8(uint8_t *parm, uint8_t value)
{
    if (*parm != value) {
        setRebootRequired();
    }
    *parm = value;
}
#pragma GCC diagnostic pop

static void mspRebootFn(serialPort_t *serialPort)
{
    UNUSED(serialPort);

    switch (rebootMode) {
    case MSP_REBOOT_FIRMWARE:
        systemReset(RESET_NONE);
        break;

    case MSP_REBOOT_BOOTLOADER_ROM:
        systemReset(RESET_BOOTLOADER_REQUEST_ROM);
        break;

#if defined(USE_USB_MSC)
    case MSP_REBOOT_MSC:
    case MSP_REBOOT_MSC_UTC: {
#ifdef USE_RTC_TIME
        const int16_t timezoneOffsetMinutes = (rebootMode == MSP_REBOOT_MSC) ? timeConfig()->tz_offsetMinutes : 0;
        systemResetToMsc(timezoneOffsetMinutes);
#else
        systemResetToMsc(0);
#endif
        }
        break;
#endif

#if defined(USE_FLASH_BOOT_LOADER)
    case MSP_REBOOT_BOOTLOADER_FLASH:
        systemReset(RESET_BOOTLOADER_REQUEST_FLASH);
        break;
#endif

    default:
        return;
    }

    // control should never return here.
    while (true) ;
}

#define MSP_DISPATCH_DELAY_US 1000000

void mspReboot(dispatchEntry_t* self)
{
    UNUSED(self);

    if (ARMING_FLAG(ARMED)) {
        return;
    }

    mspRebootFn(NULL);
}

dispatchEntry_t mspRebootEntry =
{
    mspReboot, 0, NULL, false
};

void writeReadEeprom(dispatchEntry_t* self)
{
    UNUSED(self);

    if (ARMING_FLAG(ARMED)) {
        return;
    }

    writeEEPROM();
    readEEPROM();

    // readEEPROM() validates and activates the configuration, but some
    // runtime state is otherwise built only at boot or by the config setter
    // that changed it. Clients writing through MSP2_WING_PARAM_WRITE rely on
    // the save to apply it (parameter-addressing-design.md, step 3), so the
    // save rebuilds it the way those setters did.
    gyroInitFilters();
#if defined(USE_RPM_FILTER)
    rpmFilterInit();
#endif
    smartFuelInit();
#if defined(USE_FBUS_MASTER) || defined(USE_SPORT_MASTER)
    fbusSensorInitForwarding();
#endif
    // MSP_SET_MIXER_RULE rebuilds the saturation map after each rule write.
    mixerUpdateMapping();
}

dispatchEntry_t writeReadEepromEntry =
{
    writeReadEeprom, 0, NULL, false
};

static void serializeSDCardSummaryReply(sbuf_t *dst)
{
    uint8_t flags = 0;
    uint8_t state = 0;
    uint8_t lastError = 0;
    uint32_t freeSpace = 0;
    uint32_t totalSpace = 0;

#if defined(USE_SDCARD)
    if (sdcardConfig()->mode != SDCARD_MODE_NONE) {
        flags = MSP_SDCARD_FLAG_SUPPORTED;

        // Merge the card and filesystem states together
        if (!sdcard_isInserted()) {
            state = MSP_SDCARD_STATE_NOT_PRESENT;
        } else if (!sdcard_isFunctional()) {
            state = MSP_SDCARD_STATE_FATAL;
        } else {
            switch (afatfs_getFilesystemState()) {
            case AFATFS_FILESYSTEM_STATE_READY:
                state = MSP_SDCARD_STATE_READY;
                break;

            case AFATFS_FILESYSTEM_STATE_INITIALIZATION:
             if (sdcard_isInitialized()) {
                 state = MSP_SDCARD_STATE_FS_INIT;
             } else {
                 state = MSP_SDCARD_STATE_CARD_INIT;
             }
             break;

            case AFATFS_FILESYSTEM_STATE_FATAL:
            case AFATFS_FILESYSTEM_STATE_UNKNOWN:
            default:
                state = MSP_SDCARD_STATE_FATAL;
                break;
            }
        }

        lastError = afatfs_getLastError();
        // Write free space and total space in kilobytes
        if (state == MSP_SDCARD_STATE_READY) {
            freeSpace = afatfs_getContiguousFreeSpace() / 1024;
            totalSpace = sdcard_getMetadata()->numBlocks / 2;
        }
    }
#endif

    sbufWriteU8(dst, flags);
    sbufWriteU8(dst, state);
    sbufWriteU8(dst, lastError);
    sbufWriteU32(dst, freeSpace);
    sbufWriteU32(dst, totalSpace);
}

static void serializeDataflashSummaryReply(sbuf_t *dst)
{
#ifdef USE_FLASHFS
    if (flashfsIsSupported()) {
        uint8_t flags = MSP_FLASHFS_FLAG_SUPPORTED;
        flags |= (flashfsIsReady() ? MSP_FLASHFS_FLAG_READY : 0);

        const flashPartition_t *flashPartition = flashPartitionFindByType(FLASH_PARTITION_TYPE_FLASHFS);

        sbufWriteU8(dst, flags);
        sbufWriteU32(dst, FLASH_PARTITION_SECTOR_COUNT(flashPartition));
        sbufWriteU32(dst, flashfsGetSize());
        sbufWriteU32(dst, flashfsGetOffset()); // Effectively the current number of bytes stored on the volume
    } else
#endif

    // FlashFS is not configured or valid device is not detected
    {
        sbufWriteU8(dst, 0);
        sbufWriteU32(dst, 0);
        sbufWriteU32(dst, 0);
        sbufWriteU32(dst, 0);
    }
}

#ifdef USE_FLASHFS
enum compressionType_e {
    NO_COMPRESSION,
    HUFFMAN
};

static void serializeDataflashReadReply(sbuf_t *dst, uint32_t address, const uint16_t size, bool useLegacyFormat, bool allowCompression)
{
    STATIC_ASSERT(MSP_PORT_DATAFLASH_INFO_SIZE >= 16, MSP_PORT_DATAFLASH_INFO_SIZE_invalid);

    uint16_t readLen = size;
    const int bytesRemainingInBuf = sbufBytesRemaining(dst) - MSP_PORT_DATAFLASH_INFO_SIZE;
    if (readLen > bytesRemainingInBuf) {
        readLen = bytesRemainingInBuf;
    }
    // size will be lower than that requested if we reach end of volume
    const uint32_t flashfsSize = flashfsGetSize();
    if (readLen > flashfsSize - address) {
        // truncate the request
        readLen = flashfsSize - address;
    }
    sbufWriteU32(dst, address);

    // legacy format does not support compression
#ifdef USE_HUFFMAN
    const uint8_t compressionMethod = (!allowCompression || useLegacyFormat) ? NO_COMPRESSION : HUFFMAN;
#else
    const uint8_t compressionMethod = NO_COMPRESSION;
    UNUSED(allowCompression);
#endif

    if (compressionMethod == NO_COMPRESSION) {

        uint16_t *readLenPtr = (uint16_t *)sbufPtr(dst);
        if (!useLegacyFormat) {
            // new format supports variable read lengths
            sbufWriteU16(dst, readLen);
            sbufWriteU8(dst, 0); // placeholder for compression format
        }

        const int bytesRead = flashfsReadAbs(address, sbufPtr(dst), readLen);

        if (!useLegacyFormat) {
            // update the 'read length' with the actual amount read from flash.
            *readLenPtr = bytesRead;
        }

        sbufAdvance(dst, bytesRead);

        if (useLegacyFormat) {
            // pad the buffer with zeros
            for (int i = bytesRead; i < size; i++) {
                sbufWriteU8(dst, 0);
            }
        }
    } else {
#ifdef USE_HUFFMAN
        // compress in 256-byte chunks
        const uint16_t READ_BUFFER_SIZE = 256;
        // This may be DMAable, so make it cache aligned
        __attribute__ ((aligned(32))) uint8_t readBuffer[READ_BUFFER_SIZE];

        huffmanState_t state = {
            .bytesWritten = 0,
            .outByte = sbufPtr(dst) + sizeof(uint16_t) + sizeof(uint8_t) + HUFFMAN_INFO_SIZE,
            .outBufLen = readLen,
            .outBit = 0x80,
        };
        *state.outByte = 0;

        uint16_t bytesReadTotal = 0;
        // read until output buffer overflows or flash is exhausted
        while (state.bytesWritten < state.outBufLen && address + bytesReadTotal < flashfsSize) {
            const int bytesRead = flashfsReadAbs(address + bytesReadTotal, readBuffer,
                MIN(sizeof(readBuffer), flashfsSize - address - bytesReadTotal));

            const int status = huffmanEncodeBufStreaming(&state, readBuffer, bytesRead, huffmanTable);
            if (status == -1) {
                // overflow
                break;
            }

            bytesReadTotal += bytesRead;
        }

        if (state.outBit != 0x80) {
            ++state.bytesWritten;
        }

        // header
        sbufWriteU16(dst, HUFFMAN_INFO_SIZE + state.bytesWritten);
        sbufWriteU8(dst, compressionMethod);
        // payload
        sbufWriteU16(dst, bytesReadTotal);
        sbufAdvance(dst, state.bytesWritten);
#endif
    }
}
#endif // USE_FLASHFS

/*
 * Returns true if the command was processd, false otherwise.
 * May set mspPostProcessFunc to a function to be called once the command has been processed
 */
static bool mspCommonProcessOutCommand(int16_t cmdMSP, sbuf_t *dst, mspPostProcessFnPtr *mspPostProcessFn)
{
    UNUSED(mspPostProcessFn);

    switch (cmdMSP) {
    case MSP_API_VERSION:
        sbufWriteU8(dst, MSP_PROTOCOL_VERSION);
        sbufWriteU8(dst, API_VERSION_MAJOR);
        sbufWriteU8(dst, API_VERSION_MINOR);
        break;

    case MSP_FC_VARIANT:
        sbufWriteData(dst, flightControllerIdentifier, FLIGHT_CONTROLLER_IDENTIFIER_LENGTH);
        break;

    case MSP_FC_VERSION:
        sbufWriteU8(dst, FC_VERSION_MAJOR);
        sbufWriteU8(dst, FC_VERSION_MINOR);
        sbufWriteU8(dst, FC_VERSION_PATCH_LEVEL);
        break;

    case MSP_BOARD_INFO:
    {
        sbufWriteData(dst, systemConfig()->boardIdentifier, BOARD_IDENTIFIER_LENGTH);
#ifdef USE_HARDWARE_REVISION_DETECTION
        sbufWriteU16(dst, hardwareRevision);
#else
        sbufWriteU16(dst, 0); // No other build targets currently have hardware revision detection.
#endif
        sbufWriteU8(dst, 0);  // 0 == FC

        // Target capabilities (uint8)
#define TARGET_HAS_VCP 0
#define TARGET_HAS_SOFTSERIAL 1
#define TARGET_IS_UNIFIED 2
#define TARGET_HAS_FLASH_BOOTLOADER 3
#define TARGET_SUPPORTS_CUSTOM_DEFAULTS 4
#define TARGET_HAS_CUSTOM_DEFAULTS 5
#define TARGET_SUPPORTS_RX_BIND 6

        uint8_t targetCapabilities = 0;
#ifdef USE_VCP
        targetCapabilities |= BIT(TARGET_HAS_VCP);
#endif
#if defined(USE_SOFTSERIAL1) || defined(USE_SOFTSERIAL2)
        targetCapabilities |= BIT(TARGET_HAS_SOFTSERIAL);
#endif
#if defined(USE_UNIFIED_TARGET)
        targetCapabilities |= BIT(TARGET_IS_UNIFIED);
#endif
#if defined(USE_FLASH_BOOT_LOADER)
        targetCapabilities |= BIT(TARGET_HAS_FLASH_BOOTLOADER);
#endif
#if defined(USE_CUSTOM_DEFAULTS)
        targetCapabilities |= BIT(TARGET_SUPPORTS_CUSTOM_DEFAULTS);
        if (hasCustomDefaults()) {
            targetCapabilities |= BIT(TARGET_HAS_CUSTOM_DEFAULTS);
        }
#endif
#if defined(USE_RX_BIND)
        if (getRxBindSupported()) {
            targetCapabilities |= BIT(TARGET_SUPPORTS_RX_BIND);
        }
#endif

        sbufWriteU8(dst, targetCapabilities);

        // Target name with explicit length
        sbufWriteU8(dst, strlen(targetName));
        sbufWriteData(dst, targetName, strlen(targetName));

#if defined(USE_BOARD_INFO)
        // Board name with explicit length
        char *value = getBoardName();
        sbufWriteU8(dst, strlen(value));
        sbufWriteString(dst, value);

        // Board design with explicit length
        value = getBoardDesign();
        sbufWriteU8(dst, strlen(value));
        sbufWriteString(dst, value);

        // Manufacturer id with explicit length
        value = getManufacturerId();
        sbufWriteU8(dst, strlen(value));
        sbufWriteString(dst, value);
#else
        sbufWriteU8(dst, 0);
        sbufWriteU8(dst, 0);
#endif

#if defined(USE_SIGNATURE)
        // Signature
        sbufWriteData(dst, getSignature(), SIGNATURE_LENGTH);
#else
        uint8_t emptySignature[SIGNATURE_LENGTH];
        memset(emptySignature, 0, sizeof(emptySignature));
        sbufWriteData(dst, &emptySignature, sizeof(emptySignature));
#endif

        sbufWriteU8(dst, getMcuTypeId());

        // Added in API version 1.42
        sbufWriteU8(dst, systemConfig()->configurationState);

        // Added in API version 1.43
        sbufWriteU16(dst, gyro.sampleRateHz); // informational so the configurator can display the correct gyro/pid frequencies in the drop-down

        // Configuration warnings / problems (uint32_t)
#define PROBLEM_ACC_NEEDS_CALIBRATION 0
#define PROBLEM_MOTOR_PROTOCOL_DISABLED 1

        uint32_t configurationProblems = 0;

#if defined(USE_ACC)
        if (!accHasBeenCalibrated()) {
            configurationProblems |= BIT(PROBLEM_ACC_NEEDS_CALIBRATION);
        }
#endif

        if (!checkMotorProtocolEnabled(&motorConfig()->dev)) {
            configurationProblems |= BIT(PROBLEM_MOTOR_PROTOCOL_DISABLED);
        }

        sbufWriteU32(dst, configurationProblems);

        // Added in MSP API 1.44
#if defined(USE_SPI)
        sbufWriteU8(dst, spiGetRegisteredDeviceCount());
#else
        sbufWriteU8(dst, 0);
#endif
#if defined(USE_I2C)
        sbufWriteU8(dst, i2cGetRegisteredDeviceCount());
#else
        sbufWriteU8(dst, 0);
#endif

        break;
    }

    case MSP_BUILD_INFO:
        sbufWriteData(dst, buildDate, BUILD_DATE_LENGTH);
        sbufWriteData(dst, buildTime, BUILD_TIME_LENGTH);
        sbufWriteData(dst, shortGitRevision, GIT_SHORT_REVISION_LENGTH);
        const char *version = FC_VERSION_STRING;
        sbufWriteU8(dst, strlen(version));
        sbufWriteString(dst, version);
        break;

    case MSP_ANALOG:
        sbufWriteU8(dst, (uint8_t)constrain(getLegacyBatteryVoltage(), 0, UINT8_MAX));
        sbufWriteU16(dst, (uint16_t)constrain(getBatteryCapacityUsed(), 0, UINT16_MAX));
        sbufWriteU16(dst, getRssi());
        sbufWriteU16(dst, (uint16_t)constrain(getBatteryCurrent(), 0, UINT16_MAX));
        sbufWriteU16(dst, (uint16_t)constrain(getBatteryVoltage(), 0, UINT16_MAX));
        break;

    case MSP_DEBUG:
        for (int i = 0; i < DEBUG_VALUE_COUNT; i++) {
            sbufWriteU32(dst, debug[i]);
        }
        break;

    case MSP_UID:
        sbufWriteU32(dst, U_ID_0);
        sbufWriteU32(dst, U_ID_1);
        sbufWriteU32(dst, U_ID_2);
        break;

#ifdef USE_ESC_SENSOR
    case MSP_ESC_SENSOR_CONFIG:
        sbufWriteU8(dst, escSensorConfig()->protocol);
        sbufWriteU8(dst, escSensorConfig()->halfDuplex);
        sbufWriteU16(dst, escSensorConfig()->update_hz);
        sbufWriteU16(dst, escSensorConfig()->current_offset);
        sbufWriteU8(dst, escSensorConfig()->pinSwap);
        sbufWriteS8(dst, escSensorConfig()->voltage_correction);
        sbufWriteS8(dst, escSensorConfig()->current_correction);
        sbufWriteS8(dst, escSensorConfig()->consumption_correction);
        break;

    case MSP_ESC_PARAMETERS:
        {
            const uint8_t len = escGetParamBufferLength();
            if (len == 0)
                return false;

            sbufWriteData(dst, escGetParamBuffer(), len);
        }
        break;
#endif

    case MSP_BATTERY_STATE:
        sbufWriteU8(dst, getBatteryState());
        sbufWriteU8(dst, getBatteryCellCount());
        sbufWriteU16(dst, getBatteryCapacity());  // mAh
        sbufWriteU16(dst, constrain(getBatteryCapacityUsed(), 0, UINT16_MAX));      // mAh
        sbufWriteU16(dst, getBatteryVoltage());                                     // 10mV steps
        sbufWriteU16(dst, constrain(getBatteryCurrent(), 0, UINT16_MAX));           // 10mA steps
        sbufWriteU8(dst, getBatteryChargeLevel());                                  // %
        sbufWriteU8(dst, batteryConfig()->batteryProfile); // The battery profile
        break;

    case MSP_VOLTAGE_METERS:
        // write out id and voltage meter values, once for each meter we support
        for (int i = 0; i < voltageMeterCount; i++) {
            uint8_t id = voltageMeterIds[i];
            voltageMeter_t meter;
            if (voltageMeterRead(id, &meter)) {
                sbufWriteU8(dst, id);
                sbufWriteU16(dst, constrain(meter.voltage, 0, UINT16_MAX));  // mV
            }
        }
        break;

    case MSP_CURRENT_METERS: {
        // write out id and current meter values, once for each meter we support
        for (int i = 0; i < currentMeterCount; i++) {
            uint8_t id = currentMeterIds[i];
            currentMeter_t meter;
            if (currentMeterRead(id, &meter)) {
                sbufWriteU8(dst, id);
                sbufWriteU16(dst, constrain(meter.current / 10, 0, UINT16_MAX));    // 10mA steps
                sbufWriteU16(dst, constrain(meter.capacity, 0, UINT16_MAX));        // mAh consumed
            }
        }
        break;
    }

    case MSP_EXPERIMENTAL:
        /*
         * Send your experimental parameters to LUA. Like:
         *
         * sbufWriteU8(dst, currentPidProfile->yourFancyParameterA);
         * sbufWriteU8(dst, currentPidProfile->yourFancyParameterB);
         */
        break;

    default:
        return false;
    }
    return true;
}

#ifdef USE_TUNE_ADVISOR
static uint16_t mspSatU16(uint32_t value)
{
    return MIN(value, (uint32_t)UINT16_MAX);
}

// Ratio x1000 as a signed 16-bit value on the wire
static uint16_t mspRatio(float value)
{
    return (uint16_t)(int16_t)constrain(lrintf(value * 1000), INT16_MIN, INT16_MAX);
}

// A value x scale in one byte, unsigned or signed, saturated to the byte's range
static uint8_t mspScaledU8(float value, float scale)
{
    return (uint8_t)constrain(lrintf(value * scale), 0, UINT8_MAX);
}

static uint8_t mspScaledS8(float value, float scale)
{
    return (uint8_t)(int8_t)constrain(lrintf(value * scale), INT8_MIN, INT8_MAX);
}

// P, F and B (0..PID_GAIN_MAX) in 10 bits each: bits 0-9, 10-19 and 20-29
static uint32_t mspPackGains(uint16_t p, uint16_t f, uint16_t b)
{
    return (uint32_t)MIN(p, 1023) | ((uint32_t)MIN(f, 1023) << 10) | ((uint32_t)MIN(b, 1023) << 20);
}
#endif

static bool mspProcessOutCommand(int16_t cmdMSP, sbuf_t *dst)
{
    bool unsupportedCommand = false;

    switch (cmdMSP) {
    case MSP_STATUS:
        {
            sbufWriteU16(dst, getTaskDeltaTimeUs(TASK_PID) * pidConfig()->pid_process_denom);
            sbufWriteU16(dst, getTaskDeltaTimeUs(TASK_GYRO));

            sbufWriteU16(dst, sensors(SENSOR_ACC) << 0 |
                              sensors(SENSOR_BARO) << 1 |
                              sensors(SENSOR_MAG) << 2 |
                              sensors(SENSOR_GPS) << 3 |
                              sensors(SENSOR_GYRO) << 5);

            boxBitmask_t flightModeFlags;
            packFlightModeFlags(&flightModeFlags);
            sbufWriteData(dst, &flightModeFlags, 4);

            sbufWriteU8(dst, 0); // compat: profile number

            sbufWriteU16(dst, getMaxRealTimeLoad());
            sbufWriteU16(dst, getAverageCPULoad());

            sbufWriteU8(dst, 0); // compat: extra flight mode flags count

            sbufWriteU8(dst, ARMING_DISABLE_FLAGS_COUNT);
            sbufWriteU32(dst, getArmingDisableFlags());

            sbufWriteU8(dst, getRebootRequired());
            sbufWriteU8(dst, systemConfig()->configurationState);

            sbufWriteU8(dst, getCurrentPidProfileIndex());
            sbufWriteU8(dst, PID_PROFILE_COUNT);
            sbufWriteU8(dst, getCurrentControlRateProfileIndex());
            sbufWriteU8(dst, CONTROL_RATE_PROFILE_COUNT);

            sbufWriteU8(dst, getMotorCount());
#ifdef USE_SERVOS
            // Check if bus servos are actually configured
            if (hasBusServosConfigured()) {
                // When bus servos are configured, report configured PWM servos + all bus servos
                sbufWriteU8(dst, getServoCount() + BUS_SERVO_CHANNELS);
            } else {
                // When bus servos are not configured, only report PWM servos
                sbufWriteU8(dst, getServoCount());
            }
#else
            sbufWriteU8(dst, 0);
#endif
            sbufWriteU8(dst, getGyroDetectionFlags());
        }
        break;

    case MSP_RAW_IMU:
        {
#if defined(USE_ACC)
            // Hack scale due to choice of units for sensor data in multiwii

            float scale;
            if (acc.dev.acc_1G == 2731){
                scale = 16/3.0;
            } else if (acc.dev.acc_1G > 512 * 4) {
                scale = 8;
            } else if (acc.dev.acc_1G > 512 * 2) {
                scale = 4;
            } else if (acc.dev.acc_1G >= 512) {
                scale = 2;
            } else {
                scale = 1;
            }
#endif

            for (int i = 0; i < 3; i++) {
#if defined(USE_ACC)
                sbufWriteU16(dst, lrintf(acc.accADC[i] / scale));
#else
                sbufWriteU16(dst, 0);
#endif
            }
            for (int i = 0; i < 3; i++) {
                sbufWriteU16(dst, gyroRateDps(i));
            }
            for (int i = 0; i < 3; i++) {
#if defined(USE_MAG)
                sbufWriteU16(dst, lrintf(mag.magADC[i]));
#else
                sbufWriteU16(dst, 0);
#endif
            }
        }
        break;

#ifdef USE_SERVOS
    case MSP_SERVO:
        // Check if bus servos are actually configured
        if (hasBusServosConfigured()) {
            // When bus servos are configured, send configured PWM servo outputs + all bus servo outputs
            // Skip unconfigured PWM servos between getServoCount() and BUS_SERVO_OFFSET
            const uint8_t pwmServoCount = getServoCount();
            
            // Send configured PWM servo outputs (S1-Sn where n = getServoCount())
            for (int i = 0; i < pwmServoCount; i++) {
                sbufWriteU16(dst, getServoOutput(i));
            }
            
            // Send all bus servo outputs (S9-S26)
            // Note: Unconfigured PWM servo outputs between pwmServoCount and BUS_SERVO_OFFSET are skipped
            for (int i = BUS_SERVO_OFFSET; i < BUS_SERVO_OFFSET + BUS_SERVO_CHANNELS; i++) {
                sbufWriteU16(dst, getServoOutput(i));
            }
        } else {
            // When bus servos are not configured, send all servo outputs
            for (int i = 0; i < MAX_SUPPORTED_SERVOS; i++) {
                sbufWriteU16(dst, getServoOutput(i));
            }
        }
        break;

    case MSP_SERVO_CONFIGURATIONS:
        // Check if bus servos are actually configured
        if (hasBusServosConfigured()) {
            // When bus servos are configured, send configured PWM servos + all bus servos
            // Skip unconfigured PWM servos between getServoCount() and BUS_SERVO_OFFSET
            const uint8_t pwmServoCount = getServoCount();
            const uint8_t totalCount = pwmServoCount + BUS_SERVO_CHANNELS;
            sbufWriteU8(dst, totalCount);

            // Send configured PWM servos (S1-Sn where n = getServoCount())
            for (int i = 0; i < pwmServoCount; i++) {
                sbufWriteU16(dst, servoParams(i)->mid);
                sbufWriteU16(dst, servoParams(i)->min);
                sbufWriteU16(dst, servoParams(i)->max);
                sbufWriteU16(dst, servoParams(i)->rneg);
                sbufWriteU16(dst, servoParams(i)->rpos);
                sbufWriteU16(dst, servoParams(i)->rate);
                sbufWriteU16(dst, servoParams(i)->speed);
                sbufWriteU16(dst, servoParams(i)->flags);
            }

            // Send all bus servos (S9-S26)
            // Note: Unconfigured PWM servos between pwmServoCount and BUS_SERVO_OFFSET are skipped
            for (int i = BUS_SERVO_OFFSET; i < BUS_SERVO_OFFSET + BUS_SERVO_CHANNELS; i++) {
                sbufWriteU16(dst, servoParams(i)->mid);
                sbufWriteU16(dst, servoParams(i)->min);
                sbufWriteU16(dst, servoParams(i)->max);
                sbufWriteU16(dst, servoParams(i)->rneg);
                sbufWriteU16(dst, servoParams(i)->rpos);
                sbufWriteU16(dst, servoParams(i)->rate);
                sbufWriteU16(dst, servoParams(i)->speed);
                sbufWriteU16(dst, servoParams(i)->flags);
            }
        } else {
            // When bus servos are not configured, only send PWM servo configs
            sbufWriteU8(dst, getServoCount());

            for (int i = 0; i < getServoCount(); i++) {
                sbufWriteU16(dst, servoParams(i)->mid);
                sbufWriteU16(dst, servoParams(i)->min);
                sbufWriteU16(dst, servoParams(i)->max);
                sbufWriteU16(dst, servoParams(i)->rneg);
                sbufWriteU16(dst, servoParams(i)->rpos);
                sbufWriteU16(dst, servoParams(i)->rate);
                sbufWriteU16(dst, servoParams(i)->speed);
                sbufWriteU16(dst, servoParams(i)->flags);
            }
        }
        break;

    case MSP_SERVO_TRIM:
        // U8 count, then one S16 per servo of the live, runtime-only trim in us from
        // continuous SERVO_TRIM_* adjustments (never saved), then one S16 per servo of
        // the saved trim (servoTrims PG, from stepped SERVO_TRIM_* and AUTO TRIM). Same
        // servo indexing/remap shape as MSP_SERVO_CONFIGURATIONS above. The saved block
        // is appended so a client that only reads the runtime block still works.
        {
            const uint8_t pwmServoCount = getServoCount();
            const bool busServos = hasBusServosConfigured();
            const uint8_t count = busServos ? pwmServoCount + BUS_SERVO_CHANNELS : pwmServoCount;

            sbufWriteU8(dst, count);
            for (int n = 0; n < count; n++) {
                const int i = (n < pwmServoCount) ? n : BUS_SERVO_OFFSET + (n - pwmServoCount);
                sbufWriteU16(dst, (int16_t)lrintf(getServoRuntimeTrim(i)));
            }
            for (int n = 0; n < count; n++) {
                const int i = (n < pwmServoCount) ? n : BUS_SERVO_OFFSET + (n - pwmServoCount);
                sbufWriteU16(dst, (int16_t)getServoSavedTrim(i));
            }
        }
        break;

    case MSP_SERVO_CURVES:
        // Same servo indexing/remap shape as MSP_SERVO_CONFIGURATIONS above.
        if (hasBusServosConfigured()) {
            const uint8_t pwmServoCount = getServoCount();
            const uint8_t totalCount = pwmServoCount + BUS_SERVO_CHANNELS;
            sbufWriteU8(dst, totalCount);

            for (int i = 0; i < pwmServoCount; i++) {
                sbufWriteU8(dst, servoCurves(i)->count);
                for (int p = 0; p < SERVO_CURVE_POINTS; p++) {
                    sbufWriteU16(dst, servoCurves(i)->points[p].x);
                    sbufWriteU16(dst, servoCurves(i)->points[p].y);
                }
            }

            for (int i = BUS_SERVO_OFFSET; i < BUS_SERVO_OFFSET + BUS_SERVO_CHANNELS; i++) {
                sbufWriteU8(dst, servoCurves(i)->count);
                for (int p = 0; p < SERVO_CURVE_POINTS; p++) {
                    sbufWriteU16(dst, servoCurves(i)->points[p].x);
                    sbufWriteU16(dst, servoCurves(i)->points[p].y);
                }
            }
        } else {
            sbufWriteU8(dst, getServoCount());

            for (int i = 0; i < getServoCount(); i++) {
                sbufWriteU8(dst, servoCurves(i)->count);
                for (int p = 0; p < SERVO_CURVE_POINTS; p++) {
                    sbufWriteU16(dst, servoCurves(i)->points[p].x);
                    sbufWriteU16(dst, servoCurves(i)->points[p].y);
                }
            }
        }
        break;

    case MSP_SERVO_OVERRIDE:
        for (int i = 0; i < MAX_SUPPORTED_SERVOS; i++) {
            sbufWriteU16(dst, getServoOverride(i));
        }
        break;
#endif

    case MSP_MOTOR:
        for (int i = 0; i < 8; i++) {
#ifdef USE_MOTOR
            if (i < getMotorCount() && motorIsEnabled() && motorIsMotorEnabled(i)) {
                const int16_t throttle = getMotorOutput(i);
                if (throttle < 0)
                    sbufWriteU16(dst, throttle - 1000);
                else
                    sbufWriteU16(dst, throttle + 1000); // compat: BLHeli
            }
            else
#endif
                sbufWriteU16(dst, 0); // zero means motor disabled
        }
        break;

    case MSP_MOTOR_OVERRIDE:
        for (int i = 0; i < MAX_SUPPORTED_MOTORS; i++) {
#ifdef USE_MOTOR
            if (i < getMotorCount())
                sbufWriteU16(dst, getMotorOverride(i));
            else
#endif
                sbufWriteU16(dst, 0);
        }
        break;

    case MSP_MOTOR_TELEMETRY:
        sbufWriteU8(dst, getMotorCount());
        for (unsigned i = 0; i < getMotorCount(); i++) {
            uint32_t motorRpm = 0;
            uint16_t errorRatio = 0;
            uint16_t escVoltage = 0;       // 1mV per unit
            uint16_t escCurrent = 0;       // 1mA per unit
            uint16_t escConsumption = 0;   // mAh
            uint16_t escTemperature = 0;   // 0.1C
            uint16_t escTemperature2 = 0;  // 0.1C

#ifdef USE_ESC_SENSOR
            if (featureIsEnabled(FEATURE_ESC_SENSOR)) {
                escSensorData_t *escData = getEscSensorData(i);
                if (escData && escData->age <= ESC_BATTERY_AGE_MAX) {
                    motorRpm = calcMotorRPM(i, escData->erpm);
                    escVoltage = escData->voltage;
                    escCurrent = escData->current;
                    escConsumption = escData->consumption;
                    escTemperature = escData->temperature;
                    escTemperature2 = escData->temperature2;
                }
            }
#endif

#ifdef USE_DSHOT_TELEMETRY
            if (motorConfig()->dev.useDshotTelemetry) {
                if (isDshotMotorTelemetryActive(i)) {
                    motorRpm = calcMotorRPM(i, getDshotTelemetry(i));
#ifdef USE_DSHOT_TELEMETRY_STATS
                    errorRatio = getDshotTelemetryMotorInvalidPercent(i);
#endif
                }
            }
#endif

#ifdef USE_FREQ_SENSOR
            if (featureIsEnabled(FEATURE_FREQ_SENSOR) && isFreqSensorPortInitialized(i)) {
                motorRpm = calcMotorRPM(i, getFreqSensorRPM(i));
            }
#endif

            if (isMotorRpmSourceActive(i))
                motorRpm = getMotorRPM(i);

            sbufWriteU32(dst, motorRpm);
            sbufWriteU16(dst, errorRatio);
            sbufWriteU16(dst, escVoltage);
            sbufWriteU16(dst, escCurrent);
            sbufWriteU16(dst, escConsumption);
            sbufWriteU16(dst, escTemperature);
            sbufWriteU16(dst, escTemperature2);
        }
        break;

    case MSP2_WING_EFFECTIVE_PID_GAINS: {
        pidRuntimeGains_t runtimeGains;
        pidGetRuntimeGains(&runtimeGains);

        sbufWriteU8(dst, 3); // payload version (3: GPS speed attenuation appended)
        sbufWriteU8(dst, currentPidProfile->pid_mode);
        sbufWriteU32(dst, runtimeGains.fwTpa);

        for (int axis = 0; axis < PID_AXIS_COUNT; axis++) {
            sbufWriteU16(dst, runtimeGains.raw[axis].P);
            sbufWriteU16(dst, runtimeGains.raw[axis].I);
            sbufWriteU16(dst, runtimeGains.raw[axis].D);
            sbufWriteU16(dst, runtimeGains.raw[axis].F);
            sbufWriteU16(dst, runtimeGains.raw[axis].B);
            sbufWriteU16(dst, runtimeGains.masterGain[axis]);
            sbufWriteU32(dst, runtimeGains.gainCurve[axis]);
            sbufWriteU32(dst, runtimeGains.gainCurvePosition[axis]);
            sbufWriteU32(dst, runtimeGains.effective[axis].P);
            sbufWriteU32(dst, runtimeGains.effective[axis].I);
            sbufWriteU32(dst, runtimeGains.effective[axis].D);
            sbufWriteU32(dst, runtimeGains.effective[axis].F);
            sbufWriteU32(dst, runtimeGains.effective[axis].B);
        }
        // v3: GPS speed attenuation, appended so v2 parsers still read the axes
        sbufWriteU32(dst, runtimeGains.fwSpa);
        sbufWriteU16(dst, runtimeGains.fwSpaSpeed);
        sbufWriteU8(dst, runtimeGains.fwSpaEnabled);
        break;
    }

    case MSP2_WING_SERVO_TIMING: {
        const uint8_t count = getServoCount();

        sbufWriteU8(dst, count);
        for (uint8_t i = 0; i < count; i++) {
            sbufWriteU16(dst, getServoRunRate(i));
            sbufWriteU8(dst, getServoTimerGroup(i));
        }
        break;
    }

    case MSP2_WING_TV_EFFECTIVE_PID_GAINS: {
        tvPidRuntimeGains_t runtimeGains;
        tvPidGetRuntimeGains(&runtimeGains);

        sbufWriteU8(dst, 1); // payload version
        sbufWriteU8(dst, getCurrentTvProfileIndex());

        for (int axis = 0; axis < PID_AXIS_COUNT; axis++) {
            sbufWriteU16(dst, runtimeGains.raw[axis].P);
            sbufWriteU16(dst, runtimeGains.raw[axis].I);
            sbufWriteU16(dst, runtimeGains.raw[axis].D);
            sbufWriteU16(dst, runtimeGains.raw[axis].F);
            sbufWriteU16(dst, runtimeGains.raw[axis].B);
            sbufWriteU16(dst, runtimeGains.masterGain[axis]);
            sbufWriteU32(dst, runtimeGains.gainCurve[axis]);
            sbufWriteU32(dst, runtimeGains.gainCurvePosition[axis]);
            sbufWriteU32(dst, runtimeGains.effective[axis].P);
            sbufWriteU32(dst, runtimeGains.effective[axis].I);
            sbufWriteU32(dst, runtimeGains.effective[axis].D);
            sbufWriteU32(dst, runtimeGains.effective[axis].F);
            sbufWriteU32(dst, runtimeGains.effective[axis].B);
        }
        break;
    }

#if defined(USE_FBUS_MASTER) || defined(USE_SPORT_MASTER)
    case MSP2_WING_FBUS_SENSORS: {
        const uint8_t count = fbusSensorGetObservedCount();
        sbufWriteU8(dst, count);

        for (uint8_t i = 0; i < count; i++) {
            const fbusObservedSensor_t *sensor = fbusSensorGetObserved(i);
            if (!sensor) {
                break;
            }

            sbufWriteU8(dst, sensor->physicalId);
            sbufWriteU8(dst, sensor->source);
            sbufWriteU8(dst, fbusSensorIsForwarded(sensor->physicalId) ? 1 : 0);
            sbufWriteU32(dst, sensor->packetCount);

            // Mirror the CLI's "ID_XXX" fallback for unnamed physical IDs so
            // both surfaces show identical sensor names.
            const char *sensorName = fbusSensorGetName(sensor->physicalId);
            char nameBuffer[17];
            if (strcmp(sensorName, "UNKNOWN") == 0) {
                tfp_sprintf(nameBuffer, "ID_%u", sensor->physicalId);
                sensorName = nameBuffer;
            }
            const uint8_t nameLen = (uint8_t)strlen(sensorName);
            sbufWriteU8(dst, nameLen);
            sbufWriteData(dst, sensorName, nameLen);

            sbufWriteU8(dst, sensor->appIdCount);
            for (uint8_t j = 0; j < sensor->appIdCount; j++) {
                sbufWriteU16(dst, sensor->appIds[j]);
            }
        }
        break;
    }

    case MSP2_WING_FBUS_SENSORS_STATUS: {
        // Latest decoded value from each sensor type the driver understands.
        // Each block starts with a flags byte: 0 means no fresh data (nothing
        // within the last second), and its fields are then sent as zero.
        sbufWriteU8(dst, 1); // payload version

        fbusGpsData_t gps;
        fbusSensorGetGpsData(&gps);
        uint8_t gpsFlags = 0;
        // Any fresh GPS field counts, not only position (which
        // fbusSensorHasGpsData() requires for navigation), so fields sent
        // before a position fix still show.
        if (gps.lastUpdateUs > 0 && cmpTimeUs(micros(), gps.lastUpdateUs) < 1000000) {
            gpsFlags = (gps.hasPosition   ? BIT(0) : 0) |
                       (gps.hasAltitude   ? BIT(1) : 0) |
                       (gps.hasSpeed      ? BIT(2) : 0) |
                       (gps.hasCourse     ? BIT(3) : 0) |
                       (gps.hasSatellites ? BIT(4) : 0) |
                       (gps.hasTime       ? BIT(5) : 0) |
                       (gps.hasDate       ? BIT(6) : 0);
        }
        if (!gpsFlags) {
            memset(&gps, 0, sizeof(gps));
        }
        sbufWriteU8(dst, gpsFlags);
        sbufWriteS32(dst, gps.latitude);
        sbufWriteS32(dst, gps.longitude);
        sbufWriteS32(dst, gps.altitudeCm);
        sbufWriteU32(dst, gps.speedMilliKnots);
        sbufWriteU16(dst, gps.courseDeg);
        sbufWriteU8(dst, gps.satellites);
        sbufWriteU8(dst, gps.time.hours);
        sbufWriteU8(dst, gps.time.minutes);
        sbufWriteU8(dst, gps.time.seconds);
        sbufWriteU8(dst, gps.date.day);
        sbufWriteU8(dst, gps.date.month);
        sbufWriteU16(dst, gps.date.year);

        fbusCurrentData_t current;
        fbusSensorGetCurrentData(&current);
        uint8_t currentFlags = 0;
        if (fbusSensorHasCurrentData()) {
            currentFlags = (current.hasCurrent              ? BIT(0) : 0) |
                           (current.hasVoltage              ? BIT(1) : 0) |
                           (current.hasHighPrecisionCurrent ? BIT(2) : 0);
        }
        if (!currentFlags) {
            memset(&current, 0, sizeof(current));
        }
        sbufWriteU8(dst, currentFlags);
        sbufWriteU32(dst, current.currentDeciAmps);
        sbufWriteU32(dst, current.voltageCentiVolts);
        sbufWriteU16(dst, current.currentMilliAmps);

        fbusFlvssData_t flvss;
        fbusSensorGetFlvssData(&flvss);
        const bool hasFlvss = fbusSensorHasFlvssData();
        const uint8_t cellCount = hasFlvss ? MIN(flvss.cellCount, FBUS_FLVSS_MAX_CELLS) : 0;
        sbufWriteU8(dst, hasFlvss ? BIT(0) : 0);
        sbufWriteU8(dst, cellCount);
        for (uint8_t i = 0; i < cellCount; i++) {
            sbufWriteU16(dst, flvss.cellVoltageCentiVolts[i]);
        }
        sbufWriteU32(dst, hasFlvss ? flvss.voltageCentiVolts : 0);

        // Shared by FrSky ESC sensors and the FrSky RPM sensor
        fbusEscData_t esc;
        fbusSensorGetEscData(&esc);
        uint8_t escFlags = 0;
        if (fbusSensorHasEscData()) {
            escFlags = (esc.hasPower       ? BIT(0) : 0) |
                       (esc.hasRpm         ? BIT(1) : 0) |
                       (esc.hasConsumption ? BIT(2) : 0) |
                       (esc.hasTemperature ? BIT(3) : 0);
        }
        if (!escFlags) {
            memset(&esc, 0, sizeof(esc));
        }
        sbufWriteU8(dst, escFlags);
        sbufWriteU16(dst, esc.voltageCentiVolts);
        sbufWriteU16(dst, esc.currentCentiAmps);
        sbufWriteU32(dst, esc.erpm);
        sbufWriteU16(dst, esc.consumptionMah);
        sbufWriteU8(dst, esc.temperatureDegC);
        sbufWriteU8(dst, esc.temperature2DegC);

        fbusServoData_t servo;
        fbusSensorGetServoData(&servo);
        const bool hasServo = fbusSensorHasServoData();
        if (!hasServo) {
            memset(&servo, 0, sizeof(servo));
        }
        sbufWriteU8(dst, hasServo ? BIT(0) : 0);
        sbufWriteU16(dst, servo.currentDeciAmps);
        sbufWriteU16(dst, servo.voltageDeciVolts);
        sbufWriteU16(dst, servo.temperatureDegC);
        break;
    }

#endif

#ifdef USE_SRXL2_ESC
    case MSP2_WING_SRXL2_ESC_CONFIG:
        // Payload version 1: the SRXL2 ESC bus settings (pg/srxl2_esc.h).
        // throttle_rate_hz is the rate in effect (the driver writes it back from
        // the motor PWM rate it follows); the other two are read at init, so a
        // change takes effect after the save and reboot the Motors tab already does.
        sbufWriteU8(dst, 1); // payload version
        sbufWriteU16(dst, srxl2escConfig()->throttle_rate_hz);
        sbufWriteU8(dst, srxl2escConfig()->telem_interval_frames);
        sbufWriteU8(dst, srxl2escConfig()->motor2_channel); // 0 = off, 1..31
        break;
#endif

#ifdef USE_FBUS_MUX_FPGA
    case MSP2_WING_FBUS_MUX_CONFIG: {
        // U8 version, U8 channel count, U8 DShot channel mask (bit 0 = channel 1),
        // U8 configuration UART (fpga_serial, 1-based, 0 = none), U8 status
        // (fbusMuxFpgaStatus_e), U32 mode word sent to the FPGA, then per channel
        // U8 configured mode and U8 mode in use (fbusMuxMode_e), then U8 override held
        // (MSP2_WING_SET_FBUS_MUX_OVERRIDE). The mode in use is the override's while one
        // is held, and PWM where the requested mode cannot work, see buildModeWord().
        // Polled first: ending a lapsed override changes the status, word and modes.
        const bool overrideActive = fbusMuxFpgaIsOverrideActive();
        sbufWriteU8(dst, 1);
        sbufWriteU8(dst, FBUS_MUX_FPGA_CHANNELS);
        sbufWriteU8(dst, fbusMuxFpgaDshotChannelMask);
        sbufWriteU8(dst, fbusMuxFpgaConfig()->serialPort);
        sbufWriteU8(dst, fbusMuxFpgaGetStatus());
        sbufWriteU32(dst, fbusMuxFpgaGetModeWord());
        for (int ch = 0; ch < FBUS_MUX_FPGA_CHANNELS; ch++) {
            sbufWriteU8(dst, fbusMuxFpgaConfig()->mode[ch]);
            sbufWriteU8(dst, fbusMuxFpgaGetChannelMode(ch));
        }
        sbufWriteU8(dst, overrideActive ? 1 : 0);
        break;
    }
#endif

#ifdef USE_RX_INPUT_BACKUP
    case MSP2_WING_RX_INPUT_BACKUP_STATUS: {
        // Read-only diagnostics for the configurator/Lua suite: is a backup RX
        // port configured (and with which protocol), is it currently healthy, and
        // is it the channel source in use right now (main RX link down, backup
        // covering for it)?
        //
        // Payload version 2: adds the `provider` byte (right after `enabled`, since
        // it's static config known even while disabled) now that this feature is no
        // longer SBUS-only. Both the configurator's and Lua suite's decoders now
        // actually branch on this byte (version < 2 => no provider field present,
        // assume SBUS) rather than reading-and-discarding it as v1's clients did -
        // that dead-code version check is exactly what let this field get added at
        // all without also minting a new command id.
        //
        // Payload version 3: adds `mainLinkUp` (right after `enabled`) - the main
        // RX's own live signal-received state, previously only inferable indirectly
        // via `activeSource` (which only flips to "backup" once the backup is BOTH
        // linked AND actually needed, so it can't distinguish "main is fine" from
        // "main is down but backup isn't up either"). This command already computed
        // rxIsReceivingSignal() internally for that purpose; surfacing it directly
        // lets the configurator show a genuine main-link status badge, not just a
        // backup one - unconditional, not gated on `enabled`, since it's meaningful
        // whether or not a backup port is even configured.
        //
        // Payload version 4: there are now RX_INPUT_BACKUP_COUNT backup ports. The
        // fields above keep describing backup port 0 (the one every older client
        // knew as "the backup"), except `activeSource`, which becomes the 1-based
        // index of the backup port in control (0 = no backup in control: the main
        // RX is flying, or every link is down - `mainLinkUp` tells which) - older
        // clients only test it for non-zero, so that still reads as "a backup is
        // in control".
        // Appended after the port-0 channels: `portCount`, then for each further
        // port: enabled, provider, linkUp, channelCount, channelCount x U16.
        const bool mainLinkUp = rxIsReceivingSignal();
        const int8_t activeIndex = rxInputBackupGetActiveIndex();
        const uint8_t activeSource = (!mainLinkUp && activeIndex >= 0) ? (uint8_t)(activeIndex + 1) : 0;

        sbufWriteU8(dst, 4); // payload version
        for (uint8_t port = 0; port < RX_INPUT_BACKUP_COUNT; port++) {
            const bool enabled = rxInputBackupPortIsEnabled(port);
            const bool linkUp = enabled && rxInputBackupPortIsActive(port);
            // Every main RX channel gets a backup value; channels the backup's
            // latest frame doesn't carry report stick center, the value a takeover
            // would apply (rxInputBackupPortGetChannel()).
            const uint8_t channelCount = enabled ? MAX(rxInputBackupPortGetChannelCount(port), activeRcChannelCount) : 0;

            sbufWriteU8(dst, enabled ? 1 : 0);
            if (port == 0) {
                sbufWriteU8(dst, mainLinkUp ? 1 : 0);
            }
            sbufWriteU8(dst, enabled ? rxInputBackupPortGetProvider(port) : 0);
            sbufWriteU8(dst, linkUp ? 1 : 0);
            if (port == 0) {
                sbufWriteU8(dst, activeSource); // 0 = no backup in control, n = backup n active
            }
            sbufWriteU8(dst, channelCount);
            for (uint8_t i = 0; i < channelCount; i++) {
                sbufWriteU16(dst, (uint16_t)lrintf(rxInputBackupPortGetChannel(port, i)));
            }
            if (port == 0) {
                sbufWriteU8(dst, RX_INPUT_BACKUP_COUNT); // ports that follow, including this one
            }
        }
        break;
    }

#endif

#ifdef USE_CRSF_SENSORS
    case MSP2_WING_CRSF_SENSORS_STATUS: {
        // Read-only diagnostics for the configurator's CRSF Sensors debug page
        // (mirrors MSP2_WING_FBUS_SENSORS' role for the FBUS/S.Port bus): link
        // health counters plus the latest decoded value from each sensor type
        // this driver understands.
        crsfSensorsDebugStats_t stats;
        crsfSensorsGetDebugStats(&stats);

        sbufWriteU8(dst, 1); // payload version
        sbufWriteU8(dst, crsfSensorsIsEnabled() ? 1 : 0);
        sbufWriteU32(dst, stats.rxByteCount);
        sbufWriteU32(dst, stats.rxSyncCount);
        sbufWriteU32(dst, stats.rxCrcOkCount);
        sbufWriteU32(dst, stats.rxCrcFailCount);
        sbufWriteU8(dst, stats.lastFrameType);
        sbufWriteU8(dst, stats.lastFrameLength);

        crsfSensorsGpsData_t gps;
        const bool hasGps = crsfSensorsGetGpsData(&gps);
        sbufWriteU8(dst, hasGps ? 1 : 0);
        sbufWriteS32(dst, hasGps ? gps.latitude : 0);
        sbufWriteS32(dst, hasGps ? gps.longitude : 0);
        sbufWriteU16(dst, hasGps ? gps.groundspeedCmS : 0);
        sbufWriteU16(dst, hasGps ? gps.headingDeg10 : 0);
        sbufWriteS32(dst, hasGps ? gps.altitudeCm : 0);
        sbufWriteU8(dst, hasGps ? gps.satellites : 0);

        crsfSensorsBatteryData_t battery;
        const bool hasBattery = crsfSensorsGetBatteryData(&battery);
        sbufWriteU8(dst, hasBattery ? 1 : 0);
        sbufWriteU32(dst, hasBattery ? battery.voltageMv : 0);
        sbufWriteU32(dst, hasBattery ? battery.currentMa : 0);
        sbufWriteU32(dst, hasBattery ? battery.capacityMah : 0);
        sbufWriteU8(dst, hasBattery ? battery.remainingPct : 0);

        crsfSensorsBaroData_t baro;
        const bool hasBaro = crsfSensorsGetBaroData(&baro);
        sbufWriteU8(dst, hasBaro ? 1 : 0);
        sbufWriteS32(dst, hasBaro ? baro.altitudeCm : 0);
        sbufWriteS16(dst, hasBaro ? baro.verticalSpeedCmS : 0);

        crsfSensorsCellsData_t cells;
        const bool hasCells = crsfSensorsGetCellsData(&cells);
        sbufWriteU8(dst, hasCells ? 1 : 0);
        sbufWriteU8(dst, hasCells ? cells.cellCount : 0);
        if (hasCells) {
            for (uint8_t i = 0; i < cells.cellCount; i++) {
                sbufWriteU16(dst, cells.cellVoltageMv[i]);
            }
        }
        sbufWriteU32(dst, hasCells ? cells.totalVoltageMv : 0);

        crsfSensorsRpmData_t rpm;
        const bool hasRpm = crsfSensorsGetRpmData(&rpm);
        sbufWriteU8(dst, hasRpm ? 1 : 0);
        sbufWriteU8(dst, hasRpm ? rpm.rpmCount : 0);
        if (hasRpm) {
            for (uint8_t i = 0; i < rpm.rpmCount; i++) {
                sbufWriteS32(dst, rpm.rpmValues[i]);
            }
        }
        break;
    }
#endif

    case MSP_RC:
        for (int i = 0; i < activeRcChannelCount; i++) {
            sbufWriteU16(dst, (int16_t)rcInput[i]);
        }
        break;

    case MSP_RC_COMMAND:
        for (int i = 0; i < CONTROL_CHANNEL_COUNT; i++) {
            // Try to round more "visually correct"
            sbufWriteU16(dst, (int16_t)roundf(rcCommand[i]));
        }
        break;

    case MSP_RX_CHANNELS:
        for (int i = 0; i < activeRcChannelCount; i++) {
            sbufWriteU16(dst, (int16_t)rcRawChannel[i]);
        }
        break;

    case MSP_SETPOINT:
        for (int i = 0; i < 3; i++) {
            sbufWriteS16(dst, lrintf(getSetpoint(i) * 10));
        }
        break;

    case MSP_ATTITUDE:
        sbufWriteU16(dst, attitude.values.roll);
        sbufWriteU16(dst, attitude.values.pitch);
        sbufWriteU16(dst, DECIDEGREES_TO_DEGREES(attitude.values.yaw));
        break;

    case MSP_ALTITUDE:
        sbufWriteU32(dst, getEstimatedAltitudeCm());
#ifdef USE_VARIO
        sbufWriteU16(dst, getEstimatedVarioCms());
#else
        sbufWriteU16(dst, 0);
#endif
        break;

    case MSP_MODE_RANGES:
        for (int i = 0; i < MAX_MODE_ACTIVATION_CONDITION_COUNT; i++) {
            const modeActivationCondition_t *mac = modeActivationConditions(i);
            const box_t *box = findBoxByBoxId(mac->modeId);
            if (box) {
                sbufWriteU8(dst, box->permanentId);
                sbufWriteU8(dst, mac->auxChannelIndex);
                sbufWriteU8(dst, mac->range.startStep);
                sbufWriteU8(dst, mac->range.endStep);
            } else {
                // Saved range for a mode this build no longer has: report an
                // empty slot (start == end), which clients skip.
                sbufWriteU8(dst, 0);
                sbufWriteU8(dst, 0);
                sbufWriteU8(dst, 0);
                sbufWriteU8(dst, 0);
            }
        }
        break;

    case MSP_MODE_RANGES_EXTRA:
        sbufWriteU8(dst, MAX_MODE_ACTIVATION_CONDITION_COUNT);          // prepend number of EXTRAs array elements

        for (int i = 0; i < MAX_MODE_ACTIVATION_CONDITION_COUNT; i++) {
            const modeActivationCondition_t *mac = modeActivationConditions(i);
            const box_t *box = findBoxByBoxId(mac->modeId);
            const box_t *linkedBox = findBoxByBoxId(mac->linkedTo);
            sbufWriteU8(dst, box ? box->permanentId : 0);     // each element is aligned with MODE_RANGES by the permanentId
            sbufWriteU8(dst, mac->modeLogic);
            sbufWriteU8(dst, linkedBox ? linkedBox->permanentId : 0);
        }
        break;

    case MSP_MOTOR_CONFIG:
        sbufWriteU16(dst, motorConfig()->minthrottle);
        sbufWriteU16(dst, motorConfig()->maxthrottle);
        sbufWriteU16(dst, motorConfig()->mincommand);

        sbufWriteU8(dst, getMotorCount()); // compat: BLHeliSuite
        sbufWriteU8(dst, motorConfig()->motorPoleCount[0]); // compat: BLHeliSuite

#ifdef USE_DSHOT_TELEMETRY
        sbufWriteU8(dst, motorConfig()->dev.useDshotTelemetry);
#else
        sbufWriteU8(dst, 0);
#endif
        sbufWriteU8(dst, motorConfig()->dev.motorPwmProtocol);
        sbufWriteU16(dst, motorConfig()->dev.motorPwmRate);
        sbufWriteU8(dst, motorConfig()->dev.useUnsyncedPwm);

        for (int i = 0; i < 4; i++)
            sbufWriteU8(dst, motorConfig()->motorPoleCount[i]);
        for (int i = 0; i < 4; i++)
            sbufWriteU8(dst, motorConfig()->motorRpmLpf[i]);

        sbufWriteU16(dst, motorConfig()->motor1GearRatio[0]);
        sbufWriteU16(dst, motorConfig()->motor1GearRatio[1]);
        sbufWriteU16(dst, motorConfig()->motor2GearRatio[0]);
        sbufWriteU16(dst, motorConfig()->motor2GearRatio[1]);
        break;

#ifdef USE_GPS
    case MSP_RAW_GPS:
        sbufWriteU8(dst, STATE(GPS_FIX));
        sbufWriteU8(dst, gpsSol.numSat);
        sbufWriteU32(dst, gpsSol.llh.lat);
        sbufWriteU32(dst, gpsSol.llh.lon);
        sbufWriteU16(dst, (uint16_t)constrain(gpsSol.llh.altCm / 100, 0, UINT16_MAX)); // alt changed from 1m to 0.01m per lsb since MSP API 1.39 by RTH. To maintain backwards compatibility compensate to 1m per lsb in MSP again.
        sbufWriteU16(dst, gpsSol.groundSpeed);
        sbufWriteU16(dst, gpsSol.groundCourse);
        // Added in API version 1.44
        sbufWriteU16(dst, gpsSol.hdop);
        break;

    case MSP_COMP_GPS:
        sbufWriteU16(dst, GPS_distanceToHome);
        sbufWriteU16(dst, GPS_directionToHome);
        sbufWriteU8(dst, GPS_update & 1);
        break;

    case MSP_GPSSVINFO:
        sbufWriteU8(dst, GPS_numCh);
       for (int i = 0; i < GPS_numCh; i++) {
           sbufWriteU8(dst, GPS_svinfo_chn[i]);
           sbufWriteU8(dst, GPS_svinfo_svid[i]);
           sbufWriteU8(dst, GPS_svinfo_quality[i]);
           sbufWriteU8(dst, GPS_svinfo_cno[i]);
       }
        break;

#endif

    case MSP_MIXER_CONFIG:
        sbufWriteU8(dst, mixerConfig()->model_type);
        sbufWriteU8(dst, busServoConfig()->cloneFromPwm);
        // API 22.5: bus output channel counts (8, 12, 16 or 24; 0 when not
        // built), then the count the configured bus output actually drives
#ifdef USE_SBUS_OUTPUT
        sbufWriteU8(dst, busOutChannelCount(sbusOutConfig()->channels));
#else
        sbufWriteU8(dst, 0);
#endif
#ifdef USE_FBUS_MASTER
        sbufWriteU8(dst, busOutChannelCount(fbusMasterConfig()->channels));
#else
        sbufWriteU8(dst, 0);
#endif
        sbufWriteU8(dst, getBusServoOutputCount());
        break;

    case MSP_LOGIC_CONDITIONS_STATUS:
        for (int i = 0; i < LOGIC_CONDITION_COUNT; i++) {
            sbufWriteU8(dst, logicConditionGetValue(i) ? 1 : 0);
        }
        break;

    case MSP_MIXER_OVERRIDE:
        for (int i = 0; i < MIXER_INPUT_COUNT; i++) {
            sbufWriteU16(dst, mixerGetOverride(i));
        }
        break;

    case MSP2_WING_MODE_OVERRIDE: {
        // U16 ms left before the override lapses, U8 count, count x U8 permanent box id.
        boxBitmask_t mask;
        const uint16_t remainingMs = getModeOverride(&mask);
        uint8_t ids[MODE_OVERRIDE_MAX_COUNT];
        uint8_t count = 0;
        for (int i = 0; i < CHECKBOX_ITEM_COUNT && count < MODE_OVERRIDE_MAX_COUNT; i++) {
            const box_t *box = bitArrayGet(&mask, i) ? findBoxByBoxId(i) : NULL;
            if (box) {
                ids[count++] = box->permanentId;
            }
        }
        sbufWriteU16(dst, remainingMs);
        sbufWriteU8(dst, count);
        for (int i = 0; i < count; i++) {
            sbufWriteU8(dst, ids[i]);
        }
        break;
    }

    case MSP_RXFAIL_CONFIG:
        for (int i = 0; i < activeRcChannelCount; i++) {
            sbufWriteU8(dst, rxFailsafeChannelConfigs(i)->mode);
            sbufWriteU16(dst, RXFAIL_STEP_TO_CHANNEL_VALUE(rxFailsafeChannelConfigs(i)->step));
        }
        break;

    case MSP_SERIAL_CONFIG:
        for (int i = 0; i < SERIAL_PORT_COUNT; i++) {
            if (serialIsPortAvailable(serialConfig()->portConfigs[i].identifier)) {
                sbufWriteU8(dst, serialConfig()->portConfigs[i].identifier);
                sbufWriteU32(dst, serialConfig()->portConfigs[i].functionMask);
                sbufWriteU8(dst, serialConfig()->portConfigs[i].msp_baudrateIndex);
                sbufWriteU8(dst, serialConfig()->portConfigs[i].gps_baudrateIndex);
                sbufWriteU8(dst, serialConfig()->portConfigs[i].telemetry_baudrateIndex);
                sbufWriteU8(dst, serialConfig()->portConfigs[i].blackbox_baudrateIndex);
            }
        }
        break;

    case MSP_DATAFLASH_SUMMARY:
        serializeDataflashSummaryReply(dst);
        break;

    case MSP_SDCARD_SUMMARY:
        serializeSDCardSummaryReply(dst);
        break;

    case MSP_TX_INFO:
        sbufWriteU8(dst, rssiSource);
        uint8_t rtcDateTimeIsSet = 0;
#ifdef USE_RTC_TIME
        dateTime_t dt;
        if (rtcGetDateTime(&dt)) {
            rtcDateTimeIsSet = 1;
        }
#else
        rtcDateTimeIsSet = RTC_NOT_SUPPORTED;
#endif
        sbufWriteU8(dst, rtcDateTimeIsSet);

        break;
#ifdef USE_RTC_TIME
    case MSP_RTC:
        {
            dateTime_t dt;
            if (rtcGetDateTime(&dt)) {
                sbufWriteU16(dst, dt.year);
                sbufWriteU8(dst, dt.month);
                sbufWriteU8(dst, dt.day);
                sbufWriteU8(dst, dt.hours);
                sbufWriteU8(dst, dt.minutes);
                sbufWriteU8(dst, dt.seconds);
                sbufWriteU16(dst, dt.millis);
            }
        }

        break;
#endif

#ifdef USE_FBUS_MASTER
    case MSP_XACT_SERVO_LIST:
        // List every XACT servo discovered since the last MSP_SET_XACT_SCAN. The firmware
        // reads every discovered servo's parameters in the background (not just whichever one
        // gets selected -- see fbusXactTrackServo()/xactAdvanceReadField() in fbus_xact.c), so
        // Channel is usually already available here without the GUI needing to select a servo
        // first.
        // Response format: count, then count * (phyID, appIdOffset, conflict, duplicateAppId,
        //                  ready, channel)
        // appIdOffset/channel read as 0 until ready is 1 -- see
        // fbusXactRequestParamsRead()/fbusXactIsServoParamsReady() in fbus_xact.h.
        // conflict is 1 if this Physical ID has answered with more than one App ID, meaning
        // two servos most likely share it and are colliding on the bus (see fbus_xact.h).
        // duplicateAppId is 1 if another discovered servo (a different Physical ID) shares
        // this one's App ID -- writes to either are refused until this is resolved, see
        // fbusXactHasDuplicateAppId() in fbus_xact.h.
        {
            const uint8_t count = fbusMasterIsEnabled() ? fbusXactGetDiscoveredServoCount() : 0;
            sbufWriteU8(dst, count);
            for (uint8_t i = 0; i < count; i++) {
                const uint8_t phyID = fbusXactGetDiscoveredServoPhyID(i);
                xactServoParams_t params;
                memset(&params, 0, sizeof(params));
                fbusXactGetServoParams(phyID, &params);
                sbufWriteU8(dst, phyID);
                sbufWriteU8(dst, params.appIdOffset);
                sbufWriteU8(dst, fbusXactHasServoConflict(phyID) ? 1 : 0);
                sbufWriteU8(dst, fbusXactHasDuplicateAppId(phyID) ? 1 : 0);
                sbufWriteU8(dst, fbusXactIsServoParamsReady(phyID) ? 1 : 0);
                sbufWriteU8(dst, params.channel);
            }
        }

        break;
#endif

    default:
        // the config catalogue (msp_catalogue.c)
        unsupportedCommand = !mspCatalogueProcessOutCommand(cmdMSP, dst);
    }
    return !unsupportedCommand;
}

void mspGetOptionalIndex(sbuf_t *src, const range_t *range, int *value)
{
    if (sbufBytesRemaining(src) > 0) {
        *value = constrain(sbufReadU8(src), range->min, range->max);
    }
}

void mspGetOptionalIndexRange(sbuf_t *src, const range_t *range, range_t *value)
{
    if (sbufBytesRemaining(src) > 0) {
        value->min = value->max = constrain(sbufReadU8(src), range->min, range->max);
        if (sbufBytesRemaining(src) > 0) {
            value->max = constrain(sbufReadU8(src), value->min, range->max);
        }
    }
}

static mspResult_e mspFcProcessOutCommandWithArg(mspDescriptor_t srcDesc, int16_t cmdMSP, sbuf_t *src, sbuf_t *dst, mspPostProcessFnPtr *mspPostProcessFn)
{
    switch (cmdMSP) {
    case MSP2_WING_BOARD_AUTO_ALIGN:
        if (sbufBytesRemaining(src) >= 1) {
            const uint8_t action = sbufReadU8(src);
            if (action == 1) {
                boardAutoAlignStart();
            }
        }

        {
            const boardAutoAlignStatus_t status = boardAutoAlignGetStatus();
            sbufWriteU8(dst, status.state);
            sbufWriteS16(dst, status.rollDegrees);
            sbufWriteS16(dst, status.pitchDegrees);
            sbufWriteS16(dst, status.yawDegrees);
            sbufWriteU8(dst, status.matchedSamples);
        }
        break;

    case MSP2_WING_BOARD_MOUNT_TRIM_AUTO:
        if (sbufBytesRemaining(src) >= 1) {
            const uint8_t action = sbufReadU8(src);
            if (action == 1) {
                boardMountTrimAutoStart();
            }
        }

        {
            const boardMountTrimAutoStatus_t status = boardMountTrimAutoGetStatus();
            sbufWriteU8(dst, status.state);
            sbufWriteS16(dst, status.rollTrimDecidegrees);
            sbufWriteS16(dst, status.pitchTrimDecidegrees);
            sbufWriteU8(dst, status.stabilityPercent);
        }
        break;

#ifdef USE_SERIAL_RX
    case MSP2_WING_RX_SERIAL_TRIAL:
        // action: 0 = poll only, 1 = (re)start a scan, 2 = stop/cancel - always
        // restores the pre-trial wiring and reports the resulting status either
        // way, same start/poll/action shape as MSP2_WING_BOARD_AUTO_ALIGN above.
        if (sbufBytesRemaining(src) >= 1) {
            const uint8_t action = sbufReadU8(src);
            if (action == 1) {
                rxSerialTrialStart();
            } else if (action == 2) {
                rxSerialTrialStop();
            }
        }

        {
            const rxSerialTrialStatus_t status = rxSerialTrialGetStatus();
            sbufWriteU8(dst, status.state);
            sbufWriteU8(dst, status.comboIndex);
            sbufWriteU8(dst, status.inverted);
            sbufWriteU8(dst, status.halfDuplex);
            sbufWriteU8(dst, status.pinSwap);
            sbufWriteU16(dst, status.elapsedMs);
        }
        break;
#endif

#ifdef USE_RX_INPUT_BACKUP
    case MSP2_WING_RX_INPUT_BACKUP_TRIAL:
        // Same action/status shape as MSP2_WING_RX_SERIAL_TRIAL above, applied
        // to one backup RX port's own inverted/halfDuplex/pinSwap instead. An
        // optional second request byte names the backup port (0 = backup 1,
        // the default, 1 = backup 2); the status gains that index as a trailing
        // byte, after the fields older clients read.
        if (sbufBytesRemaining(src) >= 1) {
            const uint8_t action = sbufReadU8(src);
            const uint8_t index = sbufBytesRemaining(src) >= 1 ? sbufReadU8(src) : 0;
            if (action == 1) {
                rxInputBackupTrialStart(index);
            } else if (action == 2) {
                rxInputBackupTrialStop();
            }
        }

        {
            const rxInputBackupTrialStatus_t status = rxInputBackupTrialGetStatus();
            sbufWriteU8(dst, status.state);
            sbufWriteU8(dst, status.comboIndex);
            sbufWriteU8(dst, status.inverted);
            sbufWriteU8(dst, status.halfDuplex);
            sbufWriteU8(dst, status.pinSwap);
            sbufWriteU16(dst, status.elapsedMs);
            sbufWriteU8(dst, status.index);
        }
        break;
#endif

#ifdef USE_ESC_SENSOR
    case MSP2_WING_ESC_SENSOR_TRIAL:
        // action: 0 = poll only, 1 = (re)start a scan, 2 = stop/cancel - same
        // start/poll/action shape as MSP2_WING_BOARD_AUTO_ALIGN above, and
        // the RX wiring auto-detect's MSP2_WING_RX_SERIAL_TRIAL. No
        // `inverted` field here (ESC telemetry never inverts) - just
        // halfDuplex/pinSwap.
        if (sbufBytesRemaining(src) >= 1) {
            const uint8_t action = sbufReadU8(src);
            if (action == 1) {
                escSensorTrialStart();
            } else if (action == 2) {
                escSensorTrialStop();
            }
        }

        {
            const escSensorTrialStatus_t status = escSensorTrialGetStatus();
            sbufWriteU8(dst, status.state);
            sbufWriteU8(dst, status.comboIndex);
            sbufWriteU8(dst, status.halfDuplex);
            sbufWriteU8(dst, status.pinSwap);
            sbufWriteU16(dst, status.elapsedMs);
            // Bench-diagnostic fields, temporary - appended (not inserted)
            // so the wire format stays backward compatible with the
            // original 6-byte response.
            sbufWriteU16(dst, status.frameDelta);
            sbufWriteU8(dst, status.comboCount);
            sbufWriteU8(dst, status.portOpen);
        }
        break;
#endif

#ifdef USE_RPM_FILTER
    case MSP_RPM_FILTER_V2:
        if (sbufBytesRemaining(src) == 1) {
            const uint axis = sbufReadU8(src);
            if (axis >= RPM_FILTER_AXIS_COUNT)
                return MSP_RESULT_ERROR;
            for (uint bank = 0; bank < RPM_FILTER_NOTCH_COUNT; bank++) {
                sbufWriteU8(dst, rpmFilterConfig()->custom.notch_source[axis][bank]);
                sbufWriteU16(dst, rpmFilterConfig()->custom.notch_center[axis][bank]);
                sbufWriteU8(dst, rpmFilterConfig()->custom.notch_q[axis][bank]);
            }
        }
        else {
            return MSP_RESULT_ERROR;
        }
        break;
#endif

    // Serial port diagnostics, like the CLI's serialdebug: every port, then an
    // optional detail block for the port id in the request, with one sub-block per
    // function on it that reports state (GPS for now).
    case MSP2_WING_SERIAL_STATUS: {
        const int detailId = sbufBytesRemaining(src) ? sbufReadU8(src) : 0xFF;
        sbufWriteU8(dst, 1); // payload version

        sbufWriteU8(dst, SERIAL_PORT_COUNT);
        for (int i = 0; i < SERIAL_PORT_COUNT; i++) {
            const serialPortIdentifier_e id = serialPortIdentifiers[i];
            const serialPortConfig_t *cfg = serialFindPortConfiguration(id);
            const serialPortUsage_t *usage = findSerialPortUsageByIdentifier(id);
            ioTag_t rxTag, txTag;
            uint32_t baud;
            const uint8_t flags = serialGetPortDiagnostics(id, &rxTag, &txTag, &baud);
            uint8_t options = 0;
            if (usage && usage->serialPort) {
                options = ((usage->serialPort->options & SERIAL_INVERTED) ? BIT(0) : 0) |
                          ((usage->serialPort->options & SERIAL_BIDIR) ? BIT(1) : 0) |
                          ((usage->serialPort->mode & MODE_RX) ? BIT(2) : 0) |
                          ((usage->serialPort->mode & MODE_TX) ? BIT(3) : 0);
            }
            sbufWriteU8(dst, id);
            sbufWriteU32(dst, cfg ? cfg->functionMask : 0);
            sbufWriteU8(dst, rxTag);    // DEFIO tag: ((gpio + 1) << 4) | pin, 0 = none
            sbufWriteU8(dst, txTag);
            sbufWriteU8(dst, flags);    // serialDiagFlags_e
            sbufWriteU32(dst, baud);    // 0 when closed
            sbufWriteU8(dst, options);
        }

        const serialPortConfig_t *detailCfg = detailId != 0xFF ? serialFindPortConfiguration(detailId) : NULL;
        if (!detailCfg) {
            sbufWriteU8(dst, 0xFF);     // no detail
            break;
        }
        sbufWriteU8(dst, detailId);
        sbufWriteU32(dst, baudRates[detailCfg->msp_baudrateIndex]);
        sbufWriteU32(dst, baudRates[detailCfg->gps_baudrateIndex]);
        sbufWriteU32(dst, baudRates[detailCfg->telemetry_baudrateIndex]);
        sbufWriteU32(dst, baudRates[detailCfg->blackbox_baudrateIndex]);

        uint8_t *blockCountPtr = sbufPtr(dst);
        sbufWriteU8(dst, 0);
        uint8_t blockCount = 0;

#ifdef USE_GPS
        if (detailCfg->functionMask & FUNCTION_GPS) {
            const serialPort_t *gpsSerial = gpsGetPort();
            const serialPortConfig_t *gpsCfg = findSerialPortConfig(FUNCTION_GPS);
            sbufWriteU32(dst, FUNCTION_GPS);
            uint8_t *lenPtr = sbufPtr(dst);
            sbufWriteU8(dst, 0);
            uint8_t *start = sbufPtr(dst);

            sbufWriteU8(dst, (gpsIsEnabled() ? BIT(0) : 0) |
                (gpsIsConfigured() ? BIT(1) : 0) |
                (STATE(GPS_FIX) ? BIT(2) : 0) |
                ((gpsCfg && gpsCfg->identifier == detailId) ? BIT(3) : 0) |   // this is the port GPS uses
                (gpsSerial ? BIT(4) : 0));                                      // and it is open
            sbufWriteU8(dst, gpsConfig()->provider);
            sbufWriteU32(dst, gpsSerial ? gpsSerial->baudRate : 0);
            sbufWriteU8(dst, gpsData.state);
            sbufWriteU32(dst, millis() - gpsData.lastMessage);
            sbufWriteU32(dst, gpsGetRxByteCount());
            sbufWriteU32(dst, GPS_packetCount);
            sbufWriteU32(dst, gpsData.errors);
            sbufWriteU32(dst, gpsData.timeouts);
            sbufWriteU8(dst, gpsSol.numSat);
            uint8_t recent[64];
            const uint8_t recentCount = gpsGetRecentBytes(recent, sizeof(recent));
            sbufWriteU8(dst, recentCount);
            sbufWriteData(dst, recent, recentCount);

            *lenPtr = sbufPtr(dst) - start;
            blockCount++;
        }
#endif
        *blockCountPtr = blockCount;
        break;
    }

    case MSP_BOXNAMES:
        {
            const int page = sbufBytesRemaining(src) ? sbufReadU8(src) : 0;
            serializeBoxReply(dst, page, &serializeBoxNameFn);
        }
        break;
    case MSP_BOXIDS:
        {
            const int page = sbufBytesRemaining(src) ? sbufReadU8(src) : 0;
            serializeBoxReply(dst, page, &serializeBoxPermanentIdFn);
        }
        break;
#ifdef USE_TUNE_ADVISOR
    case MSP2_WING_TUNE_ADVISOR: {
        // One axis per request (U8 axis: 0 roll, 1 pitch, 2 yaw). Payload version 2 packs each
        // value into as few bytes as the client needs: 48 bytes, within one CRSF frame (at most
        // 51: a frame holds 58 bytes after a 6-byte MSP header, and sendMspReply() sends a reply
        // that exactly fills one as two) and 11 S.Port/FBUS frames instead of 14. Counts saturate at 65535
        // (about 11 min of 100 Hz samples); scaled values saturate at their byte's range.
        if (sbufBytesRemaining(src) != 1) {
            return MSP_RESULT_ERROR;
        }
        const uint8_t axis = sbufReadU8(src);
        if (axis >= XYZ_AXIS_COUNT) {
            return MSP_RESULT_ERROR;
        }

        tuneAdvisorAxis_t ta;
        tuneAdvisorGetAxis(axis, &ta);

        sbufWriteU8(dst, 2); // payload version
        // bits 0-1 axis, bit 2 collecting now
        sbufWriteU8(dst, axis | (tuneAdvisorIsCollecting() ? 0x04 : 0));
        sbufWriteU16(dst, mspSatU16(tuneAdvisorGetValidSamples() / TA_SAMPLE_HZ));

        // The tune these numbers belong to, so a client can suggest new values
        sbufWriteU32(dst, mspPackGains(currentPidProfile->pid[axis].P, currentPidProfile->pid[axis].F,
                                       currentPidProfile->pid[axis].B));
        sbufWriteU8(dst, currentPidProfile->iterm_relax[axis]);
        sbufWriteU8(dst, currentControlRateProfile->rcRates[axis]);

        sbufWriteU16(dst, mspSatU16(ta.ffCount));
        sbufWriteU8(dst, mspScaledU8(ta.ffGain, 64));               // x64, 0..3.98
        sbufWriteU8(dst, mspScaledS8(ta.ffCorr, 100));              // x100, -1..1
        sbufWriteU8(dst, MIN(ta.ffLagMs, UINT8_MAX));               // ms, 0..250
        for (int i = 0; i < TA_SP_BAND_COUNT; i++) {
            sbufWriteU8(dst, mspScaledU8(ta.spBand[i].gain, 64));
            sbufWriteU16(dst, mspSatU16(ta.spBand[i].count));
        }
        for (int i = 0; i < TA_THR_BAND_COUNT; i++) {
            sbufWriteU8(dst, mspScaledU8(ta.thrBand[i].gain, 64));
            sbufWriteU16(dst, mspSatU16(ta.thrBand[i].count));
        }

        sbufWriteU16(dst, mspSatU16(ta.fullCount));
        sbufWriteU16(dst, mspSatU16(ta.fullSatCount));
        sbufWriteU8(dst, mspScaledU8(ta.fullRatio, 100));           // x100, 0..2.55
        sbufWriteU8(dst, mspScaledU8(ta.fullMaxRate, 0.1f));        // 10 deg/s steps, 0..2550

        sbufWriteU16(dst, ta.releases);
        sbufWriteU16(dst, ta.bigRebounds);
        sbufWriteU8(dst, mspScaledU8(ta.meanRebound, 100));         // x100, 0..2.55
        sbufWriteU8(dst, mspScaledU8(ta.meanOvershoot, 64));        // x64, 0..3.98
        sbufWriteU8(dst, mspScaledU8(ta.meanCounter, 100));         // x100, 0..2.55
        sbufWriteU16(dst, mspRatio(ta.meanIterm));                  // x1000, signed: needs the range and the 0.01s
        break;
    }
#endif

#ifdef USE_FBUS_MASTER
    case MSP_XACT_PARAMS:
        // GET all parameters of one discovered XACT servo, selected by physical ID -- pick
        // one from MSP_XACT_SERVO_LIST first. Kicks off a fresh read for it if none has
        // completed yet, so the caller should keep polling this until "ready" comes back 1.
        // A read with an unanswered field stays not ready and is retried by the next request.
        // Field set mirrors FrSky's own "XAct" ETHOS Device Config Lua script.
        // Request format: phyID
        // Response format: ready, conflict, duplicateAppId, phyID, appIdOffset, firmwareVersion,
        //                  dataRate, range, direction, pulseType, channel, center(signed),
        //                  holdingStrength, operationSmoothing, deadband, hasExtendedParams,
        //                  workingMode, maxAngle
        // duplicateAppId is 1 if another discovered servo shares this one's App ID -- see
        // fbusXactHasDuplicateAppId() in fbus_xact.h. Saving is refused while this is true,
        // unless the save is itself changing App ID to something now-unique.
        {
            xactServoParams_t params;
            memset(&params, 0, sizeof(params));
            bool ready = false;
            bool conflict = false;
            bool duplicateAppId = false;

            if (fbusMasterIsEnabled() && sbufBytesRemaining(src) >= 1) {
                const uint8_t phyID = sbufReadU8(src);
                if (fbusXactGetServoParams(phyID, &params)) {
                    ready = fbusXactIsServoParamsReady(phyID);
                    conflict = fbusXactHasServoConflict(phyID);
                    duplicateAppId = fbusXactHasDuplicateAppId(phyID);
                    if (!ready) {
                        fbusXactRequestParamsRead(phyID);
                    }
                }
            }

            sbufWriteU8(dst, ready ? 1 : 0);
            sbufWriteU8(dst, conflict ? 1 : 0);
            sbufWriteU8(dst, duplicateAppId ? 1 : 0);
            sbufWriteU8(dst, params.physicalId);          // 0x00
            sbufWriteU8(dst, params.appIdOffset);         // 0x01
            sbufWriteU8(dst, params.firmwareVersion);     // 0xFE, read-only
            sbufWriteU16(dst, params.dataRate);           // 0x02
            sbufWriteU8(dst, params.range);               // 0x04
            sbufWriteU8(dst, params.direction);           // 0x05
            sbufWriteU8(dst, params.pulseType);           // 0x06
            sbufWriteU8(dst, params.channel);             // 0x07
            sbufWriteU8(dst, (uint8_t)params.center);     // 0x08, signed -125..125
            sbufWriteU8(dst, params.holdingStrength);     // 0x11
            sbufWriteU8(dst, params.operationSmoothing);  // 0x13
            sbufWriteU8(dst, params.deadband);            // 0x21
            sbufWriteU8(dst, params.hasExtendedParams ? 1 : 0);
            sbufWriteU8(dst, params.workingMode);         // 0x40, only if hasExtendedParams
            sbufWriteU16(dst, params.maxAngle);           // 0x41, only if hasExtendedParams
        }
        break;
#endif
#ifdef USE_SERVOS
    case MSP_SET_SERVO_CONFIG:
        {
            const int rem = sbufBytesRemaining(src);
            // Expect index (U8) + eight U16 fields = 1 + 8*2 bytes
            if (rem != 1 + 8 * 2) {
                return MSP_RESULT_ERROR;
            }

            const uint8_t i = sbufReadU8(src);
            if (i >= MAX_SUPPORTED_SERVOS) {
                return MSP_RESULT_ERROR;
            }

            // Read and apply new servo parameters in the same order as GET
            servoParamsMutable(i)->mid   = sbufReadU16(src);
            servoParamsMutable(i)->min   = sbufReadU16(src);
            servoParamsMutable(i)->max   = sbufReadU16(src);
            servoParamsMutable(i)->rneg  = sbufReadU16(src);
            servoParamsMutable(i)->rpos  = sbufReadU16(src);
            servoParamsMutable(i)->rate  = sbufReadU16(src);
            servoParamsMutable(i)->speed = sbufReadU16(src);
            servoParamsMutable(i)->flags = sbufReadU16(src);

            // Validate and fix the servo configuration
            validateAndFixServoConfig();
        }
        break;
#endif
    case MSP_REBOOT:
        if (sbufBytesRemaining(src)) {
            rebootMode = sbufReadU8(src);

            if (rebootMode >= MSP_REBOOT_COUNT
#if !defined(USE_USB_MSC)
                || rebootMode == MSP_REBOOT_MSC || rebootMode == MSP_REBOOT_MSC_UTC
#endif
                ) {
                return MSP_RESULT_ERROR;
            }
        } else {
            rebootMode = MSP_REBOOT_FIRMWARE;
        }

        sbufWriteU8(dst, rebootMode);

#if defined(USE_USB_MSC)
        if (rebootMode == MSP_REBOOT_MSC || rebootMode == MSP_REBOOT_MSC_UTC) {
            if (mscCheckFilesystemReady()) {
                sbufWriteU8(dst, 1);
            } else {
                sbufWriteU8(dst, 0);

                return MSP_RESULT_ACK;
            }
        }
#endif

#if defined(USE_MSP_OVER_TELEMETRY)
        if (featureIsEnabled(FEATURE_RX_SPI) && srcDesc == getMspTelemetryDescriptor()) {
            dispatchAdd(&mspRebootEntry, MSP_DISPATCH_DELAY_US);
        } else
#endif
        if (mspPostProcessFn) {
            *mspPostProcessFn = mspRebootFn;
        }

        break;
    case MSP_MULTIPLE_MSP:
        {
            uint8_t maxMSPs = 0;
            if (sbufBytesRemaining(src) == 0) {
                return MSP_RESULT_ERROR;
            }
            int bytesRemaining = sbufBytesRemaining(dst) - 1; // need to keep one byte for checksum
            mspPacket_t packetIn, packetOut;
            sbufInit(&packetIn.buf, src->end, src->end);
            uint8_t* resetInputPtr = src->ptr;
            while (sbufBytesRemaining(src) && bytesRemaining > 0) {
                uint8_t newMSP = sbufReadU8(src);
                sbufInit(&packetOut.buf, dst->ptr, dst->end);
                packetIn.cmd = newMSP;
                mspFcProcessCommand(srcDesc, &packetIn, &packetOut, NULL);
                // Each reply's length goes in one byte, so a reply over 255 bytes (or one
                // that did not fit at all) cannot be batched; stop before it rather than
                // send a wrapped length the client would misparse.
                const int replySize = sbufPtr(&packetOut.buf) - dst->ptr;
                if (sbufWriteOverflowed(&packetOut.buf) || replySize > UINT8_MAX) {
                    break;
                }
                const int mspSize = replySize + 1; // need to add length information for each MSP
                bytesRemaining -= mspSize;
                if (bytesRemaining >= 0) {
                    maxMSPs++;
                }
            }
            src->ptr = resetInputPtr;
            sbufInit(&packetOut.buf, dst->ptr, dst->end);
            for (int i = 0; i < maxMSPs; i++) {
                uint8_t* sizePtr = sbufPtr(&packetOut.buf);
                sbufWriteU8(&packetOut.buf, 0); // dummy
                packetIn.cmd = sbufReadU8(src);
                mspFcProcessCommand(srcDesc, &packetIn, &packetOut, NULL);
                (*sizePtr) = sbufPtr(&packetOut.buf) - (sizePtr + 1);
            }
            dst->ptr = packetOut.buf.ptr;
        }
        break;

    case MSP_RESET_CONF:
        {
#if defined(USE_CUSTOM_DEFAULTS)
            defaultsType_e defaultsType = DEFAULTS_TYPE_CUSTOM;
#endif
            if (sbufBytesRemaining(src) >= 1) {
                // Added in MSP API 1.42
#if defined(USE_CUSTOM_DEFAULTS)
                defaultsType = sbufReadU8(src);
#else
                sbufReadU8(src);
#endif
            }

            bool success = false;
            if (!ARMING_FLAG(ARMED)) {
#if defined(USE_CUSTOM_DEFAULTS)
                success = resetEEPROM(defaultsType == DEFAULTS_TYPE_CUSTOM);
#else
                success = resetEEPROM(false);
#endif

                if (success && mspPostProcessFn) {
                    rebootMode = MSP_REBOOT_FIRMWARE;
                    *mspPostProcessFn = mspRebootFn;
                }
            }

            // Added in API version 1.42
            sbufWriteU8(dst, success);
        }

        break;
    default:
        // the config catalogue (msp_catalogue.c)
        return mspCatalogueProcessOutCommandWithArg(srcDesc, cmdMSP, src, dst);
    }
    return MSP_RESULT_ACK;
}

#ifdef USE_FLASHFS
static void mspFcDataFlashReadCommand(sbuf_t *dst, sbuf_t *src)
{
    const unsigned int dataSize = sbufBytesRemaining(src);
    const uint32_t readAddress = sbufReadU32(src);
    uint16_t readLength;
    bool allowCompression = false;
    bool useLegacyFormat;
    if (dataSize >= sizeof(uint32_t) + sizeof(uint16_t)) {
        readLength = sbufReadU16(src);
        if (sbufBytesRemaining(src)) {
            allowCompression = sbufReadU8(src);
        }
        useLegacyFormat = false;
    } else {
        readLength = 128;
        useLegacyFormat = true;
    }

    serializeDataflashReadReply(dst, readAddress, readLength, useLegacyFormat, allowCompression);
}
#endif

static mspResult_e mspProcessInCommand(mspDescriptor_t srcDesc, int16_t cmdMSP, sbuf_t *src)
{
    uint32_t i;
    uint8_t value;
    const unsigned int dataSize = sbufBytesRemaining(src);
    switch (cmdMSP) {
    case MSP_SELECT_SETTING:
        value = sbufReadU8(src);
        if ((value & RATEPROFILE_MASK) == 0) {
            if (value >= PID_PROFILE_COUNT) {
                value = 0;
            }
            changePidProfile(value);
        } else {
            value = value & ~RATEPROFILE_MASK;

            if (value >= CONTROL_RATE_PROFILE_COUNT) {
                value = 0;
            }
            changeControlRateProfile(value);
        }
        break;

    case MSP_COPY_PROFILE:
        value = sbufReadU8(src);        // 0 = pid profile, 1 = control rate profile
        uint8_t dstProfileIndex = sbufReadU8(src);
        uint8_t srcProfileIndex = sbufReadU8(src);
        if (value == 0) {
            pidCopyProfile(dstProfileIndex, srcProfileIndex);
            if (!ARMING_FLAG(ARMED) && dstProfileIndex == getCurrentPidProfileIndex()) {
                changePidProfile(dstProfileIndex);
            }
        }
        else if (value == 1) {
            copyControlRateProfile(dstProfileIndex, srcProfileIndex);
            if (!ARMING_FLAG(ARMED) && dstProfileIndex == getCurrentControlRateProfileIndex()) {
              changeControlRateProfile(dstProfileIndex);
            }
        }
        break;

#if defined(USE_GPS) || defined(USE_MAG)
    case MSP_SET_HEADING:
        magHold = sbufReadU16(src);
        break;
#endif

    case MSP_SET_RAW_RC:
#ifdef USE_RX_MSP
        {
            uint8_t channelCount = dataSize / sizeof(uint16_t);
            if (channelCount > MAX_SUPPORTED_RC_CHANNEL_COUNT) {
                return MSP_RESULT_ERROR;
            } else {
                uint16_t frame[MAX_SUPPORTED_RC_CHANNEL_COUNT];
                for (int i = 0; i < channelCount; i++) {
                    frame[i] = sbufReadU16(src);
                }
                rxMspFrameReceive(frame, channelCount);
            }
        }
#endif
        break;

    case MSP_SET_MODE_RANGE:
        i = sbufReadU8(src);
        if (i < MAX_MODE_ACTIVATION_CONDITION_COUNT) {
            modeActivationCondition_t *mac = modeActivationConditionsMutable(i);
            i = sbufReadU8(src);
            const box_t *box = findBoxByPermanentId(i);
            if (box) {
                mac->modeId = box->boxId;
                mac->auxChannelIndex = sbufReadU8(src);
                mac->range.startStep = sbufReadU8(src);
                mac->range.endStep = sbufReadU8(src);
                if (sbufBytesRemaining(src) != 0) {
                    mac->modeLogic = sbufReadU8(src);

                    i = sbufReadU8(src);
                    const box_t *linkedBox = findBoxByPermanentId(i);
                    mac->linkedTo = linkedBox ? linkedBox->boxId : 0;
                }
                rcControlsInit();
            } else {
                return MSP_RESULT_ERROR;
            }
        } else {
            return MSP_RESULT_ERROR;
        }
        break;

#ifdef USE_MSP_SET_MOTOR
    case MSP_SET_MOTOR:
#ifdef USE_MOTOR
        for (int i = 0; i < getMotorCount(); i++) {
            int throttle = sbufReadU16(src);
            if (motorIsEnabled() && motorIsMotorEnabled(i)) {
                if (throttle >= 1000 && throttle <= 2000)
                    setMotorOverride(i, throttle - 1000, 0);
            }
        }
#endif
        break;
#endif

    case MSP_SET_MOTOR_OVERRIDE:
#ifdef USE_MOTOR
        i = sbufReadU8(src);
        if (i >= MAX_SUPPORTED_MOTORS) {
            return MSP_RESULT_ERROR;
        }
        setMotorOverride(i, sbufReadU16(src), MOTOR_OVERRIDE_TIMEOUT);
#endif
        break;

#ifdef USE_SERVOS
    case MSP_SET_SERVO_CONFIGURATION:
        if (dataSize != 1 + 16) {
            return MSP_RESULT_ERROR;
        }
        i = sbufReadU8(src);
        
        // Check if bus servos are actually configured
        if (hasBusServosConfigured()) {
            // When bus servos are configured, map the received index to actual servo index
            // Skip unconfigured PWM servos between getServoCount() and BUS_SERVO_OFFSET
            const uint8_t pwmServoCount = getServoCount();
            const uint8_t totalCount = pwmServoCount + BUS_SERVO_CHANNELS;
            
            if (i >= totalCount) {
                return MSP_RESULT_ERROR;
            }
            
            // Map received index to actual servo index
            if (i < pwmServoCount) {
                // Configured PWM servo (S1-Sn where n = getServoCount())
                // Index stays the same (0 to pwmServoCount-1)
            } else {
                // Bus servo (S9-S26)
                // Map from sequential index to bus servo index
                i = BUS_SERVO_OFFSET + (i - pwmServoCount);
            }
        } else {
            // When bus servos are not configured, only accept PWM servo indices
            if (i >= getServoCount()) {
                return MSP_RESULT_ERROR;
            }
        }
        
        if (i >= MAX_SUPPORTED_SERVOS) {
            return MSP_RESULT_ERROR;
        }
        
        servoParamsMutable(i)->mid = sbufReadU16(src);
        servoParamsMutable(i)->min = sbufReadU16(src);
        servoParamsMutable(i)->max = sbufReadU16(src);
        servoParamsMutable(i)->rneg = sbufReadU16(src);
        servoParamsMutable(i)->rpos = sbufReadU16(src);
        servoParamsMutable(i)->rate = sbufReadU16(src);
        servoParamsMutable(i)->speed = sbufReadU16(src);
        servoParamsMutable(i)->flags = sbufReadU16(src);
        
        // Validate and fix the servo configuration
        validateAndFixServoConfig();
        break;

    case MSP_SET_SERVO_CURVE:
        i = sbufReadU8(src);

        // Same servo indexing/remap shape as MSP_SET_SERVO_CONFIGURATION above.
        if (hasBusServosConfigured()) {
            const uint8_t pwmServoCount = getServoCount();
            const uint8_t totalCount = pwmServoCount + BUS_SERVO_CHANNELS;

            if (i >= totalCount) {
                return MSP_RESULT_ERROR;
            }

            if (i >= pwmServoCount) {
                i = BUS_SERVO_OFFSET + (i - pwmServoCount);
            }
        } else {
            if (i >= getServoCount()) {
                return MSP_RESULT_ERROR;
            }
        }

        if (i >= MAX_SUPPORTED_SERVOS) {
            return MSP_RESULT_ERROR;
        }

        {
            // count is later used unchecked as an array bound by
            // evaluateCurvePoints() -- reject anything outside the wire
            // format's actual valid range, same discipline as
            // MSP_SET_MIXER_CURVE above.
            uint8_t pointCount = sbufReadU8(src);
            if (pointCount < 2 || pointCount > SERVO_CURVE_POINTS) {
                return MSP_RESULT_ERROR;
            }
            servoCurvesMutable(i)->count = pointCount;
        }
        for (int p = 0; p < SERVO_CURVE_POINTS; p++) {
            servoCurvesMutable(i)->points[p].x = sbufReadU16(src);
            servoCurvesMutable(i)->points[p].y = sbufReadU16(src);
        }
        break;

    case MSP_SET_SERVO_OVERRIDE:
        i = sbufReadU8(src);
        if (i >= MAX_SUPPORTED_SERVOS) {
            return MSP_RESULT_ERROR;
        }
        {
            // Optional U16 timeout ms (API 22.14): the override lapses unless re-sent.
            const int16_t value = sbufReadU16(src);
            if (sbufBytesRemaining(src) >= 2) {
                setServoOverrideTimed(i, value, sbufReadU16(src));
            } else {
                setServoOverride(i, value);
            }
        }
        break;

    case MSP_SET_SERVO_OVERRIDE_ALL: {
        // payload: U16 value (e.g. 0 => enable/center-focus, 2001 => disable),
        // optional U16 timeout ms (API 22.14)
        if (dataSize != 2 && dataSize != 4) {
            return MSP_RESULT_ERROR;
        }
        const uint16_t v = sbufReadU16(src);
        const uint16_t timeoutMs = (dataSize == 4) ? sbufReadU16(src) : 0;
        for (int s = 0; s < MAX_SUPPORTED_SERVOS; s++) {
            if (timeoutMs) {
                setServoOverrideTimed(s, v, timeoutMs);
            } else {
                setServoOverride(s, v);
            }
        }
        break;
    }

    case MSP2_WING_SET_SERVO_PROBE: {
        // U8 servo, S16 offset us from mid, U16 timeout ms (0 clears; otherwise clamped
        // to 500-30000). The client re-sends it before the timeout to keep the servo there.
        if (dataSize != 5) {
            return MSP_RESULT_ERROR;
        }
        const uint8_t servo = sbufReadU8(src);
        const int16_t offset = sbufReadU16(src);
        if (!setServoProbe(servo, offset, sbufReadU16(src))) {
            return MSP_RESULT_ERROR;
        }
        break;
    }

    case MSP2_WING_SET_SERVO_TRIM: {
        // U8 servo (same indexing/remap as MSP_SET_SERVO_CURVE), S16 saved trim in us.
        // Limited to SERVO_TRIM_LIMIT_PERCENT of the servo's scale, like every other
        // way of setting it. Never changes the center.
        if (dataSize != 3) {
            return MSP_RESULT_ERROR;
        }
        uint8_t servo = sbufReadU8(src);
        const int16_t trim = sbufReadU16(src);
        const uint8_t pwmServoCount = getServoCount();
        const uint8_t count = hasBusServosConfigured() ? pwmServoCount + BUS_SERVO_CHANNELS : pwmServoCount;
        if (servo >= count) {
            return MSP_RESULT_ERROR;
        }
        if (servo >= pwmServoCount) {
            servo = BUS_SERVO_OFFSET + (servo - pwmServoCount);
        }
        if (servo >= MAX_SUPPORTED_SERVOS) {
            return MSP_RESULT_ERROR;
        }
        setServoSavedTrim(servo, trim);
        break;
    }

    case MSP_SET_SERVO_CENTER:
        // payload: U8 idx + U16 mid  => 3 bytes
        if (dataSize != 1 + 2) {
            return MSP_RESULT_ERROR;
        }

        i = sbufReadU8(src);
        if (i >= MAX_SUPPORTED_SERVOS) {
            return MSP_RESULT_ERROR;
        }

        servoParamsMutable(i)->mid = sbufReadU16(src);
        
        // Validate and fix the servo configuration
        validateAndFixServoConfig();
        break;

#endif

    case MSP_SET_RESET_CURR_PID:
        resetPidProfile(currentPidProfile);
        memset(attitudeLimitsMutable(getCurrentPidProfileIndex()), 0, sizeof(attitudeLimits_t));
        resetFwSpaConfig(fwSpaConfigsMutable(getCurrentPidProfileIndex()));
        resetLevelConfig(levelConfigsMutable(getCurrentPidProfileIndex()));
        resetSnapRelaxConfig(snapRelaxConfigsMutable(getCurrentPidProfileIndex()));
        resetPropHangConfig(propHangConfigsMutable(getCurrentPidProfileIndex()));
        break;

#ifdef USE_RPM_FILTER
    case MSP_SET_RPM_FILTER_V2:
        if (sbufBytesRemaining(src) == 1 + 4 * RPM_FILTER_NOTCH_COUNT) {
            const uint axis = sbufReadU8(src);
            if (axis >= RPM_FILTER_AXIS_COUNT)
                return MSP_RESULT_ERROR;
            for (uint bank = 0; bank < RPM_FILTER_NOTCH_COUNT; bank++) {
                rpmFilterConfigMutable()->custom.notch_source[axis][bank] = sbufReadU8(src);
                rpmFilterConfigMutable()->custom.notch_center[axis][bank] = sbufReadU16(src);
                rpmFilterConfigMutable()->custom.notch_q[axis][bank] = sbufReadU8(src);
            }
            validateAndFixRPMFilterConfig();
            rpmFilterInit();
        }
        else {
            return MSP_RESULT_ERROR;
        }
        break;
#endif

#ifdef USE_ACC
    case MSP_ACC_CALIBRATION:
        if (!ARMING_FLAG(ARMED))
            accStartCalibration();
        break;
#endif

#if defined(USE_MAG)
    case MSP_MAG_CALIBRATION:
        if (!ARMING_FLAG(ARMED)) {
            compassStartCalibration();
        }
        break;
#endif

#ifdef USE_ESC_SENSOR
    case MSP_SET_ESC_SENSOR_CONFIG:
        escSensorConfigMutable()->protocol = sbufReadU8(src);
        escSensorConfigMutable()->halfDuplex = sbufReadU8(src);
        escSensorConfigMutable()->update_hz = sbufReadU16(src);
        escSensorConfigMutable()->current_offset = sbufReadU16(src);
        if (sbufBytesRemaining(src) >= 1) {
            escSensorConfigMutable()->pinSwap = sbufReadU8(src);
        }
        if (sbufBytesRemaining(src) >= 3) {
            escSensorConfigMutable()->voltage_correction = sbufReadS8(src);
            escSensorConfigMutable()->current_correction = sbufReadS8(src);
            escSensorConfigMutable()->consumption_correction = sbufReadS8(src);
        }
        break;

    case MSP_SET_ESC_PARAMETERS:
        {
            const uint8_t len = escGetParamBufferLength();
            if (len == 0)
                return MSP_RESULT_ERROR;

            sbufReadData(src, escGetParamUpdBuffer(), len);

            if (!escCommitParameters())
                return MSP_RESULT_ERROR;
        }
        break;
    
    case MSP_SET_4WIF_ESC_FWD_PROG:
        {
            if (ARMING_FLAG(ARMED)) {
                return MSP_RESULT_ERROR;
            }

            /* Expect exactly one byte: the ESC id */
            const int rem = sbufBytesRemaining(src);
            if (rem != 1) {
                return MSP_RESULT_ERROR;
            }

            uint8_t id = sbufReadU8(src);
            if (escSelect4WIfById(id) != 0) {
                return MSP_RESULT_ERROR;
            }
        }
        break;
#endif

    case MSP_EEPROM_WRITE:
        if (ARMING_FLAG(ARMED)) {
            setConfigDirty();
            return MSP_RESULT_ERROR;
        }

        // This is going to take some time and won't be done where real-time performance is needed so
        // ignore how long it takes to avoid confusing the scheduler
        schedulerIgnoreTaskStateTime();

#if defined(USE_MSP_OVER_TELEMETRY)
        if (featureIsEnabled(FEATURE_RX_SPI) && srcDesc == getMspTelemetryDescriptor()) {
            dispatchAdd(&writeReadEepromEntry, MSP_DISPATCH_DELAY_US);
        } else
#endif
        {
            writeReadEeprom(NULL);
        }

        break;

#ifdef USE_BLACKBOX
    case MSP_SET_BLACKBOX_CONFIG:
        // Don't allow config to be updated while Blackbox is logging
        if (blackboxMayEditConfig()) {
            blackboxConfigMutable()->device = sbufReadU8(src);
            blackboxConfigMutable()->mode = sbufReadU8(src);
            blackboxConfigMutable()->denom = sbufReadU16(src);
            blackboxConfigMutable()->fields = sbufReadU32(src);
            if (sbufBytesRemaining(src) >= 3) {
                blackboxConfigMutable()->initialEraseFreeSpaceKiB = sbufReadU16(src);
                blackboxConfigMutable()->rollingErase = sbufReadU8(src);
            }
            if (sbufBytesRemaining(src) >= 1) {
                blackboxConfigMutable()->gracePeriod = sbufReadU8(src);
            }
        }
        break;
#endif

#ifdef USE_DSHOT
    case MSP2_SEND_DSHOT_COMMAND:
        {
            const bool armed = ARMING_FLAG(ARMED);

            if (!armed) {
                const uint8_t commandType = sbufReadU8(src);
                const uint8_t motorIndex = sbufReadU8(src);
                const uint8_t commandCount = sbufReadU8(src);

                if (DSHOT_CMD_TYPE_BLOCKING == commandType) {
                    motorDisable();
                }

                for (uint8_t i = 0; i < commandCount; i++) {
                    const uint8_t commandIndex = sbufReadU8(src);
                    dshotCommandWrite(motorIndex, getMotorCount(), commandIndex, commandType);
                }

                if (DSHOT_CMD_TYPE_BLOCKING == commandType) {
                    motorEnable();
                }
            }
        }
        break;
#endif

    case MSP_SET_ARMING_DISABLED:
        {
            const uint8_t command = sbufReadU8(src);
            if (command) {
                mspArmingDisableByDescriptor(srcDesc);
                setArmingDisabled(ARMING_DISABLED_MSP);
                if (ARMING_FLAG(ARMED)) {
                    disarm(DISARM_REASON_ARMING_DISABLED);
                }
            } else {
                mspArmingEnableByDescriptor(srcDesc);
                if (mspIsMspArmingEnabled()) {
                    unsetArmingDisabled(ARMING_DISABLED_MSP);
                }
            }
        }
        break;

#ifdef USE_FLASHFS
    case MSP_DATAFLASH_ERASE:
        blackboxErase();
        break;
#endif

#ifdef USE_GPS
    case MSP_SET_RAW_GPS:
        gpsSetFixState(sbufReadU8(src));
        gpsSol.numSat = sbufReadU8(src);
        gpsSol.llh.lat = sbufReadU32(src);
        gpsSol.llh.lon = sbufReadU32(src);
        gpsSol.llh.altCm = sbufReadU16(src) * 100; // alt changed from 1m to 0.01m per lsb since MSP API 1.39 by RTH. Received MSP altitudes in 1m per lsb have to upscaled.
        gpsSol.groundSpeed = sbufReadU16(src);
        gpsMspDataReceived();                // mark data fresh so gpsUpdate() doesn't consider it stale
        GPS_update |= GPS_MSP_UPDATE;        // MSP data signalisation to GPS functions
        break;
#endif // USE_GPS
    case MSP_SET_FEATURE_CONFIG:
        featureConfigReplace(sbufReadU32(src));
        break;

    case MSP2_WING_SELECT_TV_PROFILE:
        value = sbufReadU8(src);
        if (value >= PID_PROFILE_COUNT) {
            value = 0;
        }
        changeTvProfile(value);
        break;

    case MSP2_WING_COPY_TV_PID_PROFILE: {
        if (dataSize < 2) {
            return MSP_RESULT_ERROR;
        }
        const uint8_t dstTvProfileIndex = sbufReadU8(src);
        const uint8_t srcTvProfileIndex = sbufReadU8(src);
        if (dstTvProfileIndex < PID_PROFILE_COUNT && srcTvProfileIndex < PID_PROFILE_COUNT &&
            dstTvProfileIndex != srcTvProfileIndex) {
            memcpy(tvPidProfilesMutable(dstTvProfileIndex), tvPidProfiles(srcTvProfileIndex), sizeof(tvPidProfile_t));
            if (!ARMING_FLAG(ARMED) && dstTvProfileIndex == getCurrentTvProfileIndex()) {
                changeTvProfile(dstTvProfileIndex);
            }
        }
        break;
    }

    case MSP_SET_MIXER_CONFIG:
        mixerConfigMutable()->model_type = sbufReadU8(src);
        busServoConfigMutable()->cloneFromPwm = sbufReadU8(src);
        // API 22.5: optional SBUS and F.Bus output channel counts. Takes
        // effect on the next frame.
        if (sbufBytesRemaining(src) >= 2) {
            const uint8_t sbusSetting = busOutChannelSetting(sbufReadU8(src));
            const uint8_t fbusSetting = busOutChannelSetting(sbufReadU8(src));
            if (sbusSetting > BUS_OUT_CHANNELS_16 || fbusSetting >= BUS_OUT_CHANNELS_COUNT) {
                return MSP_RESULT_ERROR;
            }
#ifdef USE_SBUS_OUTPUT
            sbusOutConfigMutable()->channels = sbusSetting;
#endif
#ifdef USE_FBUS_MASTER
            fbusMasterConfigMutable()->channels = fbusSetting;
#endif
        }
        break;

    case MSP_SET_MIXER_OVERRIDE:
        i = sbufReadU8(src);
        if (i >= MIXER_INPUT_COUNT) {
            return MSP_RESULT_ERROR;
        }
        {
            // Optional U16 timeout ms (API 22.14): the override lapses unless re-sent.
            const int16_t value = sbufReadU16(src);
            if (sbufBytesRemaining(src) >= 2) {
                mixerSetOverrideTimed(i, value, sbufReadU16(src));
            } else {
                mixerSetOverride(i, value);
            }
        }
        break;

    case MSP2_WING_SET_MODE_OVERRIDE: {
        // U16 timeout ms (clamped to 500-30000), U8 count, count x U8 permanent box id.
        // Replaces the whole override; timeout 0 clears it. Count 0 holds the setup state with
        // no mode forced (a setup tool is in charge, arming is blocked). The client re-sends
        // it before the timeout to keep it. Refused while armed or for a mode that cannot be
        // forced.
        if (sbufBytesRemaining(src) < 3) {
            return MSP_RESULT_ERROR;
        }
        const uint16_t timeoutMs = sbufReadU16(src);
        const uint8_t count = sbufReadU8(src);
        if (count > MODE_OVERRIDE_MAX_COUNT || sbufBytesRemaining(src) != count) {
            return MSP_RESULT_ERROR;
        }
        if (timeoutMs == 0) {
            clearModeOverride();
            break;
        }
        boxBitmask_t mask;
        memset(&mask, 0, sizeof(mask));
        for (int n = 0; n < count; n++) {
            const box_t *box = findBoxByPermanentId(sbufReadU8(src));
            if (!box) {
                return MSP_RESULT_ERROR;
            }
            bitArraySet(&mask, box->boxId);
        }
        if (!setModeOverride(&mask, timeoutMs)) {
            return MSP_RESULT_ERROR;
        }
        break;
    }

    case MSP_SET_RXFAIL_CONFIG:
        i = sbufReadU8(src);
        if (i < MAX_SUPPORTED_RC_CHANNEL_COUNT) {
            rxFailsafeChannelConfigsMutable(i)->mode = sbufReadU8(src);
            rxFailsafeChannelConfigsMutable(i)->step = CHANNEL_VALUE_TO_RXFAIL_STEP(sbufReadU16(src));
        } else {
            return MSP_RESULT_ERROR;
        }
        break;

    case MSP_SET_SERIAL_CONFIG:
        {
            const uint8_t portConfigSize = 1 + 4 + 4;
            if (dataSize % portConfigSize != 0) {
                return MSP_RESULT_ERROR;
            }

            uint8_t portCount = dataSize / portConfigSize;

            while (portCount-- > 0) {
                uint8_t identifier = sbufReadU8(src);
                serialPortConfig_t *portConfig = serialFindPortConfigurationMutable(identifier);
                if (!portConfig) {
                    return MSP_RESULT_ERROR;
                }

                portConfig->identifier = identifier;
                portConfig->functionMask = sbufReadU32(src);
                portConfig->msp_baudrateIndex = sbufReadU8(src);
                portConfig->gps_baudrateIndex = sbufReadU8(src);
                portConfig->telemetry_baudrateIndex = sbufReadU8(src);
                portConfig->blackbox_baudrateIndex = sbufReadU8(src);
            }
        }
        break;

#ifdef USE_LED_STRIP_STATUS_MODE
    case MSP_SET_LED_STRIP_MODECOLOR:
        {
            ledModeIndex_e modeIdx = sbufReadU8(src);
            int funIdx = sbufReadU8(src);
            int color = sbufReadU8(src);

            if (!setModeColor(modeIdx, funIdx, color)) {
                return MSP_RESULT_ERROR;
            }
        }
        break;
#endif

#ifdef USE_RTC_TIME
    case MSP_SET_RTC:
        {
            // Use seconds and milliseconds to make senders
            // easier to implement. Generating a 64 bit value
            // might not be trivial in some platforms.
            int32_t secs = (int32_t)sbufReadU32(src);
            uint16_t millis = sbufReadU16(src);
            rtcTime_t t = rtcTimeMake(secs, millis);
            rtcSet(&t);
        }

        break;
#endif

    case MSP_SET_TX_INFO:
        setRssiMsp(sbufReadU8(src));

        break;

#ifdef USE_FBUS_MASTER
    case MSP_SET_XACT_SCAN:
        // Start a new sensor discovery phase on the FBUS master link. Not while armed: a
        // rescan drops the discovered telemetry sensors until they are found again.
        if (!fbusMasterIsEnabled() || ARMING_FLAG(ARMED)) {
            return MSP_RESULT_ERROR;
        }
        fbusXactStartSensorDiscovery();

        break;

    case MSP_SET_XACT_PARAMS:
        // SET all servo parameters for one discovered servo - collect from GUI, compare with
        // cache, write only differences. Field set mirrors FrSky's own "XAct" ETHOS Device
        // Config Lua script; workingMode/maxAngle are ignored unless the servo has already
        // reported (via a prior GET) that it supports them.
        // Request format: targetPhyID, then physicalId, appIdOffset, dataRate, range, direction,
        // pulseType, channel, center(signed), holdingStrength, operationSmoothing, deadband,
        // workingMode, maxAngle. targetPhyID selects which discovered servo to write to; the
        // "physicalId" value right after it is the new value to write into that servo's own
        // Physical ID field, and may differ from targetPhyID if the user is deliberately
        // re-addressing the servo. Refused while armed, so a servo's direction, center or
        // channel can't change in flight.
        if (fbusMasterIsEnabled() && !ARMING_FLAG(ARMED) && sbufBytesRemaining(src) >= 16) {
            const uint8_t phyID = sbufReadU8(src);
            xactServoParams_t params;

            if (fbusXactGetServoParams(phyID, &params)) {
                xactServoParams_t newParams;
                newParams.physicalId = sbufReadU8(src);
                newParams.appIdOffset = sbufReadU8(src);
                newParams.dataRate = sbufReadU16(src);
                newParams.range = sbufReadU8(src);
                newParams.direction = sbufReadU8(src);
                newParams.pulseType = sbufReadU8(src);
                newParams.channel = sbufReadU8(src);
                newParams.center = (int8_t)sbufReadU8(src);
                newParams.holdingStrength = sbufReadU8(src);
                newParams.operationSmoothing = sbufReadU8(src);
                newParams.deadband = sbufReadU8(src);
                newParams.workingMode = sbufReadU8(src);
                newParams.maxAngle = sbufReadU16(src);

                // Compare with cache and write only differences
                if (!fbusXactCompareAndWriteParams(phyID, &newParams)) {
                    return MSP_RESULT_ERROR;
                }
            } else {
                return MSP_RESULT_ERROR;
            }
        } else {
            return MSP_RESULT_ERROR;
        }

        break;
#endif

#if defined(USE_RX_BIND)
    case MSP2_BETAFLIGHT_BIND:
        if (!startRxBind()) {
            return MSP_RESULT_ERROR;
        }

        break;
#endif

    default:
        // the config catalogue (msp_catalogue.c), which answers
        // MSP_RESULT_ERROR for a message nobody handles
        return mspCatalogueProcessInCommand(srcDesc, cmdMSP, src);
    }
    return MSP_RESULT_ACK;
}

#ifdef USE_FBUS_MUX_FPGA
// Reads U8 channel count, then count x U8 mode (fbusMuxMode_e), as the rest of the
// payload. The count must match the build's, and DShot is refused on channels
// without a DShot bridge.
static bool mspReadFbusMuxModes(sbuf_t *src, uint8_t *modes)
{
    if (sbufBytesRemaining(src) < 1) {
        return false;
    }
    const uint8_t count = sbufReadU8(src);
    if (count != FBUS_MUX_FPGA_CHANNELS || sbufBytesRemaining(src) != count) {
        return false;
    }
    for (int ch = 0; ch < FBUS_MUX_FPGA_CHANNELS; ch++) {
        modes[ch] = sbufReadU8(src);
        if (modes[ch] >= FBUS_MUX_MODE_COUNT ||
            (modes[ch] == FBUS_MUX_MODE_DSHOT && !(fbusMuxFpgaDshotChannelMask & (1 << ch)))) {
            return false;
        }
    }
    return true;
}
#endif

static mspResult_e mspCommonProcessInCommand(mspDescriptor_t srcDesc, int16_t cmdMSP, sbuf_t *src, mspPostProcessFnPtr *mspPostProcessFn)
{
    UNUSED(mspPostProcessFn);
    const unsigned int dataSize = sbufBytesRemaining(src);
    UNUSED(dataSize); // maybe unused due to compiler options

    switch (cmdMSP) {

#ifdef USE_TUNE_ADVISOR
    case MSP2_WING_TUNE_ADVISOR_CLEAR:
        tuneAdvisorReset();
        break;
#endif

#if defined(USE_FBUS_MASTER) || defined(USE_SPORT_MASTER)
    case MSP2_WING_CLEAR_FBUS_SENSORS:
        fbusSensorClearObserved();
        break;

#endif

#ifdef USE_SRXL2_ESC
    case MSP2_WING_SET_SRXL2_ESC_CONFIG: {
        // Same fields as MSP2_WING_SRXL2_ESC_CONFIG without the version byte.
        // Out-of-range values are repaired by validateAndFixSrxl2escConfig()
        // and the driver (an invalid motor 2 channel reads as off).
        // Exact length, so a GET-format payload (leading version byte) is refused.
        if (dataSize != 4) {
            return MSP_RESULT_ERROR;
        }
        const uint16_t throttleRateHz = sbufReadU16(src);
        srxl2escConfigMutable()->throttle_rate_hz = throttleRateHz;
        srxl2escConfigMutable()->telem_interval_frames = sbufReadU8(src);
        srxl2escConfigMutable()->motor2_channel = sbufReadU8(src);
#if defined(USE_MOTOR)
        // The driver follows the motor PWM rate, not throttle_rate_hz, and adopts a
        // change at runtime (capped at 250 Hz while the ESC is active), so the
        // requested rate goes there too. Only with an SRXL2 ESC: on the other
        // protocols the motor PWM rate is their own.
        if (isMotorProtocolSrxl2() && throttleRateHz != 0) {
            motorConfigMutable()->dev.motorPwmRate = constrain(throttleRateHz, 50, 2000);
        }
#endif
        break;
    }
#endif

#ifdef USE_FBUS_MUX_FPGA
    case MSP2_WING_SET_FBUS_MUX_CONFIG: {
        // U8 version, U8 channel count, then count x U8 mode (fbusMuxMode_e). The FPGA
        // is configured at boot, so the modes apply after save and reboot.
        if (sbufBytesRemaining(src) < 1) {
            return MSP_RESULT_ERROR;
        }
        sbufReadU8(src); // payload version, unused for now
        uint8_t modes[FBUS_MUX_FPGA_CHANNELS];
        if (!mspReadFbusMuxModes(src, modes)) {
            return MSP_RESULT_ERROR;
        }
        memcpy(fbusMuxFpgaConfigMutable()->mode, modes, sizeof(modes));
        break;
    }

    case MSP2_WING_SET_FBUS_MUX_OVERRIDE: {
        // U16 timeout ms, U8 channel count, then count x U8 mode. Applies the modes at
        // once without saving them, e.g. FBUS on one output at a time to give XACT servos
        // their Physical IDs. The client re-sends it before the timeout (clamped to
        // 500-30000 ms) to keep it; timeout 0 ends it, with or without modes. Blocks
        // arming while held. Refused while armed, without a configured FPGA or if the
        // FPGA does not confirm the modes (also the saved ones when ending it).
        if (sbufBytesRemaining(src) < 2) {
            return MSP_RESULT_ERROR;
        }
        const uint16_t timeoutMs = sbufReadU16(src);
        if (timeoutMs == 0) {
            if (!fbusMuxFpgaClearOverride()) {
                return MSP_RESULT_ERROR;
            }
            break;
        }
        uint8_t modes[FBUS_MUX_FPGA_CHANNELS];
        if (!mspReadFbusMuxModes(src, modes) || !fbusMuxFpgaSetOverride(modes, timeoutMs)) {
            return MSP_RESULT_ERROR;
        }
        break;
    }
#endif

    case MSP_SET_BATTERY_PROFILE:
        {
            uint8_t index = sbufReadU8(src);
            if (index < BATTERY_PROFILE_COUNT) {
                changeBatteryProfile(index);
            } else {
                return MSP_RESULT_ERROR;
            }
        }
        break;

    case MSP_SET_EXPERIMENTAL:
        /*
         * Receive your experimental parameters from LUA. Like:
         *
         * if (sbufBytesRemaining(src) >= 2) {
         *     currentPidProfile->yourFancyParameterA = sbufReadU8(src);
         *     currentPidProfile->yourFancyParameterB = sbufReadU8(src);
         * }
         */
        break;

    default:
        return mspProcessInCommand(srcDesc, cmdMSP, src);
    }
    return MSP_RESULT_ACK;
}

/*
 * Returns MSP_RESULT_ACK, MSP_RESULT_ERROR or MSP_RESULT_NO_REPLY
 */
mspResult_e mspFcProcessCommand(mspDescriptor_t srcDesc, mspPacket_t *cmd, mspPacket_t *reply, mspPostProcessFnPtr *mspPostProcessFn)
{
    int ret = MSP_RESULT_ACK;
    mspResult_e paramResult = MSP_RESULT_ERROR;
    sbuf_t *dst = &reply->buf;
    sbuf_t *src = &cmd->buf;
    const int16_t cmdMSP = cmd->cmd;
    // initialize reply by default
    reply->cmd = cmd->cmd;

    if (mspCommonProcessOutCommand(cmdMSP, dst, mspPostProcessFn)) {
        ret = MSP_RESULT_ACK;
    } else if (mspProcessOutCommand(cmdMSP, dst)) {
        ret = MSP_RESULT_ACK;
    } else if ((ret = mspFcProcessOutCommandWithArg(srcDesc, cmdMSP, src, dst, mspPostProcessFn)) != MSP_RESULT_CMD_UNKNOWN) {
        /* ret */;
    } else if (mspParamCommand(cmdMSP, src, dst, &paramResult)) {
        ret = paramResult;
    } else if (mspRuntimeCommand(cmdMSP, src, dst, &paramResult)) {
        ret = paramResult;
    } else if (cmdMSP == MSP_SET_PASSTHROUGH) {
        mspFcSetPassthroughCommand(dst, src, mspPostProcessFn);
        ret = MSP_RESULT_ACK;
#ifdef USE_FLASHFS
    } else if (cmdMSP == MSP_DATAFLASH_READ) {
        mspFcDataFlashReadCommand(dst, src);
        ret = MSP_RESULT_ACK;
#endif
    } else {
        ret = mspCommonProcessInCommand(srcDesc, cmdMSP, src, mspPostProcessFn);
    }
    reply->result = ret;
    return ret;
}

void mspFcProcessReply(mspPacket_t *reply)
{
    //sbuf_t *src = &reply->buf;

    switch (reply->cmd) {
        default:
            break;
    }
}

void mspInit(void)
{
    initActiveBoxIds();
}
