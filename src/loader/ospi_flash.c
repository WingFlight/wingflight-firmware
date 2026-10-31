/*
 * This file is part of Wingflight.
 *
 * Wingflight is free software. You can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Wingflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software. If not, see <https://www.gnu.org/licenses/>.
 */

/*
 * Quad SPI NOR flash on OCTOSPI1, in indirect mode for the DFU transfers and in memory mapped
 * quad mode for the firmware to execute from. The FrSky Aegis Lite has a GigaDevice GD25Q128
 * (16 MB, of which the loader uses the first 2 MB). Winbond's W25Q has the same commands.
 */

#include <stdbool.h>
#include <stdint.h>

#include "stm32h7xx_hal.h"

#include "ospi_flash.h"

#define CMD_WRITE_ENABLE        0x06
#define CMD_READ_STATUS1        0x05
#define CMD_READ_STATUS2        0x35
#define CMD_WRITE_STATUS        0x01
#define CMD_WRITE_STATUS2       0x31
#define CMD_READ_JEDEC_ID       0x9F
#define CMD_PAGE_PROGRAM        0x02
#define CMD_FAST_READ           0x0B
#define CMD_FAST_READ_QUAD_IO   0xEB
#define CMD_SECTOR_ERASE_4K     0x20
#define CMD_BLOCK_ERASE_64K     0xD8
#define CMD_ENABLE_RESET        0x66
#define CMD_RESET               0x99
#define CMD_MODE_BIT_RESET      0xFF

#define STATUS1_BUSY            0x01
#define STATUS2_QE              0x02

#define PAGE_SIZE               256

// The kernel clock is HCLK3: 64 MHz HSI in the loader, 260 MHz once the firmware has set up its clocks.
// Divided by 3 that is 21 / 87 MHz, below the 104 MHz these flashes read at in quad I/O mode.
#define CLOCK_PRESCALER         3

static OSPI_HandleTypeDef hospi;

#define JEDEC_GIGADEVICE        0xC8
#define JEDEC_WINBOND           0xEF

static void pinInit(GPIO_TypeDef *gpio, uint32_t pin, uint32_t af)
{
    const GPIO_InitTypeDef init = {
        .Pin = pin,
        .Mode = GPIO_MODE_AF_PP,
        .Pull = GPIO_NOPULL,
        .Speed = GPIO_SPEED_FREQ_VERY_HIGH,
        .Alternate = af,
    };
    HAL_GPIO_Init(gpio, (GPIO_InitTypeDef *)&init);
}

static OSPI_RegularCmdTypeDef command(uint8_t instruction, uint32_t lines)
{
    const OSPI_RegularCmdTypeDef cmd = {
        .OperationType = HAL_OSPI_OPTYPE_COMMON_CFG,
        .FlashId = HAL_OSPI_FLASH_ID_1,
        .Instruction = instruction,
        .InstructionMode = lines,
        .InstructionSize = HAL_OSPI_INSTRUCTION_8_BITS,
        .InstructionDtrMode = HAL_OSPI_INSTRUCTION_DTR_DISABLE,
        .AddressMode = HAL_OSPI_ADDRESS_NONE,
        .AddressSize = HAL_OSPI_ADDRESS_24_BITS,
        .AddressDtrMode = HAL_OSPI_ADDRESS_DTR_DISABLE,
        .AlternateBytesMode = HAL_OSPI_ALTERNATE_BYTES_NONE,
        .AlternateBytesDtrMode = HAL_OSPI_ALTERNATE_BYTES_DTR_DISABLE,
        .DataMode = HAL_OSPI_DATA_NONE,
        .DataDtrMode = HAL_OSPI_DATA_DTR_DISABLE,
        .DummyCycles = 0,
        .DQSMode = HAL_OSPI_DQS_DISABLE,
        .SIOOMode = HAL_OSPI_SIOO_INST_EVERY_CMD,
    };
    return cmd;
}

static bool sendCommand(uint8_t instruction, uint32_t lines)
{
    OSPI_RegularCmdTypeDef cmd = command(instruction, lines);
    return HAL_OSPI_Command(&hospi, &cmd, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) == HAL_OK;
}

static bool readRegister(uint8_t instruction, uint8_t *data, uint32_t length)
{
    OSPI_RegularCmdTypeDef cmd = command(instruction, HAL_OSPI_INSTRUCTION_1_LINE);
    cmd.DataMode = HAL_OSPI_DATA_1_LINE;
    cmd.NbData = length;
    return HAL_OSPI_Command(&hospi, &cmd, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) == HAL_OK &&
        HAL_OSPI_Receive(&hospi, data, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) == HAL_OK;
}

static bool waitReady(uint32_t timeoutMs)
{
    const uint32_t start = HAL_GetTick();
    do {
        uint8_t status;
        if (!readRegister(CMD_READ_STATUS1, &status, 1)) {
            return false;
        }
        if (!(status & STATUS1_BUSY)) {
            return true;
        }
    } while (HAL_GetTick() - start < timeoutMs);
    return false;
}

// Leaves the flash in its power-on state, whatever the previous boot set up (continuous read, QPI).
static void flashReset(void)
{
    static const uint32_t lineModes[] = { HAL_OSPI_INSTRUCTION_4_LINES, HAL_OSPI_INSTRUCTION_1_LINE };

    for (unsigned i = 0; i < 2; i++) {
        sendCommand(CMD_MODE_BIT_RESET, lineModes[i]);
        sendCommand(CMD_ENABLE_RESET, lineModes[i]);
        sendCommand(CMD_RESET, lineModes[i]);
    }
    HAL_Delay(1); // tRST 30 us
}

static bool writeStatus(uint8_t instruction, uint8_t *data, uint32_t length)
{
    OSPI_RegularCmdTypeDef cmd = command(instruction, HAL_OSPI_INSTRUCTION_1_LINE);
    cmd.DataMode = HAL_OSPI_DATA_1_LINE;
    cmd.NbData = length;
    return sendCommand(CMD_WRITE_ENABLE, HAL_OSPI_INSTRUCTION_1_LINE) &&
        HAL_OSPI_Command(&hospi, &cmd, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) == HAL_OK &&
        HAL_OSPI_Transmit(&hospi, data, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) == HAL_OK &&
        waitReady(100); // tW max 30 ms
}

static bool quadEnabled(void)
{
    uint8_t status2;
    return readRegister(CMD_READ_STATUS2, &status2, 1) && (status2 & STATUS2_QE);
}

// The quad enable bit is non-volatile, so this writes the status register once per flash chip
static bool enableQuad(void)
{
    if (quadEnabled()) {
        return true;
    }

    uint8_t status[2];
    if (!readRegister(CMD_READ_STATUS1, &status[0], 1) || !readRegister(CMD_READ_STATUS2, &status[1], 1)) {
        return false;
    }
    status[1] |= STATUS2_QE;

    // Write Status Register-2 (W25Q, GD25Q128E), else both registers at once (GD25Q128C)
    writeStatus(CMD_WRITE_STATUS2, &status[1], 1);
    if (quadEnabled()) {
        return true;
    }
    writeStatus(CMD_WRITE_STATUS, status, 2);
    return quadEnabled();
}

bool ospiFlashInit(void)
{
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_OCTOSPIM_CLK_ENABLE();
    __HAL_RCC_OSPI1_CLK_ENABLE();
    __HAL_RCC_OSPI1_FORCE_RESET();
    __HAL_RCC_OSPI1_RELEASE_RESET();

    // FrSky Aegis Lite, as in the stock firmware
    pinInit(GPIOB, GPIO_PIN_2, GPIO_AF9_OCTOSPIM_P1);   // CLK
    pinInit(GPIOE, GPIO_PIN_11, GPIO_AF11_OCTOSPIM_P1); // NCS
    pinInit(GPIOD, GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13, GPIO_AF9_OCTOSPIM_P1); // IO0 IO1 IO3
    pinInit(GPIOB, GPIO_PIN_13, GPIO_AF4_OCTOSPIM_P1);  // IO2

    hospi.Instance = OCTOSPI1;
    hospi.Init.FifoThreshold = 4;
    hospi.Init.DualQuad = HAL_OSPI_DUALQUAD_DISABLE;
    hospi.Init.MemoryType = HAL_OSPI_MEMTYPE_MICRON;
    hospi.Init.DeviceSize = 21; // 2 MB
    hospi.Init.ChipSelectHighTime = 2;
    hospi.Init.FreeRunningClock = HAL_OSPI_FREERUNCLK_DISABLE;
    hospi.Init.ClockMode = HAL_OSPI_CLOCK_MODE_0;
    hospi.Init.WrapSize = HAL_OSPI_WRAP_NOT_SUPPORTED;
    hospi.Init.ClockPrescaler = CLOCK_PRESCALER;
    hospi.Init.SampleShifting = HAL_OSPI_SAMPLE_SHIFTING_HALFCYCLE;
    hospi.Init.DelayHoldQuarterCycle = HAL_OSPI_DHQC_DISABLE;
    hospi.Init.ChipSelectBoundary = 0;
    hospi.Init.ClkChipSelectHighTime = 0;
    hospi.Init.DelayBlockBypass = HAL_OSPI_DELAY_BLOCK_BYPASSED;
    hospi.Init.MaxTran = 0;
    hospi.Init.Refresh = 0;
    if (HAL_OSPI_Init(&hospi) != HAL_OK) {
        return false;
    }

    OSPIM_CfgTypeDef ospim = {
        .ClkPort = 1,
        .DQSPort = 0,
        .NCSPort = 1,
        .IOLowPort = HAL_OSPIM_IOPORT_1_LOW,
        .IOHighPort = 0, // unused
        .Req2AckTime = 1,
    };
    if (HAL_OSPIM_Config(&hospi, &ospim, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
        return false;
    }

    flashReset();

    // Manufacturer, memory type, capacity as a power of two, at least the 2 MB used
    uint8_t id[3];
    if (!readRegister(CMD_READ_JEDEC_ID, id, sizeof(id)) ||
        (id[0] != JEDEC_GIGADEVICE && id[0] != JEDEC_WINBOND) ||
        id[2] < 21 || id[2] > 24) {
        return false;
    }

    return waitReady(100) && enableQuad();
}

bool ospiFlashRead(uint32_t offset, uint8_t *data, uint32_t length)
{
    OSPI_RegularCmdTypeDef cmd = command(CMD_FAST_READ, HAL_OSPI_INSTRUCTION_1_LINE);
    cmd.Address = offset;
    cmd.AddressMode = HAL_OSPI_ADDRESS_1_LINE;
    cmd.DataMode = HAL_OSPI_DATA_1_LINE;
    cmd.NbData = length;
    cmd.DummyCycles = 8;
    return HAL_OSPI_Command(&hospi, &cmd, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) == HAL_OK &&
        HAL_OSPI_Receive(&hospi, data, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) == HAL_OK;
}

static bool erase(uint8_t instruction, uint32_t offset, uint32_t timeoutMs)
{
    OSPI_RegularCmdTypeDef cmd = command(instruction, HAL_OSPI_INSTRUCTION_1_LINE);
    cmd.Address = offset;
    cmd.AddressMode = HAL_OSPI_ADDRESS_1_LINE;
    return sendCommand(CMD_WRITE_ENABLE, HAL_OSPI_INSTRUCTION_1_LINE) &&
        HAL_OSPI_Command(&hospi, &cmd, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) == HAL_OK &&
        waitReady(timeoutMs);
}

bool ospiFlashEraseBlock(uint32_t offset)
{
    return erase(CMD_BLOCK_ERASE_64K, offset, 3000); // tBE2 max 2 s
}

bool ospiFlashEraseSector(uint32_t offset)
{
    return erase(CMD_SECTOR_ERASE_4K, offset, 500); // tSE max 400 ms
}

bool ospiFlashWrite(uint32_t offset, const uint8_t *data, uint32_t length)
{
    while (length) {
        const uint32_t chunk = MIN(length, PAGE_SIZE - (offset % PAGE_SIZE));

        OSPI_RegularCmdTypeDef cmd = command(CMD_PAGE_PROGRAM, HAL_OSPI_INSTRUCTION_1_LINE);
        cmd.Address = offset;
        cmd.AddressMode = HAL_OSPI_ADDRESS_1_LINE;
        cmd.DataMode = HAL_OSPI_DATA_1_LINE;
        cmd.NbData = chunk;
        if (!sendCommand(CMD_WRITE_ENABLE, HAL_OSPI_INSTRUCTION_1_LINE) ||
            HAL_OSPI_Command(&hospi, &cmd, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK ||
            HAL_OSPI_Transmit(&hospi, (uint8_t *)data, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK ||
            !waitReady(10)) { // tPP max 3 ms
            return false;
        }

        offset += chunk;
        data += chunk;
        length -= chunk;
    }
    return true;
}

bool ospiFlashMemoryMapped(void)
{
    // Fast Read Quad I/O: address and data on four lines, mode bits 0xFF (no continuous read), 4 dummy cycles
    OSPI_RegularCmdTypeDef cmd = command(CMD_FAST_READ_QUAD_IO, HAL_OSPI_INSTRUCTION_1_LINE);
    cmd.OperationType = HAL_OSPI_OPTYPE_READ_CFG;
    cmd.AddressMode = HAL_OSPI_ADDRESS_4_LINES;
    cmd.AlternateBytes = 0xFF;
    cmd.AlternateBytesMode = HAL_OSPI_ALTERNATE_BYTES_4_LINES;
    cmd.AlternateBytesSize = HAL_OSPI_ALTERNATE_BYTES_8_BITS;
    cmd.DataMode = HAL_OSPI_DATA_4_LINES;
    cmd.DummyCycles = 4;
    if (HAL_OSPI_Command(&hospi, &cmd, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
        return false;
    }

    // The memory mapped mode needs a write command too, writes to the mapped flash are not used
    cmd = command(CMD_PAGE_PROGRAM, HAL_OSPI_INSTRUCTION_1_LINE);
    cmd.OperationType = HAL_OSPI_OPTYPE_WRITE_CFG;
    cmd.AddressMode = HAL_OSPI_ADDRESS_1_LINE;
    cmd.DataMode = HAL_OSPI_DATA_1_LINE;
    if (HAL_OSPI_Command(&hospi, &cmd, HAL_OSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) {
        return false;
    }

    OSPI_MemoryMappedTypeDef mapped = {
        .TimeOutActivation = HAL_OSPI_TIMEOUT_COUNTER_DISABLE,
        .TimeOutPeriod = 0,
    };
    return HAL_OSPI_MemoryMapped(&hospi, &mapped) == HAL_OK;
}
