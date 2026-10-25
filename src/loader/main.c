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
 * Wingflight loader for the STM32H730 (FrSky Aegis Lite)
 *
 * The H730 has 128K of internal flash, too little for the firmware, which executes in place from
 * the 2 MB OCTOSPI NOR flash instead. This loader sits in the internal flash. It maps the external
 * flash into memory and starts the firmware from it, or, when there is no complete firmware or
 * one asked for it, it becomes a USB DFU device (DfuSe, like the ROM DFU) for the external flash.
 *
 * External flash layout:
 *   0x90000000-0x901EFFFF  firmware image, written over DFU (31 blocks of 64K)
 *   0x901F0000             image marker, written when a DFU download ends with a manifest
 *
 * The loader runs from the 64 MHz HSI. The firmware sets up its own clocks.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "stm32h7xx_hal.h"

#include "usbd_core.h"
#include "usbd_ctlreq.h"
#include "usbd_dfu.h"
#include "usbd_desc.h"

#include "drivers/persistent.h"
#include "drivers/system.h"

#include "ospi_flash.h"

#define IMAGE_SIZE              (31 * OSPI_FLASH_BLOCK_SIZE)
#define IMAGE_MARKER_OFFSET     IMAGE_SIZE
#define IMAGE_MARKER_MAGIC      0x444C4657  // "WFLD"

// Shared with the firmware, see src/main/drivers/persistent.c
#define PERSISTENT_OBJECT_MAGIC_VALUE (('B' << 24) | ('e' << 16) | ('f' << 8) | ('1' << 0))

#define SYSTEM_MEMORY_BASE      0x1FF00000
#define SYSTEM_MEMORY_SIZE      0x20000
#define SYSTEM_MEMORY_VECTORS   ((const uint32_t *)0x1FF09800)

#define LED_PORT                GPIOE
#define LED_PIN                 GPIO_PIN_10

typedef struct {
    uint32_t magic;
    uint32_t inverted;
} imageMarker_t;

uint32_t SystemCoreClock = 64000000;
uint32_t SystemD2Clock = 64000000;
const uint8_t D1CorePrescTable[16] = { 0, 0, 0, 0, 1, 2, 3, 4, 1, 2, 3, 4, 6, 7, 8, 9 };

static USBD_HandleTypeDef usbDevice;

static bool flashReady;
static bool imageChanged;

static void handleRomDfuExit(void);

void SystemInit(void)
{
    handleRomDfuExit();

    SCB->CPACR |= (3UL << 10 * 2) | (3UL << 11 * 2); // FPU
    SCB->VTOR = FLASH_BANK1_BASE;
}

void SysTick_Handler(void)
{
    HAL_IncTick();
}

static void jumpTo(const uint32_t *vectors)
{
    const uint32_t stack = vectors[0];
    void (*const reset)(void) = (void (*)(void))vectors[1];

    SysTick->CTRL = 0;
    SCB->ICSR = SCB_ICSR_PENDSTCLR_Msk;
    SCB->VTOR = (uint32_t)vectors;
    __set_MSP(stack);
    __DSB();
    __ISB();
    reset();

    while (1);
}

static volatile uint32_t *backupRegisters(void)
{
    HAL_PWR_EnableBkUpAccess();
    __HAL_RCC_RTC_CLK_ENABLE();

    RTC_TypeDef *rtc = RTC;
    return &rtc->BKP0R;
}

// Started by the ROM DFU when it leaves: it jumps here with the vector table still in system memory,
// where a reset never leaves it. It has just written this loader, and the host continues with the
// firmware through the loader's DFU. Ask for that and start over from a reset.
// Called first thing in SystemInit(), which moves the vector table, before RAM is initialised.
static void handleRomDfuExit(void)
{
    if (SCB->VTOR < SYSTEM_MEMORY_BASE || SCB->VTOR >= SYSTEM_MEMORY_BASE + SYSTEM_MEMORY_SIZE) {
        return;
    }

    volatile uint32_t *bkp = backupRegisters();
    if (bkp[PERSISTENT_OBJECT_MAGIC] != PERSISTENT_OBJECT_MAGIC_VALUE) {
        for (unsigned i = 1; i < PERSISTENT_OBJECT_COUNT; i++) {
            bkp[i] = 0;
        }
        bkp[PERSISTENT_OBJECT_MAGIC] = PERSISTENT_OBJECT_MAGIC_VALUE;
    }
    bkp[PERSISTENT_OBJECT_RESET_REASON] = RESET_BOOTLOADER_REQUEST_FLASH;

    NVIC_SystemReset();
}

// A firmware request left in the RTC backup registers before a soft reset, see systemReset()
static uint32_t takeResetRequest(void)
{
    volatile uint32_t *bkp = backupRegisters();

    if (!(RCC->RSR & RCC_RSR_SFTRSTF) || bkp[PERSISTENT_OBJECT_MAGIC] != PERSISTENT_OBJECT_MAGIC_VALUE) {
        return RESET_NONE;
    }

    const uint32_t request = bkp[PERSISTENT_OBJECT_RESET_REASON];
    if (request == RESET_BOOTLOADER_REQUEST_FLASH || request == RESET_BOOTLOADER_REQUEST_ROM) {
        bkp[PERSISTENT_OBJECT_RESET_REASON] = RESET_NONE;
        return request;
    }
    return RESET_NONE;
}

static bool imageValid(void)
{
    imageMarker_t marker;
    uint32_t vectors[2];

    if (!ospiFlashRead(IMAGE_MARKER_OFFSET, (uint8_t *)&marker, sizeof(marker)) ||
        !ospiFlashRead(0, (uint8_t *)vectors, sizeof(vectors))) {
        return false;
    }

    const uint32_t stack = vectors[0];
    const uint32_t reset = vectors[1];
    return marker.magic == IMAGE_MARKER_MAGIC && marker.inverted == (uint32_t)~IMAGE_MARKER_MAGIC &&
        stack >= 0x20000000 && stack <= 0x24050000 &&
        (reset & 1) && reset > OSPI_FLASH_BASE && reset < OSPI_FLASH_BASE + IMAGE_SIZE;
}

// The marker goes first when the image is touched, so an interrupted download is never started
static bool invalidateImage(void)
{
    if (!imageChanged) {
        if (!ospiFlashEraseSector(IMAGE_MARKER_OFFSET)) {
            return false;
        }
        imageChanged = true;
    }
    return true;
}

static uint16_t dfuInit(void)
{
    return flashReady ? USBD_OK : USBD_FAIL;
}

// Called by the class on every USB reset, with its data still allocated. Only DFU_Leave(), at the
// end of a download (manifest), calls it once more after freeing it: that completes the image.
static uint16_t dfuDeInit(void)
{
    if (imageChanged && usbDevice.pClassData == NULL) {
        const imageMarker_t marker = { .magic = IMAGE_MARKER_MAGIC, .inverted = ~IMAGE_MARKER_MAGIC };
        ospiFlashWrite(IMAGE_MARKER_OFFSET, (const uint8_t *)&marker, sizeof(marker));
        imageChanged = false;
    }
    return USBD_OK;
}

static bool inImage(uint32_t address, uint32_t length)
{
    return address >= OSPI_FLASH_BASE && length <= IMAGE_SIZE && address - OSPI_FLASH_BASE <= IMAGE_SIZE - length;
}

static uint16_t dfuErase(uint32_t address)
{
    if (!flashReady || !inImage(address, 1) || !invalidateImage()) {
        return USBD_FAIL;
    }
    const uint32_t offset = (address - OSPI_FLASH_BASE) & ~(OSPI_FLASH_BLOCK_SIZE - 1);
    return ospiFlashEraseBlock(offset) ? USBD_OK : USBD_FAIL;
}

static uint16_t dfuWrite(uint8_t *src, uint8_t *dest, uint32_t length)
{
    const uint32_t address = (uint32_t)dest;
    if (!flashReady || !inImage(address, length) || !invalidateImage()) {
        return USBD_FAIL;
    }
    return ospiFlashWrite(address - OSPI_FLASH_BASE, src, length) ? USBD_OK : USBD_FAIL;
}

static uint8_t *dfuRead(uint8_t *src, uint8_t *dest, uint32_t length)
{
    const uint32_t address = (uint32_t)src;
    if (!flashReady || !inImage(address, length) || !ospiFlashRead(address - OSPI_FLASH_BASE, dest, length)) {
        memset(dest, 0, length);
    }
    return dest;
}

static uint16_t dfuGetStatus(uint32_t address, uint8_t command, uint8_t *buffer)
{
    (void)address;

    // bwPollTimeout: the host waits this long before it asks for the result.
    // The operation itself runs once the status is sent (USBD_DFU_EP0_TxReady).
    const uint32_t timeoutMs = command == DFU_MEDIA_ERASE ? 150 : 2;
    buffer[1] = timeoutMs & 0xFF;
    buffer[2] = (timeoutMs >> 8) & 0xFF;
    buffer[3] = (timeoutMs >> 16) & 0xFF;
    return USBD_OK;
}

static USBD_DFU_MediaTypeDef dfuMedia = {
    (const uint8_t *)"@External Flash /0x90000000/31*064Kg",
    dfuInit,
    dfuDeInit,
    dfuErase,
    dfuWrite,
    dfuRead,
    dfuGetStatus,
};

// The class answers string indices it doesn't know with a NULL buffer of undefined length, which the
// library then sends (e.g. for Windows' 0xEE OS descriptor query). Stall them instead.
static uint8_t *dfuUserString(USBD_HandleTypeDef *pdev, uint8_t index, uint16_t *length)
{
    if (index == USBD_IDX_INTERFACE_STR + 1) {
        return USBD_DFU.GetUsrStrDescriptor(pdev, index, length);
    }
    *length = 0;
    USBD_CtlError(pdev, &pdev->request);
    return NULL;
}

static USBD_ClassTypeDef dfuClass;

static void usbClockInit(void)
{
    __HAL_RCC_HSI48_ENABLE();
    while (!__HAL_RCC_GET_FLAG(RCC_FLAG_HSI48RDY));

    RCC_PeriphCLKInitTypeDef clock = {
        .PeriphClockSelection = RCC_PERIPHCLK_USB,
        .UsbClockSelection = RCC_USBCLKSOURCE_HSI48,
    };
    HAL_RCCEx_PeriphCLKConfig(&clock);

    // Trim the HSI48 to the host's start of frames
    RCC_CRSInitTypeDef crs = {
        .Prescaler = RCC_CRS_SYNC_DIV1,
        .Source = RCC_CRS_SYNC_SOURCE_USB1,
        .Polarity = RCC_CRS_SYNC_POLARITY_RISING,
        .ReloadValue = RCC_CRS_RELOADVALUE_DEFAULT,
        .ErrorLimitValue = RCC_CRS_ERRORLIMIT_DEFAULT,
        .HSI48CalibrationValue = RCC_CRS_HSI48CALIBRATION_DEFAULT,
    };
    __HAL_RCC_CRS_CLK_ENABLE();
    HAL_RCCEx_CRSConfig(&crs);

    HAL_PWREx_EnableUSBVoltageDetector();
}

static void ledInit(void)
{
    __HAL_RCC_GPIOE_CLK_ENABLE();
    const GPIO_InitTypeDef init = {
        .Pin = LED_PIN,
        .Mode = GPIO_MODE_OUTPUT_PP,
        .Pull = GPIO_NOPULL,
        .Speed = GPIO_SPEED_FREQ_LOW,
    };
    HAL_GPIO_Init(LED_PORT, (GPIO_InitTypeDef *)&init);
    HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET);
}

int main(void)
{
    const uint32_t request = takeResetRequest();

    if (request == RESET_BOOTLOADER_REQUEST_ROM) {
        // Straight from reset, the state the ROM boot loader expects
        jumpTo(SYSTEM_MEMORY_VECTORS);
    }

    HAL_Init();
    ledInit();

    flashReady = ospiFlashInit();

    if (flashReady && request != RESET_BOOTLOADER_REQUEST_FLASH && imageValid() && ospiFlashMemoryMapped()) {
        // The firmware takes over with interrupts masked, see xipBootLoaderHandover()
        __disable_irq();
        jumpTo((const uint32_t *)OSPI_FLASH_BASE);
    }

    // DFU, the LED blinks
    usbClockInit();
    USBD_Init(&usbDevice, &dfuDescriptors, DEVICE_FS);
    dfuClass = USBD_DFU;
    dfuClass.GetUsrStrDescriptor = dfuUserString;
    USBD_RegisterClass(&usbDevice, &dfuClass);
    USBD_DFU_RegisterMedia(&usbDevice, &dfuMedia);
    USBD_Start(&usbDevice);

    while (1) {
        HAL_GPIO_TogglePin(LED_PORT, LED_PIN);
        HAL_Delay(flashReady ? 500 : 100);
    }
}
