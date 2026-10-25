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
 * The loader presents itself like the STM32 ROM DFU (VID/PID 0483:DF11, same serial number),
 * so the DFU driver and any WebUSB permission of the ROM DFU apply to it as well.
 */

#include "stm32h7xx_hal.h"

#include "usbd_core.h"
#include "usbd_ctlreq.h"

#include "usbd_desc.h"

#define USBD_VID                0x0483
#define USBD_PID                0xDF11
#define USBD_LANGID             0x0409
#define USBD_MANUFACTURER       "Wingflight"
#define USBD_PRODUCT            "Wingflight Loader"
#define USBD_CONFIGURATION      "DFU Config"
#define USBD_INTERFACE          "DFU Interface"

#define SERIAL_STRING_SIZE      0x1A

static uint8_t *deviceDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);
static uint8_t *langIdDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);
static uint8_t *manufacturerDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);
static uint8_t *productDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);
static uint8_t *serialDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);
static uint8_t *configurationDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);
static uint8_t *interfaceDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);

USBD_DescriptorsTypeDef dfuDescriptors = {
    deviceDescriptor,
    langIdDescriptor,
    manufacturerDescriptor,
    productDescriptor,
    serialDescriptor,
    configurationDescriptor,
    interfaceDescriptor,
};

static __ALIGN_BEGIN uint8_t deviceDesc[USB_LEN_DEV_DESC] __ALIGN_END = {
    0x12,                       // bLength
    USB_DESC_TYPE_DEVICE,       // bDescriptorType
    0x00, 0x02,                 // bcdUSB 2.00
    0x00,                       // bDeviceClass
    0x00,                       // bDeviceSubClass
    0x00,                       // bDeviceProtocol
    USB_MAX_EP0_SIZE,           // bMaxPacketSize
    LOBYTE(USBD_VID), HIBYTE(USBD_VID),
    LOBYTE(USBD_PID), HIBYTE(USBD_PID),
    0x00, 0x02,                 // bcdDevice 2.00
    USBD_IDX_MFC_STR,
    USBD_IDX_PRODUCT_STR,
    USBD_IDX_SERIAL_STR,
    USBD_MAX_NUM_CONFIGURATION,
};

static __ALIGN_BEGIN uint8_t langIdDesc[USB_LEN_LANGID_STR_DESC] __ALIGN_END = {
    USB_LEN_LANGID_STR_DESC,
    USB_DESC_TYPE_STRING,
    LOBYTE(USBD_LANGID), HIBYTE(USBD_LANGID),
};

static __ALIGN_BEGIN uint8_t serialString[SERIAL_STRING_SIZE] __ALIGN_END = {
    SERIAL_STRING_SIZE,
    USB_DESC_TYPE_STRING,
};

static __ALIGN_BEGIN uint8_t stringDesc[USBD_MAX_STR_DESC_SIZ] __ALIGN_END;

static uint8_t *deviceDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
    (void)speed;
    *length = sizeof(deviceDesc);
    return deviceDesc;
}

static uint8_t *langIdDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
    (void)speed;
    *length = sizeof(langIdDesc);
    return langIdDesc;
}

static uint8_t *stringDescriptor(const char *string, uint16_t *length)
{
    USBD_GetString((uint8_t *)string, stringDesc, length);
    return stringDesc;
}

static uint8_t *manufacturerDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
    (void)speed;
    return stringDescriptor(USBD_MANUFACTURER, length);
}

static uint8_t *productDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
    (void)speed;
    return stringDescriptor(USBD_PRODUCT, length);
}

static uint8_t *configurationDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
    (void)speed;
    return stringDescriptor(USBD_CONFIGURATION, length);
}

static uint8_t *interfaceDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
    (void)speed;
    return stringDescriptor(USBD_INTERFACE, length);
}

static void intToUnicode(uint32_t value, uint8_t *buffer, uint8_t digits)
{
    for (uint8_t i = 0; i < digits; i++) {
        const uint8_t digit = value >> 28;
        buffer[2 * i] = digit < 10 ? '0' + digit : 'A' + digit - 10;
        buffer[2 * i + 1] = 0;
        value <<= 4;
    }
}

// Same serial number as the ROM DFU and the firmware's USB VCP, see src/main/vcp_hal/usbd_desc.c
static uint8_t *serialDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
    (void)speed;

    const uint32_t serial0 = *(uint32_t *)UID_BASE + *(uint32_t *)(UID_BASE + 8);
    const uint32_t serial1 = *(uint32_t *)(UID_BASE + 4);

    intToUnicode(serial0, &serialString[2], 8);
    intToUnicode(serial1, &serialString[18], 4);

    *length = SERIAL_STRING_SIZE;
    return serialString;
}
