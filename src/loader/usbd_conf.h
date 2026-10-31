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

#pragma once

#include <string.h>

#include "stm32h7xx_hal.h"

#define USBD_MAX_NUM_INTERFACES         1U
#define USBD_MAX_NUM_CONFIGURATION      1U
#define USBD_MAX_STR_DESC_SIZ           0x100U
#define USBD_SELF_POWERED               1U
#define USBD_DEBUG_LEVEL                0U
#define USBD_SUPPORT_USER_STRING        1

#define USBD_DFU_MAX_ITF_NUM            1U
#define USBD_DFU_XFER_SIZE              1024U
#define USBD_DFU_APP_DEFAULT_ADD        0x90000000U

#define DEVICE_FS                       0

// The DFU class allocates its one handle, no heap needed
void *USBD_static_malloc(uint32_t size);
void USBD_static_free(void *p);

#define USBD_malloc     USBD_static_malloc
#define USBD_free       USBD_static_free
#define USBD_memset     memset
#define USBD_memcpy     memcpy
#define USBD_Delay      HAL_Delay

#define USBD_UsrLog(...) do {} while (0)
#define USBD_ErrLog(...) do {} while (0)
#define USBD_DbgLog(...) do {} while (0)
