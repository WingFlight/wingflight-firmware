/*
 * This file is part of Wingflight
 *
 * Wingflight is free software. You can redistribute this software
 * and/or modify this software under the terms of the GNU General
 * Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later
 * version.
 *
 * Wingflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

/**
 * X-Plane 11/12 Hardware-in-Loop Target Configuration
 *
 * This target enables Wingflight firmware to run in Software-in-the-Loop (SITL)
 * mode connected to X-Plane 11/12 flight simulator via UDP.
 *
 * Unlike the generic SITL target, XPLANE target:
 * - Uses X-Plane protocol (FDM/PWM packets)
 * - Optimized for fixed-wing aircraft (not multirotor)
 * - Includes motor mixer for servo control
 * - Direct integration with X-Plane plugin
 */

#pragma once

#include <stdint.h>
#include <stddef.h>

#include "common/utils.h"

#define TARGET_BOARD_IDENTIFIER "XPLN"  // X-Plane HITL target

#define TARGET_XPLANE  // Used for conditional compilation in platform headers

#define SIMULATOR_MULTITHREAD

/* ============================================================================
 * HAL TYPE DEFINITIONS FOR SITL
 * ============================================================================ */

typedef void NVIC_InitTypeDef;
typedef void DMA_Stream_TypeDef;
typedef int IRQn_Type;

enum FlagStatus { RESET = 0, SET = !RESET };
typedef enum FlagStatus FlagStatus;
typedef FlagStatus FunctionalState;

// Use simulator's attitude directly
// Disable this if testing AHRS algorithm independently
#undef USE_IMU_CALC

// Configuration storage in file instead of EEPROM
#define EEPROM_FILENAME "xplane_config.bin"
#define CONFIG_IN_FILE
#define EEPROM_SIZE     32768

// Unique ID (placeholder values)
#define U_ID_0 0x58504C4E  // "XPLN" in hex
#define U_ID_1 0x48495452  // "HITR" in hex
#define U_ID_2 0x4C000000  // "L..." in hex

#ifndef UID_BASE
static const uint32_t xplane_uid_words[3] = { U_ID_0, U_ID_1, U_ID_2 };
#define UID_BASE ((uintptr_t)xplane_uid_words)
#endif

// Task timing - synchronized with X-Plane flight loop (20 Hz)
#undef TASK_GYROPID_DESIRED_PERIOD
#define TASK_GYROPID_DESIRED_PERIOD     50  // 50ms = 20 Hz

#undef SCHEDULER_DELAY_LIMIT
#define SCHEDULER_DELAY_LIMIT           2

#define USE_FAKE_LED

/* ============================================================================
 * SENSOR CONFIGURATION
 * ============================================================================ */

// X-Plane provides all sensor data via UDP packets
// These drivers inject X-Plane data into Wingflight's sensor interface

#define USE_ACC
#define USE_ACCGYRO_XPLANE
#define USE_FAKE_ACC

#define USE_GYRO
#define USE_GYRO_XPLANE
#define USE_FAKE_GYRO

#define USE_MAG
#define USE_MAG_XPLANE
#define USE_FAKE_MAG

#define USE_BARO
#define USE_BARO_XPLANE
#define USE_FAKE_BARO

#define USE_GPS
#define USE_GPS_XPLANE

/* ============================================================================
 * UART CONFIGURATION
 * ============================================================================ */

// Enable all UART ports for configurator and debugging
#define USE_UART1
#define USE_UART2
#define USE_UART3
#define USE_UART4
#define USE_UART5
#define USE_UART6
#define USE_UART7
#define USE_UART8

// Use TCP for UART1 (Configurator MSP connection)
#define USE_SERIALRX_MSP
#define USE_TELEMETRY_MSP

#define SERIAL_PORT_COUNT 8

/* ============================================================================
 * FEATURE CONFIGURATION
 * ============================================================================ */

#define DEFAULT_RX_FEATURE      FEATURE_RX_MSP
#define DEFAULT_FEATURES        (FEATURE_GPS | FEATURE_TELEMETRY)

#define USE_PARAMETER_GROUPS

/* ============================================================================
 * DISABLED FEATURES (Not needed for SITL/HITL)
 * ============================================================================ */

#undef USE_STACK_CHECK
#undef USE_CLI
#undef USE_DASHBOARD
#undef USE_TELEMETRY_LTM
#undef USE_ADC
#undef USE_VCP
#undef USE_OSD
#undef USE_PPM
#undef USE_PWM
#undef USE_SERIAL_RX
#undef USE_SERIALRX_CRSF
#undef USE_SERIALRX_GHST
#undef USE_SERIALRX_IBUS
#undef USE_SERIALRX_IBUS2
#undef USE_SERIALRX_SBUS
#undef USE_SERIALRX_SPEKTRUM
#undef USE_SERIALRX_SUMD
#undef USE_SERIALRX_SUMH
#undef USE_SERIALRX_XBUS
#undef USE_LED_STRIP
#undef USE_TELEMETRY_FRSKY_HUB
#undef USE_TELEMETRY_HOTT
#undef USE_TELEMETRY_SMARTPORT
#undef USE_TELEMETRY_MAVLINK
#undef USE_RESOURCE_MGMT
#undef USE_CMS
#undef USE_TELEMETRY_CRSF
#undef USE_TELEMETRY_GHST
#undef USE_TELEMETRY_IBUS
#undef USE_TELEMETRY_JETIEXBUS
#undef USE_TELEMETRY_SRXL
#undef USE_SERIALRX_JETIEXBUS
#undef USE_SBUS_OUTPUT
#undef USE_FBUS_MASTER
#undef USE_SPORT_MASTER
#undef USE_VTX_COMMON
#undef USE_VTX_CONTROL
#undef USE_VTX_SMARTAUDIO
#undef USE_VTX_TRAMP
#undef USE_CAMERA_CONTROL
#undef USE_GPS_RESCUE
#undef USE_SERIAL_4WAY_BLHELI_BOOTLOADER
#undef USE_SERIAL_4WAY_SK_BOOTLOADER

#undef USE_I2C
#undef USE_SPI

/* ============================================================================
 * FLASH MEMORY
 * ============================================================================ */

#define TARGET_FLASH_SIZE 2048

/* ============================================================================
 * TIMER CONFIGURATION (Unused in SITL)
 * ============================================================================ */

#define LED_STRIP_TIMER 1
#define SOFTSERIAL_1_TIMER 2
#define SOFTSERIAL_2_TIMER 3

#define DEFIO_NO_PORTS   // Suppress 'no pins defined' warning

/* ============================================================================
 * INTERNAL DEFINITIONS
 * ============================================================================ */

extern uint32_t SystemCoreClock;

// X-Plane HITL feature flag
#define FEATURE_XPLANE (1 << 31)  // Use bit 31 for X-Plane feature

// Dummy GPIO definitions for SITL compatibility
typedef struct {
  uint32_t IDR;
  uint32_t ODR;
  uint32_t BSRR;
  uint32_t BRR;
} GPIO_TypeDef;

#define GPIOA_BASE ((intptr_t)0x0001)

typedef struct {
    uint32_t CCR1;
    uint32_t CCR2;
    uint32_t CCR3;
    uint32_t CCR4;
} TIM_TypeDef;

typedef struct {
    void* test;
} TIM_OCInitTypeDef;

typedef struct {
    void* test;
} DMA_TypeDef;

typedef struct {
    void* test;
} DMA_Channel_TypeDef;

typedef struct {
    void* test;
} DMA_InitTypeDef;

typedef struct {
    void* test;
} SPI_TypeDef;

typedef struct {
    void* test;
} USART_TypeDef;

#define USART1 ((USART_TypeDef *)0x0001)
#define USART2 ((USART_TypeDef *)0x0002)
#define USART3 ((USART_TypeDef *)0x0003)
#define USART4 ((USART_TypeDef *)0x0004)
#define USART5 ((USART_TypeDef *)0x0005)
#define USART6 ((USART_TypeDef *)0x0006)

typedef struct {
    void* test;
} I2C_TypeDef;

typedef enum {
    FLASH_BUSY = 1,
    FLASH_ERROR_PG,
    FLASH_ERROR_WRP,
    FLASH_COMPLETE,
    FLASH_TIMEOUT
} FLASH_Status;

void FLASH_Unlock(void);
void FLASH_Lock(void);
FLASH_Status FLASH_ErasePage(uintptr_t Page_Address);
FLASH_Status FLASH_ProgramWord(uintptr_t addr, uint32_t Data);

/* Timing and delay functions */
void delayMicroseconds(uint32_t us);
void delayMicroseconds_real(uint32_t us);
