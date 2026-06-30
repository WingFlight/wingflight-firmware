/*
 * Platform abstraction for X-Plane SITL target
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>

/* Dummy HAL types for SITL environment */
typedef void GPIO_TypeDef;
typedef void TIM_TypeDef;
typedef void SPI_TypeDef;
typedef void USART_TypeDef;
typedef void I2C_TypeDef;
typedef void NVIC_InitTypeDef;

typedef int IRQn_Type;

enum FlagStatus { RESET = 0, SET = !RESET };
typedef enum FlagStatus FlagStatus;

/* DMA types */
typedef struct {
    void *test;
} DMA_TypeDef;

typedef struct {
    void *test;
} DMA_Channel_TypeDef;

typedef struct {
    void *test;
} DMA_InitTypeDef;

/* System configuration */
#define USE_HAL_DRIVER
#define VECT_TAB_OFFSET     0x0

/* Timing constants */
#define SYSTEM_CLOCK_FREQ   168000000
#define TICK_FREQ_HZ        1000

#endif
