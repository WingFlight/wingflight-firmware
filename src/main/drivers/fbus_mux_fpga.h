/*
 * This file is part of Wingflight.
 */

#pragma once

#include <stdbool.h>

#include "common/time.h"

void fbusMuxFpgaInit(void);
bool fbusMuxFpgaIsReady(void);
uint16_t fbusMuxFpgaGetConfigWord(void);
void fbusMuxFpgaSendConfigWord(uint16_t configWord);
void fbusMuxFpgaOnFbusTxStart(timeUs_t currentTimeUs, uint16_t txBytes, uint32_t baudRate);
void fbusMuxFpgaOnFbusTxUpdate(timeUs_t currentTimeUs, bool txBufferEmpty);
