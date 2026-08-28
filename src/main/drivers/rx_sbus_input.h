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
 * If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

// A secondary, independent SBUS receiver input used as an instant fallback when the
// main RX link's signal is lost. See rx/rx.c's detectAndApplySignalLossBehaviour()
// for where this is consumed. Electrical settings (inversion/pin swap) are its own
// config, pg/rx_sbus_input.h - independent from the main RX's serialrx_inverted/
// serialrx_pinswap, since this is a different physical UART.
//
// This intentionally does not reuse rx/sbus.c's sbusInit()/sbusDataReceive() - that
// module keeps its frame-assembly state in function-local statics tied to the single
// main RX instance (and touches other main-RX-only globals like rssiSource), so it
// cannot be safely instantiated a second time. Only the reentrant channel decode in
// rx/sbus_channels.c (sbusChannelsDecode()) is shared between the two.

#define SBUS_INPUT_MAX_CHANNEL 18

void sbusInputInit(void);

// True once a serial port has been assigned FUNCTION_RX_SBUS_INPUT.
bool sbusInputIsEnabled(void);

// True when enabled AND a valid SBUS frame has been decoded within the last
// SBUS_INPUT_STALE_MS - i.e. the SBUS-in link is currently healthy and its
// channel data should be trusted as a fallback for the main RX.
bool sbusInputIsActive(void);

// Number of channels the SBUS-in decoder provides (always SBUS_INPUT_MAX_CHANNEL).
uint8_t sbusInputGetChannelCount(void);

// Channel value in the same convention as rx/rx.c's rcInput[]/rcChannel[] (~880-2012us).
float sbusInputGetChannel(uint8_t channel);
