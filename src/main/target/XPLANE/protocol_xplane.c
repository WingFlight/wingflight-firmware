/**
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
 * X-Plane HITL Protocol Implementation
 *
 * Provides utility functions for packet creation, validation, and CRC computation.
 */

#include "protocol_xplane.h"
#include <stddef.h>
#include <string.h>

static uint16_t crc_with_field_cleared(const uint8_t* packet_bytes, size_t packet_size, size_t crc_offset)
{
    uint16_t crc = 0xFFFF;

    for (size_t i = 0; i < packet_size; i++) {
        uint8_t byte = packet_bytes[i];
        if (i == crc_offset || i == (crc_offset + 1U)) {
            byte = 0;
        }

        crc ^= (uint16_t)(byte << 8);

        for (int j = 0; j < 8; j++) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc = crc << 1;
            }
            crc &= 0xFFFF;
        }
    }

    return crc;
}

/* ============================================================================
 * CRC16-CCITT CALCULATION
 * ============================================================================ */

/**
 * Calculate CRC16-CCITT checksum
 * Polynomial: 0x1021 (CRC-CCITT)
 * Initial value: 0xFFFF
 *
 * @param data Pointer to data buffer
 * @param size Size of data in bytes
 * @return CRC16 checksum value
 */
uint16_t xplane_crc16(const uint8_t* data, size_t size) {
    uint16_t crc = 0xFFFF;
    
    for (size_t i = 0; i < size; i++) {
        crc ^= (data[i] << 8);
        
        for (int j = 0; j < 8; j++) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc = crc << 1;
            }
            crc &= 0xFFFF;
        }
    }
    
    return crc;
}

/* ============================================================================
 * PACKET VALIDATION
 * ============================================================================ */

/**
 * Validate FDM packet integrity
 *
 * Checks:
 * - Magic number matches XPLANE_FDM_MAGIC
 * - Protocol version is supported
 * - CRC16 checksum is correct
 *
 * @param packet Pointer to FDM packet
 * @return 1 if valid, 0 if invalid
 */
int xplane_validate_fdm_packet(const xplane_fdm_packet_t* packet) {
    if (packet == NULL) {
        return 0;
    }
    
    // Check magic number
    if (packet->magic != XPLANE_FDM_MAGIC) {
        return 0;
    }
    
    // Check version (should be compatible)
    if (packet->version != XPLANE_PROTOCOL_VERSION) {
        // Version mismatch - could be warning instead of failure
        // For now, we'll be strict
        return 0;
    }
    
    const uint8_t* packet_bytes = (const uint8_t*)packet;
    uint16_t computed_crc = crc_with_field_cleared(packet_bytes, sizeof(xplane_fdm_packet_t), offsetof(xplane_fdm_packet_t, crc16));
    
    if (computed_crc != packet->crc16) {
        return 0;
    }
    
    return 1;
}

/**
 * Validate PWM packet integrity
 *
 * Checks:
 * - Magic number matches XPLANE_PWM_MAGIC
 * - Protocol version is supported
 * - CRC16 checksum is correct
 * - Number of channels is valid (1-16)
 *
 * @param packet Pointer to PWM packet
 * @return 1 if valid, 0 if invalid
 */
int xplane_validate_pwm_packet(const xplane_pwm_packet_t* packet) {
    if (packet == NULL) {
        return 0;
    }
    
    // Check magic number
    if (packet->magic != XPLANE_PWM_MAGIC) {
        return 0;
    }
    
    // Check version
    if (packet->version != XPLANE_PROTOCOL_VERSION) {
        return 0;
    }
    
    // Check channel count (must be 1-16)
    if (packet->num_channels < 1 || packet->num_channels > XPLANE_PWM_CHANNELS) {
        return 0;
    }
    
    const uint8_t* packet_bytes = (const uint8_t*)packet;
    uint16_t computed_crc = crc_with_field_cleared(packet_bytes, sizeof(xplane_pwm_packet_t), offsetof(xplane_pwm_packet_t, crc16));
    
    if (computed_crc != packet->crc16) {
        return 0;
    }
    
    return 1;
}

/* ============================================================================
 * PACKET INITIALIZATION
 * ============================================================================ */

/**
 * Initialize FDM packet with sensible defaults
 *
 * Sets:
 * - Magic number and version
 * - Packet header fields
 * - All sensor values to safe defaults (level, stationary at sea level)
 * - CRC16 checksum
 *
 * @param packet Pointer to FDM packet to initialize
 */
void xplane_init_fdm_packet(xplane_fdm_packet_t* packet) {
    memset(packet, 0, sizeof(xplane_fdm_packet_t));
    
    // Header
    packet->magic = XPLANE_FDM_MAGIC;
    packet->version = XPLANE_PROTOCOL_VERSION;
    packet->flags = 0;
    
    // Timing
    packet->timestamp_ms = 0;
    packet->sim_speed = 1.0f;
    
    // Attitude: level flight, heading north
    packet->roll = 0.0f;
    packet->pitch = 0.0f;
    packet->yaw = 0.0f;
    
    // Rates: steady
    packet->p = 0.0f;
    packet->q = 0.0f;
    packet->r = 0.0f;
    
    // Acceleration: 1g down in level flight
    packet->accel_x = 0.0f;
    packet->accel_y = 0.0f;
    packet->accel_z = 9.81f;
    
    // GPS: stationary at sea level, equator
    packet->latitude = 0.0;
    packet->longitude = 0.0;
    packet->altitude_msl = 0.0f;
    packet->gps_fix_type = 2;  // 3D fix
    packet->num_satellites = 12;
    
    // Velocity: not moving
    packet->velocity_n = 0.0f;
    packet->velocity_e = 0.0f;
    packet->velocity_d = 0.0f;
    
    // Airspeed: zero
    packet->indicated_airspeed = 0.0f;
    packet->true_airspeed = 0.0f;
    packet->ground_speed = 0.0f;
    
    // Barometer: sea level standard
    packet->pressure_pa = 101325.0f;
    packet->pressure_alt = 0.0f;
    
    // Compass: pointing north
    packet->mag_x = 25.0f;
    packet->mag_y = 0.0f;
    packet->mag_z = -45.0f;
    
    // Battery: fresh state
    packet->battery_voltage = 0.0f;
    packet->battery_current = 0.0f;
    packet->battery_capacity = 0.0f;
    
    // Engine
    packet->throttle = 0.0f;
    packet->rpm = 0.0f;
    packet->fuel_total = 0.0f;
    packet->engine_flags = 0;
    
    // Control surfaces
    packet->flap_position = 0.0f;
    packet->gear_deploy = 0.0f;
    
    // Wind and misc
    packet->wind_speed = 0.0f;
    packet->wind_direction = 0.0f;
    packet->air_density = 1.225f;  // Sea level
    packet->system_time = 0;
    
    xplane_update_packet_crc(packet, sizeof(xplane_fdm_packet_t));
}

/**
 * Initialize PWM packet with neutral values
 *
 * Sets:
 * - Magic number and version
 * - All PWM channels to center position (1500 μs)
 * - Disarmed state, manual mode, no failsafe
 * - CRC16 checksum
 *
 * @param packet Pointer to PWM packet to initialize
 */
void xplane_init_pwm_packet(xplane_pwm_packet_t* packet) {
    memset(packet, 0, sizeof(xplane_pwm_packet_t));
    
    // Header
    packet->magic = XPLANE_PWM_MAGIC;
    packet->version = XPLANE_PROTOCOL_VERSION;
    packet->num_channels = XPLANE_PWM_CHANNELS;  // Use all 16 channels
    
    // Timing
    packet->timestamp_ms = 0;
    
    // Initialize all channels to neutral (1500 μs / center)
    for (int i = 0; i < XPLANE_PWM_CHANNELS; i++) {
        packet->pwm_values[i] = PWM_CENTER_VALUE;
    }
    
    // Status flags: safe defaults
    packet->arm_state = 0;         // Disarmed
    packet->flight_mode = 0;       // Manual mode
    packet->failsafe_active = 0;   // Normal operation
    packet->reserved = 0;
    
    xplane_update_packet_crc(packet, sizeof(xplane_pwm_packet_t));
}

/* ============================================================================
 * HELPER FUNCTIONS FOR PACKET OPERATIONS
 * ============================================================================ */

/**
 * Update CRC16 checksum for a packet
 * Call this after modifying packet contents
 *
 * @param packet Pointer to packet (FDM or PWM)
 * @param packet_size Size of packet structure
 */
void xplane_update_packet_crc(void* packet, size_t packet_size) {
    if (packet == NULL) {
        return;
    }

    uint8_t* packet_bytes = (uint8_t*)packet;
    size_t crc_offset;

    if (packet_size == sizeof(xplane_fdm_packet_t)) {
        crc_offset = offsetof(xplane_fdm_packet_t, crc16);
    } else if (packet_size == sizeof(xplane_pwm_packet_t)) {
        crc_offset = offsetof(xplane_pwm_packet_t, crc16);
    } else {
        return;
    }

    uint16_t* crc_field = (uint16_t*)(packet_bytes + crc_offset);
    *crc_field = 0;
    *crc_field = crc_with_field_cleared(packet_bytes, packet_size, crc_offset);
}

/**
 * Convert PWM value to normalized control input
 * Useful for interfacing with flight control algorithms
 *
 * Formula: normalized = (pwm - 1000) / 1000 - 1.0
 *   Range: -1.0 (1000 μs) to +1.0 (2000 μs)
 *
 * @param pwm PWM value in microseconds (1000-2000)
 * @return Normalized value (-1.0 to +1.0)
 */
float xplane_pwm_to_normalized(uint16_t pwm) {
    if (pwm < PWM_MIN_VALUE) pwm = PWM_MIN_VALUE;
    if (pwm > PWM_MAX_VALUE) pwm = PWM_MAX_VALUE;
    
    return ((float)(pwm - PWM_MIN_VALUE) / (float)PWM_RANGE) * 2.0f - 1.0f;
}

/**
 * Convert normalized control input to PWM value
 * Useful for sending commands from flight controller
 *
 * Formula: pwm = 1000 + (normalized + 1.0) * 500
 *   Range: -1.0 to +1.0 maps to 1000-2000 μs
 *
 * @param normalized Normalized value (-1.0 to +1.0)
 * @return PWM value in microseconds (1000-2000)
 */
uint16_t xplane_normalized_to_pwm(float normalized) {
    if (normalized < -1.0f) normalized = -1.0f;
    if (normalized > 1.0f) normalized = 1.0f;
    
    uint16_t pwm = (uint16_t)(1000 + (normalized + 1.0f) * 500.0f);
    return pwm;
}

/**
 * Set PWM channel value with CRC update
 * Safe way to modify individual PWM channel
 *
 * @param packet Pointer to PWM packet
 * @param channel Channel index (0-15)
 * @param pwm_value PWM value in microseconds (1000-2000)
 * @return 1 if success, 0 if invalid channel
 */
int xplane_set_pwm_channel(xplane_pwm_packet_t* packet, uint8_t channel, uint16_t pwm_value) {
    if (packet == NULL || channel >= XPLANE_PWM_CHANNELS) {
        return 0;
    }
    
    // Clamp PWM value to valid range
    if (pwm_value < PWM_MIN_VALUE) pwm_value = PWM_MIN_VALUE;
    if (pwm_value > PWM_MAX_VALUE) pwm_value = PWM_MAX_VALUE;
    
    packet->pwm_values[channel] = pwm_value;
    
    // Update CRC
    xplane_update_packet_crc(packet, sizeof(xplane_pwm_packet_t));
    
    return 1;
}
