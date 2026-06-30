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
 * X-Plane 11/12 Hardware-in-Loop Protocol Definition
 *
 * This module defines the data packet structures for communication between
 * the X-Plane plugin and Wingflight SITL firmware over UDP.
 *
 * Protocol Overview:
 * - X-Plane Plugin → Wingflight: FDM packets (sensor data)
 * - Wingflight → X-Plane Plugin: PWM packets (motor/servo commands)
 * - Both directions use UDP for low-latency communication
 * - Typical update rate: 20 Hz (50ms per frame)
 */

#ifndef __PROTOCOL_XPLANE_H
#define __PROTOCOL_XPLANE_H

#include <stdint.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 * PROTOCOL VERSION & MAGIC NUMBERS
 * ============================================================================ */

#define XPLANE_PROTOCOL_VERSION     1
#define XPLANE_FDM_MAGIC            0x4658504E  // "FXPN" (FDM X-Plane Wingflight)
#define XPLANE_PWM_MAGIC            0x5058504E  // "PXPN" (PWM X-Plane Wingflight)

#define XPLANE_HEARTBEAT_INTERVAL_MS  1000  // Heartbeat every 1 second
#define XPLANE_TIMEOUT_MS             5000  // Connection timeout after 5 seconds

/* ============================================================================
 * PACKET STRUCTURE: FDM DATA (X-Plane → Wingflight)
 *
 * Contains flight dynamics model data from X-Plane flight simulator.
 * Sent at ~20 Hz from X-Plane plugin to Wingflight SITL firmware.
 * ============================================================================ */

#pragma pack(1)

typedef struct {
    uint32_t magic;           // 0x4658504E ("FXPN")
    uint8_t version;          // Protocol version
    uint8_t flags;            // Packet flags (reserved for future use)
    uint16_t packet_size;     // Total packet size for validation
} xplane_packet_header_t;

typedef struct {
    // === PACKET HEADER (8 bytes) ===
    uint32_t magic;           // 0x4658504E ("FXPN")
    uint8_t version;          // Protocol version
    uint8_t flags;            // Reserved for future use
    uint16_t crc16;           // CRC16 checksum for error detection

    // === TIMING (8 bytes) ===
    uint32_t timestamp_ms;    // Millisecond timestamp from X-Plane
    float sim_speed;          // Simulation speed factor (1.0 = normal)

    // === ATTITUDE (12 bytes) ===
    float roll;               // Roll angle (radians, -π to +π)
    float pitch;              // Pitch angle (radians, -π/2 to +π/2)
    float yaw;                // Yaw angle (radians, -π to +π)

    // === ANGULAR RATES (12 bytes) ===
    float p;                  // Roll rate (rad/s)
    float q;                  // Pitch rate (rad/s)
    float r;                  // Yaw rate (rad/s)

    // === LINEAR ACCELERATION (12 bytes) ===
    // In body frame (aircraft coordinate system)
    float accel_x;            // X-axis acceleration (m/s²)
    float accel_y;            // Y-axis acceleration (m/s²)
    float accel_z;            // Z-axis acceleration (m/s²)

    // === GPS POSITION (16 bytes) ===
    double latitude;          // Latitude (decimal degrees)
    double longitude;         // Longitude (decimal degrees)
    float altitude_msl;       // Altitude above mean sea level (meters)
    uint8_t gps_fix_type;     // 0=no fix, 1=2D fix, 2=3D fix
    uint8_t num_satellites;   // Number of satellites in view
    uint16_t reserved1;       // Reserved for alignment

    // === GPS VELOCITY (12 bytes) ===
    float velocity_n;         // North velocity (m/s, NED frame)
    float velocity_e;         // East velocity (m/s, NED frame)
    float velocity_d;         // Down velocity (m/s, NED frame)

    // === AIRSPEED & WIND (12 bytes) ===
    float indicated_airspeed; // Airspeed from pitot tube (m/s)
    float true_airspeed;      // True airspeed (m/s)
    float ground_speed;       // Ground speed (m/s)

    // === BAROMETER & ALTITUDE (8 bytes) ===
    float pressure_pa;        // Barometric pressure (Pascals)
    float pressure_alt;       // Altitude from pressure altitude formula (meters)

    // === MAGNETOMETER (12 bytes) ===
    // Magnetic field in body frame (μT, microtesla)
    float mag_x;              // X-axis magnetic field
    float mag_y;              // Y-axis magnetic field
    float mag_z;              // Z-axis magnetic field

    // === BATTERY / POWER (12 bytes) ===
    float battery_voltage;    // Battery voltage (volts)
    float battery_current;    // Battery current (amperes)
    float battery_capacity;   // Battery capacity consumed (mAh)

    // === ENGINE / THROTTLE STATE (8 bytes) ===
    float throttle;           // Throttle position (0.0 to 1.0)
    float rpm;                // Engine RPM (0-10000)
    float fuel_total;         // Total fuel (liters, simulated)
    uint32_t engine_flags;    // Engine status flags

    // === LANDING GEAR / FLAPS (8 bytes) ===
    float flap_position;      // Flap deflection (0.0 to 1.0)
    float gear_deploy;        // Landing gear position (0.0=retracted, 1.0=deployed)
    uint16_t reserved2;       // Reserved for future use
    uint16_t reserved3;

    // === PAYLOAD / MISC (16 bytes) ===
    float wind_speed;         // Wind speed (m/s)
    float wind_direction;     // Wind direction (degrees, 0-359)
    float air_density;        // Air density (kg/m³)
    uint32_t system_time;     // System uptime (ms)

} xplane_fdm_packet_t;

/* ============================================================================
 * PACKET STRUCTURE: PWM OUTPUT DATA (Wingflight → X-Plane)
 *
 * Contains motor/servo PWM command values from Wingflight firmware.
 * Sent at ~20 Hz from Wingflight SITL to X-Plane plugin.
 * ============================================================================ */

#define XPLANE_PWM_CHANNELS 16  // Maximum PWM channels supported

typedef struct {
    // === PACKET HEADER (8 bytes) ===
    uint32_t magic;           // 0x5058504E ("PXPN")
    uint8_t version;          // Protocol version
    uint8_t num_channels;     // Number of active PWM channels
    uint16_t crc16;           // CRC16 checksum for error detection

    // === TIMING (4 bytes) ===
    uint32_t timestamp_ms;    // Firmware timestamp (milliseconds)

    // === PWM CHANNEL VALUES (32 bytes) ===
    // PWM values in microseconds (1000-2000 μs typical range)
    // For fixed-wing aircraft typical mapping:
    //   Channel 0: Throttle / Motor
    //   Channel 1: Aileron Left (differential)
    //   Channel 2: Aileron Right (differential)
    //   Channel 3: Elevator
    //   Channel 4: Rudder
    //   Channel 5: Flaps
    //   Channel 6-15: Auxiliary channels
    uint16_t pwm_values[XPLANE_PWM_CHANNELS];

    // === STATUS & FLAGS (4 bytes) ===
    uint8_t arm_state;        // 0=disarmed, 1=armed
    uint8_t flight_mode;      // Flight mode indicator
    uint8_t failsafe_active;  // 0=normal, 1=failsafe active
    uint8_t reserved;

} xplane_pwm_packet_t;

/* ============================================================================
 * HEARTBEAT / STATUS PACKET
 *
 * Lightweight keep-alive packet sent periodically to detect disconnections.
 * ============================================================================ */

typedef struct {
    uint32_t magic;           // 0x4558504E ("HXPN" - Heartbeat)
    uint32_t timestamp_ms;    // Heartbeat timestamp
    uint16_t crc16;           // CRC16 for validation
    uint16_t reserved;

} xplane_heartbeat_packet_t;

#pragma pack()

/* ============================================================================
 * UTILITY MACROS & CONSTANTS
 * ============================================================================ */

// Packet size validation
#define XPLANE_FDM_PACKET_SIZE    sizeof(xplane_fdm_packet_t)
#define XPLANE_PWM_PACKET_SIZE    sizeof(xplane_pwm_packet_t)
#define XPLANE_HEARTBEAT_SIZE     sizeof(xplane_heartbeat_packet_t)

// PWM Value Conversion
#define PWM_MIN_VALUE             1000  // Minimum PWM pulse (μs)
#define PWM_MAX_VALUE             2000  // Maximum PWM pulse (μs)
#define PWM_CENTER_VALUE          1500  // Center PWM value
#define PWM_RANGE                 (PWM_MAX_VALUE - PWM_MIN_VALUE)

// Conversion constants
#define DEG_TO_RAD                (3.14159265359f / 180.0f)
#define RAD_TO_DEG                (180.0f / 3.14159265359f)

/* ============================================================================
 * FUNCTION PROTOTYPES
 * ============================================================================ */

/**
 * Calculate CRC16 checksum for packet validation
 * @param data Pointer to data buffer
 * @param size Size of data in bytes
 * @return CRC16 checksum value
 */
uint16_t xplane_crc16(const uint8_t* data, size_t size);

/**
 * Validate FDM packet
 * @param packet Pointer to FDM packet
 * @return 1 if valid, 0 if invalid
 */
int xplane_validate_fdm_packet(const xplane_fdm_packet_t* packet);

/**
 * Validate PWM packet
 * @param packet Pointer to PWM packet
 * @return 1 if valid, 0 if invalid
 */
int xplane_validate_pwm_packet(const xplane_pwm_packet_t* packet);

/**
 * Initialize FDM packet with default values
 * @param packet Pointer to FDM packet to initialize
 */
void xplane_init_fdm_packet(xplane_fdm_packet_t* packet);

/**
 * Initialize PWM packet with default values (neutral)
 * @param packet Pointer to PWM packet to initialize
 */
void xplane_init_pwm_packet(xplane_pwm_packet_t* packet);

/**
 * Update packet CRC using the packet's actual crc16 field location.
 * @param packet Pointer to packet storage
 * @param packet_size Size of packet structure
 */
void xplane_update_packet_crc(void* packet, size_t packet_size);

#ifdef __cplusplus
} // extern "C"
#endif

#endif // __PROTOCOL_XPLANE_H
