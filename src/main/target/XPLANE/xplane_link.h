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
 * X-Plane HITL UDP Communication Link
 *
 * Handles UDP communication with X-Plane plugin:
 * - Receives FDM packets (flight data model / sensor data)
 * - Sends PWM packets (motor/servo commands)
 * - Connection management and error handling
 * - Thread-safe packet handling
 */

#ifndef __XPLANE_LINK_H
#define __XPLANE_LINK_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 * UDP SOCKET STRUCTURE
 * ============================================================================ */

typedef struct {
    int socket_fd;              // UDP socket file descriptor
    int local_port_in;          // Local port for receiving (FDM packets)
    int local_port_out;         // Local port for sending (PWM packets)
    int remote_port_in;         // Remote X-Plane plugin FDM port
    int remote_port_out;        // Remote X-Plane plugin PWM port
    const char* remote_addr;    // Remote address (typically "127.0.0.1")
    bool is_connected;          // Connection status
    uint32_t last_packet_time;  // Timestamp of last received packet (ms)
    uint32_t packet_count;      // Number of packets received
    uint32_t error_count;       // Number of errors
} xplane_link_t;

/* ============================================================================
 * INITIALIZATION & CONNECTION
 * ============================================================================ */

/**
 * Initialize X-Plane HITL link
 * Sets up UDP sockets for bidirectional communication
 *
 * @param link Pointer to xplane_link_t structure
 * @param local_port_in Local port for receiving FDM packets (typically 5502)
 * @param local_port_out Local port for sending PWM packets (typically 5503)
 * @param remote_addr Remote address ("127.0.0.1" for localhost)
 * @return 0 on success, -1 on failure
 */
int xplane_link_init(xplane_link_t* link, int local_port_in, int local_port_out, 
                     const char* remote_addr);

/**
 * Close X-Plane HITL link and cleanup resources
 *
 * @param link Pointer to xplane_link_t structure
 * @return 0 on success, -1 on failure
 */
int xplane_link_close(xplane_link_t* link);

/**
 * Check if link is connected and healthy
 *
 * @param link Pointer to xplane_link_t structure
 * @return 1 if connected, 0 if disconnected
 */
int xplane_link_is_connected(xplane_link_t* link);

/* ============================================================================
 * PACKET TRANSMISSION & RECEPTION
 * ============================================================================ */

/**
 * Receive FDM packet from X-Plane plugin
 * Non-blocking receive with optional timeout
 *
 * @param link Pointer to xplane_link_t structure
 * @param buffer Buffer to store received data
 * @param max_size Maximum size of buffer
 * @param timeout_ms Timeout in milliseconds (0 = non-blocking)
 * @return Number of bytes received, 0 if no data, -1 on error
 */
int xplane_link_recv_fdm(xplane_link_t* link, void* buffer, size_t max_size, 
                         uint32_t timeout_ms);

/**
 * Send PWM packet to X-Plane plugin
 * Sends motor/servo commands for aircraft control
 *
 * @param link Pointer to xplane_link_t structure
 * @param data Pointer to PWM packet data
 * @param size Size of PWM packet in bytes
 * @return Number of bytes sent, -1 on error
 */
int xplane_link_send_pwm(xplane_link_t* link, const void* data, size_t size);

/* ============================================================================
 * CONNECTION MONITORING
 * ============================================================================ */

/**
 * Update connection status (check for timeouts)
 * Should be called periodically to detect disconnections
 *
 * @param link Pointer to xplane_link_t structure
 * @param timeout_ms Timeout threshold in milliseconds (typically 5000)
 * @return 1 if still connected, 0 if timeout occurred
 */
int xplane_link_update_status(xplane_link_t* link, uint32_t timeout_ms);

/**
 * Get connection statistics
 *
 * @param link Pointer to xplane_link_t structure
 * @param packet_count Pointer to store packet count
 * @param error_count Pointer to store error count
 * @return Milliseconds since last packet
 */
uint32_t xplane_link_get_stats(xplane_link_t* link, uint32_t* packet_count, 
                               uint32_t* error_count);

/* ============================================================================
 * DIAGNOSTIC & DEBUG FUNCTIONS
 * ============================================================================ */

/**
 * Print connection status to stdout
 *
 * @param link Pointer to xplane_link_t structure
 */
void xplane_link_print_status(xplane_link_t* link);

/**
 * Enable/disable debug logging
 *
 * @param enable 1 to enable, 0 to disable
 */
void xplane_link_set_debug(int enable);

#ifdef __cplusplus
} // extern "C"
#endif

#endif // __XPLANE_LINK_H
