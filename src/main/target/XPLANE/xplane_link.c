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
 * X-Plane HITL UDP Communication Implementation
 */

#include "xplane_link.h"
#include "protocol_xplane.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <time.h>

/* Platform-specific includes */
#ifdef _WIN32
    #include <winsock2.h>
    #include <ws2tcpip.h>
    #include <sys/types.h>
    #include <sys/timeb.h>
    typedef int socklen_t;
    #define CLOSE_SOCKET(s) closesocket(s)
    #define IOCTLSOCKET ioctlsocket
    #define u_long unsigned long
#else
    #include <sys/socket.h>
    #include <netinet/in.h>
    #include <arpa/inet.h>
    #include <fcntl.h>
    #include <unistd.h>
    #include <time.h>
    typedef int SOCKET;
    #define CLOSE_SOCKET(s) close(s)
    #define SOCKET_ERROR (-1)
    #define INVALID_SOCKET (-1)
    #define IOCTLSOCKET fcntl
    #define u_long unsigned long
#endif

/* ============================================================================
 * STATIC STATE
 * ============================================================================ */

static int debug_enabled = 0;

/**
 * Get current time in milliseconds (cross-platform)
 */
static uint32_t get_current_time_ms(void) {
#ifdef _WIN32
    struct _timeb tb;
    _ftime_s(&tb);
    return (uint32_t)(tb.time * 1000 + tb.millitm);
#else
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint32_t)((ts.tv_sec * 1000) + (ts.tv_nsec / 1000000));
#endif
}

/**
 * Convert IPv4 address string to network byte order
 */
static int inet_aton_compat(const char* cp, struct in_addr* inp) {
    inp->s_addr = inet_addr(cp);
    return (inp->s_addr != INADDR_NONE) ? 1 : 0;
}

/* ============================================================================
 * SOCKET OPERATIONS
 * ============================================================================ */

/**
 * Initialize UDP socket with non-blocking mode
 */
static int create_udp_socket(int port) {
    SOCKET sock;
    struct sockaddr_in addr;
    int reuse = 1;
    
    // Create UDP socket
    sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock == INVALID_SOCKET) {
        if (debug_enabled) {
            fprintf(stderr, "[XPLANE_LINK] Failed to create socket: %s\n", strerror(errno));
        }
        return -1;
    }
    
    // Allow socket reuse (for quick restart)
    if (setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, (const char*)&reuse, sizeof(reuse)) < 0) {
        if (debug_enabled) {
            fprintf(stderr, "[XPLANE_LINK] Failed to set SO_REUSEADDR: %s\n", strerror(errno));
        }
    }
    
    // Set non-blocking mode
#ifdef _WIN32
    u_long mode = 1;
    if (ioctlsocket(sock, FIONBIO, &mode) != 0) {
        if (debug_enabled) {
            fprintf(stderr, "[XPLANE_LINK] Failed to set non-blocking mode\n");
        }
        CLOSE_SOCKET(sock);
        return -1;
    }
#else
    int flags = fcntl(sock, F_GETFL, 0);
    if (flags == -1 || fcntl(sock, F_SETFL, flags | O_NONBLOCK) == -1) {
        if (debug_enabled) {
            fprintf(stderr, "[XPLANE_LINK] Failed to set non-blocking mode\n");
        }
        close(sock);
        return -1;
    }
#endif
    
    // Bind to local port if specified
    if (port > 0) {
        memset(&addr, 0, sizeof(addr));
        addr.sin_family = AF_INET;
        addr.sin_addr.s_addr = htonl(INADDR_ANY);
        addr.sin_port = htons(port);
        
        if (bind(sock, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
            if (debug_enabled) {
                fprintf(stderr, "[XPLANE_LINK] Failed to bind socket to port %d: %s\n", 
                       port, strerror(errno));
            }
            CLOSE_SOCKET(sock);
            return -1;
        }
        
        if (debug_enabled) {
            printf("[XPLANE_LINK] Socket bound to port %d\n", port);
        }
    }
    
    return (int)sock;
}

/* ============================================================================
 * PUBLIC API IMPLEMENTATION
 * ============================================================================ */

int xplane_link_init(xplane_link_t* link, int local_port_in, int local_port_out,
                     const char* remote_addr) {
    if (link == NULL || remote_addr == NULL) {
        return -1;
    }
    
    memset(link, 0, sizeof(xplane_link_t));
    
    // Create receive socket for FDM packets
    link->socket_fd = create_udp_socket(local_port_in);
    if (link->socket_fd < 0) {
        fprintf(stderr, "[XPLANE_LINK] Failed to create UDP socket\n");
        return -1;
    }
    
    link->local_port_in = local_port_in;
    link->local_port_out = local_port_out;
    link->remote_addr = remote_addr;
    link->remote_port_in = 5502;   // X-Plane FDM output port (default)
    link->remote_port_out = 5503;  // X-Plane PWM input port (default)
    link->last_packet_time = get_current_time_ms();
    link->is_connected = false;
    
    if (debug_enabled) {
        printf("[XPLANE_LINK] Initialized: listen on %d, send to %s:%d\n",
               local_port_in, remote_addr, link->remote_port_out);
    }
    
    return 0;
}

int xplane_link_close(xplane_link_t* link) {
    if (link == NULL || link->socket_fd < 0) {
        return -1;
    }
    
    CLOSE_SOCKET((SOCKET)link->socket_fd);
    link->socket_fd = -1;
    link->is_connected = false;
    
    if (debug_enabled) {
        printf("[XPLANE_LINK] Socket closed\n");
    }
    
    return 0;
}

int xplane_link_is_connected(xplane_link_t* link) {
    if (link == NULL) {
        return 0;
    }
    return link->is_connected ? 1 : 0;
}

int xplane_link_recv_fdm(xplane_link_t* link, void* buffer, size_t max_size,
                         uint32_t timeout_ms) {
    struct sockaddr_in remote_addr;
    socklen_t remote_addr_len;
    int bytes_received;
    
    (void)timeout_ms;  // Suppress unused parameter warning (non-blocking socket)
    
    if (link == NULL || buffer == NULL || link->socket_fd < 0) {
        return -1;
    }
    
    if (max_size < XPLANE_FDM_PACKET_SIZE) {
        if (debug_enabled) {
            fprintf(stderr, "[XPLANE_LINK] Buffer too small (need %zu, got %zu)\n",
                   XPLANE_FDM_PACKET_SIZE, max_size);
        }
        return -1;
    }
    
    remote_addr_len = sizeof(remote_addr);
    memset(&remote_addr, 0, sizeof(remote_addr));
    
    // Receive packet (non-blocking)
    bytes_received = recvfrom((SOCKET)link->socket_fd, (char*)buffer, max_size, 0,
                             (struct sockaddr*)&remote_addr, &remote_addr_len);
    
    if (bytes_received <= 0) {
        // No data available or error (non-blocking socket returns -1 on no data)
        return 0;
    }
    
    // In host HITL mode, do not block inbound traffic on local validation mismatch.
    xplane_fdm_packet_t* fdm_pkt = (xplane_fdm_packet_t*)buffer;
    if (!xplane_validate_fdm_packet(fdm_pkt) && debug_enabled) {
        fprintf(stderr, "[XPLANE_LINK] Warning: FDM packet failed local validation, accepting anyway\n");
    }
    
    // Update connection state
    link->last_packet_time = get_current_time_ms();
    link->is_connected = true;
    link->packet_count++;
    
    if (debug_enabled && (link->packet_count % 20 == 0)) {
        printf("[XPLANE_LINK] Received FDM packet #%u from %s:%u\n",
               link->packet_count, inet_ntoa(remote_addr.sin_addr),
               ntohs(remote_addr.sin_port));
    }
    
    return bytes_received;
}

int xplane_link_send_pwm(xplane_link_t* link, const void* data, size_t size) {
    struct sockaddr_in remote_addr;
    int bytes_sent;
    
    if (link == NULL || data == NULL || link->socket_fd < 0) {
        return -1;
    }
    
    if (size != XPLANE_PWM_PACKET_SIZE) {
        if (debug_enabled) {
            fprintf(stderr, "[XPLANE_LINK] Invalid PWM packet size: %zu\n", size);
        }
        return -1;
    }
    
    // In host HITL mode, do not block outbound traffic on local validation mismatch.
    if (!xplane_validate_pwm_packet((xplane_pwm_packet_t*)data) && debug_enabled) {
        fprintf(stderr, "[XPLANE_LINK] Warning: PWM packet failed local validation, sending anyway\n");
    }
    
    // Create remote address
    memset(&remote_addr, 0, sizeof(remote_addr));
    remote_addr.sin_family = AF_INET;
    remote_addr.sin_port = htons(link->remote_port_out);
    
    // Convert address string to binary
    if (!inet_aton_compat(link->remote_addr, &remote_addr.sin_addr)) {
        if (debug_enabled) {
            fprintf(stderr, "[XPLANE_LINK] Invalid remote address: %s\n", link->remote_addr);
        }
        return -1;
    }
    
    // Send packet
    bytes_sent = sendto((SOCKET)link->socket_fd, (const char*)data, size, 0,
                       (struct sockaddr*)&remote_addr, sizeof(remote_addr));
    
    if (bytes_sent < 0) {
        if (debug_enabled) {
            fprintf(stderr, "[XPLANE_LINK] Failed to send PWM packet: %s\n", strerror(errno));
        }
        link->error_count++;
        return -1;
    }
    
    if (debug_enabled && (link->packet_count % 20 == 0)) {
        printf("[XPLANE_LINK] Sent PWM packet to %s:%u (%d bytes)\n",
               link->remote_addr, link->remote_port_out, bytes_sent);
    }
    
    return bytes_sent;
}

int xplane_link_update_status(xplane_link_t* link, uint32_t timeout_ms) {
    uint32_t current_time;
    uint32_t time_since_last_packet;
    
    if (link == NULL) {
        return 0;
    }
    
    current_time = get_current_time_ms();
    time_since_last_packet = current_time - link->last_packet_time;
    
    // Check for timeout
    if (time_since_last_packet > timeout_ms) {
        if (link->is_connected) {
            if (debug_enabled) {
                printf("[XPLANE_LINK] Connection timeout (no packet for %ums)\n",
                       time_since_last_packet);
            }
            link->is_connected = false;
        }
        return 0;
    }
    
    return 1;
}

uint32_t xplane_link_get_stats(xplane_link_t* link, uint32_t* packet_count,
                               uint32_t* error_count) {
    uint32_t current_time;
    uint32_t ms_since_packet;
    
    if (link == NULL) {
        return 0;
    }
    
    current_time = get_current_time_ms();
    ms_since_packet = current_time - link->last_packet_time;
    
    if (packet_count != NULL) {
        *packet_count = link->packet_count;
    }
    
    if (error_count != NULL) {
        *error_count = link->error_count;
    }
    
    return ms_since_packet;
}

void xplane_link_print_status(xplane_link_t* link) {
    if (link == NULL) {
        return;
    }
    
    uint32_t packets, errors;
    uint32_t ms_since_packet = xplane_link_get_stats(link, &packets, &errors);
    
    printf("\n=== X-Plane HITL Link Status ===\n");
    printf("Connected: %s\n", link->is_connected ? "YES" : "NO");
    printf("Local Port: %d\n", link->local_port_in);
    printf("Remote: %s:%d\n", link->remote_addr, link->remote_port_out);
    printf("Packets Received: %u\n", packets);
    printf("Errors: %u\n", errors);
    printf("Time Since Last Packet: %ums\n", ms_since_packet);
    printf("================================\n\n");
}

void xplane_link_set_debug(int enable) {
    debug_enabled = enable;
}
