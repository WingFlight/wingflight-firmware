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
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#if defined(_WIN32) || defined(__MINGW32__)
// Trim down windows.h (pulled in transitively via winsock2.h) so it does not
// define the legacy serial-comm BAUD_* macros (from winbase.h), which clash
// with the BAUD_* enum values declared in io/serial.h.
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOCOMM
#define NOCOMM
#endif
#include <winsock2.h>
#include <ws2tcpip.h>
// windows.h (via winbase.h) unconditionally defines legacy COMM BAUD_* macros
// that collide with the BAUD_* values of io/serial.h's baudRate_e enum.
// They are never used by this simulator build, so drop them.
#undef BAUD_075
#undef BAUD_110
#undef BAUD_134_5
#undef BAUD_150
#undef BAUD_300
#undef BAUD_600
#undef BAUD_1200
#undef BAUD_1800
#undef BAUD_2400
#undef BAUD_4800
#undef BAUD_7200
#undef BAUD_9600
#undef BAUD_14400
#undef BAUD_19200
#undef BAUD_38400
#undef BAUD_56K
#undef BAUD_128K
#undef BAUD_115200
#undef BAUD_57600
#undef BAUD_USER
#else
#include <netinet/in.h>
#include <sys/socket.h>
#endif
#include <pthread.h>

#define RX_BUFFER_SIZE    1400

// Blocking-socket, thread-per-port model (mirrors iNav's src/main/drivers/serial_tcp.c),
// chosen over a non-blocking single-threaded reactor (previously dyad) after the latter
// was found to suffer from mysterious, highly-reproducible mid-session connection drops
// on Windows/MinGW loopback sockets that could not be root-caused. See Changes.md.
#if defined(_WIN32) || defined(__MINGW32__)
typedef SOCKET tcpSocket_t;
#define TCP_INVALID_SOCKET INVALID_SOCKET
#else
typedef int tcpSocket_t;
#define TCP_INVALID_SOCKET (-1)
#endif

typedef struct {
    serialPort_t port;
    uint8_t rxBuffer[RX_BUFFER_SIZE];

    tcpSocket_t listenSocket;
    tcpSocket_t clientSocket;
    pthread_t acceptThread;
    pthread_mutex_t connLock;
    pthread_mutex_t rxLock;
    bool connected;
    uint8_t id;
} tcpPort_t;

serialPort_t *serTcpOpen(int id, serialReceiveCallbackPtr rxCallback, void *rxCallbackData, uint32_t baudRate, portMode_e mode, portOptions_e options);

// tcpPort API
void tcpDataIn(tcpPort_t *instance, uint8_t* ch, int size);

bool tcpIsStart(void);
bool* tcpGetUsed(void);
tcpPort_t* tcpGetPool(void);

