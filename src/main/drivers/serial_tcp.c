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

/*
 * Authors:
 * Dominic Clifton - Serial port abstraction, Separation of common STM32 code for cleanflight, various cleanups.
 * Hamasaki/Timecop - Initial baseflight code
*/
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>

#include "platform.h"

#include "build/build_config.h"

#include "common/utils.h"

#include "io/serial.h"
#include "serial_tcp.h"

#if !defined(_WIN32) && !defined(__MINGW32__)
#include <unistd.h>
#define closesocket close
#endif

#define BASE_PORT 5760

static const struct serialPortVTable tcpVTable; // Forward
static tcpPort_t tcpSerialPorts[SERIAL_PORT_COUNT];
static bool tcpPortInitialized[SERIAL_PORT_COUNT];
static bool tcpStart = false;
bool tcpIsStart(void) {
    return tcpStart;
}

// Blocking-socket, thread-per-port connection handling, modeled directly on
// iNav's proven SITL implementation (src/main/drivers/serial_tcp.c there):
// one dedicated thread per port blocks in accept()/recv() rather than
// multiplexing all ports through a single non-blocking reactor (previously
// dyad). See serial_tcp.h for the rationale.
static void* tcpAcceptThread(void *arg)
{
    tcpPort_t *s = (tcpPort_t*)arg;

    for (;;) {
        struct sockaddr_in clientAddr;
        socklen_t clientLen = sizeof(clientAddr);
        tcpSocket_t clientSocket = accept(s->listenSocket, (struct sockaddr*)&clientAddr, &clientLen);
        if (clientSocket == TCP_INVALID_SOCKET) {
            // The listening socket itself is gone (shouldn't normally happen
            // for the lifetime of the process) - stop trying to accept.
            break;
        }

        int one = 1;
        setsockopt(clientSocket, IPPROTO_TCP, TCP_NODELAY, (const char*)&one, sizeof(one));

        // Discard any bytes left over from a previous client's connection so
        // they don't get prepended to the new client's first read.
        pthread_mutex_lock(&s->rxLock);
        s->port.rxBufferHead = 0;
        s->port.rxBufferTail = 0;
        pthread_mutex_unlock(&s->rxLock);

        pthread_mutex_lock(&s->connLock);
        s->clientSocket = clientSocket;
        s->connected = true;
        pthread_mutex_unlock(&s->connLock);

        fprintf(stderr, "[NEW]UART%u\n", s->id + 1);

        for (;;) {
            uint8_t buf[512];
            int n = recv(clientSocket, (char*)buf, sizeof(buf), 0);
            if (n <= 0) {
                break;
            }
            tcpDataIn(s, buf, n);
        }

        fprintf(stderr, "[CLS]UART%u\n", s->id + 1);

        pthread_mutex_lock(&s->connLock);
        s->connected = false;
        s->clientSocket = TCP_INVALID_SOCKET;
        pthread_mutex_unlock(&s->connLock);

        closesocket(clientSocket);
    }

    return NULL;
}

static tcpPort_t* tcpReconfigure(tcpPort_t *s, int id)
{
    if (tcpPortInitialized[id]) {
        fprintf(stderr, "port is already initialized!\n");
        return s;
    }

    if (pthread_mutex_init(&s->connLock, NULL) != 0) {
        fprintf(stderr, "conn mutex init failed - %d\n", errno);
        return NULL;
    }
    if (pthread_mutex_init(&s->rxLock, NULL) != 0) {
        fprintf(stderr, "RX mutex init failed - %d\n", errno);
        return NULL;
    }

#if defined(_WIN32) || defined(__MINGW32__)
    if (!tcpStart) {
        // WSAStartup()/WSACleanup() are refcounted per-process by Winsock,
        // but we never call WSACleanup() (see serial_tcp.h) so a single
        // one-time call here is sufficient for the life of the process.
        WSADATA wsaData;
        int err = WSAStartup(MAKEWORD(2, 2), &wsaData);
        if (err != 0) {
            fprintf(stderr, "WSAStartup failed (%d)\n", err);
            return NULL;
        }
    }
#endif

    tcpStart = true;
    tcpPortInitialized[id] = true;

    s->connected = false;
    s->id = id;
    s->clientSocket = TCP_INVALID_SOCKET;

    s->listenSocket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (s->listenSocket == TCP_INVALID_SOCKET) {
        fprintf(stderr, "socket() failed for UART%u\n", (unsigned)id + 1);
        return NULL;
    }

    int one = 1;
    setsockopt(s->listenSocket, SOL_SOCKET, SO_REUSEADDR, (const char*)&one, sizeof(one));

    struct sockaddr_in bindAddr;
    memset(&bindAddr, 0, sizeof(bindAddr));
    bindAddr.sin_family = AF_INET;
    bindAddr.sin_addr.s_addr = htonl(INADDR_ANY);
    bindAddr.sin_port = htons(BASE_PORT + id + 1);

    if (bind(s->listenSocket, (struct sockaddr*)&bindAddr, sizeof(bindAddr)) != 0) {
        fprintf(stderr, "bind port %u for UART%u failed!!\n", (unsigned)BASE_PORT + id + 1, (unsigned)id + 1);
        return NULL;
    }
    if (listen(s->listenSocket, 10) != 0) {
        fprintf(stderr, "listen port %u for UART%u failed!!\n", (unsigned)BASE_PORT + id + 1, (unsigned)id + 1);
        return NULL;
    }
    fprintf(stderr, "bind port %u for UART%u\n", (unsigned)BASE_PORT + id + 1, (unsigned)id + 1);

    if (pthread_create(&s->acceptThread, NULL, tcpAcceptThread, s) != 0) {
        fprintf(stderr, "Unable to create accept thread for UART%u\n", (unsigned)id + 1);
        return NULL;
    }

    return s;
}

serialPort_t *serTcpOpen(int id, serialReceiveCallbackPtr rxCallback, void *rxCallbackData, uint32_t baudRate, portMode_e mode, portOptions_e options)
{
    tcpPort_t *s = NULL;

#if defined(USE_UART1) || defined(USE_UART2) || defined(USE_UART3) || defined(USE_UART4) || defined(USE_UART5) || defined(USE_UART6) || defined(USE_UART7) || defined(USE_UART8)
    if (id >= 0 && id < SERIAL_PORT_COUNT) {
    s = tcpReconfigure(&tcpSerialPorts[id], id);
    }
#endif
    if (!s)
        return NULL;

    s->port.vTable = &tcpVTable;

    // common serial initialisation code should move to serialPort::init()
    s->port.rxBufferHead = s->port.rxBufferTail = 0;
    s->port.rxBufferSize = RX_BUFFER_SIZE;
    s->port.rxBuffer = s->rxBuffer;

    // callback works for IRQ-based RX ONLY
    s->port.rxCallback = rxCallback;
    s->port.rxCallbackData = rxCallbackData;
    s->port.mode = mode;
    s->port.baudRate = baudRate;
    s->port.options = options;

    return (serialPort_t *)s;
}

uint32_t tcpTotalRxBytesWaiting(const serialPort_t *instance)
{
    tcpPort_t *s = (tcpPort_t*)instance;
    uint32_t count;
    pthread_mutex_lock(&s->rxLock);
    if (s->port.rxBufferHead >= s->port.rxBufferTail) {
        count = s->port.rxBufferHead - s->port.rxBufferTail;
    } else {
        count = s->port.rxBufferSize + s->port.rxBufferHead - s->port.rxBufferTail;
    }
    pthread_mutex_unlock(&s->rxLock);

    return count;
}

// Writes are synchronous send() calls (see tcpWriteBuf()), so there is no TX
// queue to report on - always claim plenty of room / an empty buffer, as
// iNav's equivalent tcpTotalTxBytesFree()/isTcpTransmitBufferEmpty() do.
uint32_t tcpTotalTxBytesFree(const serialPort_t *instance)
{
    UNUSED(instance);
    return 0xFFFF;
}

bool isTcpTransmitBufferEmpty(const serialPort_t *instance)
{
    UNUSED(instance);
    return true;
}

uint8_t tcpRead(serialPort_t *instance)
{
    uint8_t ch;
    tcpPort_t *s = (tcpPort_t *)instance;
    pthread_mutex_lock(&s->rxLock);

    ch = s->port.rxBuffer[s->port.rxBufferTail];
    if (s->port.rxBufferTail + 1 >= s->port.rxBufferSize) {
        s->port.rxBufferTail = 0;
    } else {
        s->port.rxBufferTail++;
    }
    pthread_mutex_unlock(&s->rxLock);

    return ch;
}

// Sends are direct, blocking send() calls on the current client socket (if
// any) rather than going through a queue - matches iNav's tcpWritBuf(). The
// accept thread is the only other thread touching the socket and only ever
// reads from it, so no lock is needed around send() itself, only around the
// connLock-guarded fields identifying which socket (if any) is current.
void tcpWriteBuf(serialPort_t *instance, const void *data, int count)
{
    tcpPort_t *s = (tcpPort_t *)instance;
    if (count <= 0) {
        return;
    }

    pthread_mutex_lock(&s->connLock);
    if (s->connected) {
        send(s->clientSocket, (const char*)data, count, 0);
    }
    pthread_mutex_unlock(&s->connLock);
}

void tcpWrite(serialPort_t *instance, uint8_t ch)
{
    tcpWriteBuf(instance, &ch, 1);
}

void tcpDataIn(tcpPort_t *instance, uint8_t* ch, int size)
{
    tcpPort_t *s = (tcpPort_t *)instance;
    pthread_mutex_lock(&s->rxLock);

    while (size--) {
        s->port.rxBuffer[s->port.rxBufferHead] = *(ch++);
        if (s->port.rxBufferHead + 1 >= s->port.rxBufferSize) {
            s->port.rxBufferHead = 0;
        } else {
            s->port.rxBufferHead++;
        }
    }
    pthread_mutex_unlock(&s->rxLock);
}

static const struct serialPortVTable tcpVTable = {
        .serialWrite = tcpWrite,
        .serialTotalRxWaiting = tcpTotalRxBytesWaiting,
        .serialTotalTxFree = tcpTotalTxBytesFree,
        .serialRead = tcpRead,
        .serialSetBaudRate = NULL,
        .isSerialTransmitBufferEmpty = isTcpTransmitBufferEmpty,
        .setMode = NULL,
        .setCtrlLineStateCb = NULL,
        .setBaudRateCb = NULL,
        .writeBuf = tcpWriteBuf,
        .beginWrite = NULL,
        .endWrite = NULL,
};

