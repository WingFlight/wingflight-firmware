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
 * X-Plane 11/12 Hardware-in-Loop Target Implementation
 *
 * This file provides the target-specific initialization and packet processing
 * for Wingflight SITL connected to X-Plane flight simulator.
 */

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <errno.h>
#include <time.h>

#ifdef _WIN32
    #include <winsock2.h>
    #include <windows.h>
#endif

#include "common/maths.h"
#include "common/utils.h"

#include "drivers/io.h"
#include "drivers/dma.h"
#include "drivers/motor.h"
#include "drivers/serial.h"
#include "drivers/serial_tcp.h"
#include "drivers/system.h"
#include "drivers/pwm_output.h"
#include "drivers/light_led.h"
#include "dyad.h"

#include "drivers/timer.h"
#include "drivers/timer_def.h"
#include "flight/motors.h"
#include "flight/servos.h"

#include "drivers/accgyro/accgyro_fake.h"
#include "drivers/barometer/barometer_fake.h"
#include "drivers/compass/compass_fake.h"
#include "flight/imu.h"
#include "flight/mixer.h"

#include "config/feature.h"
#include "config/config.h"
#include "scheduler/scheduler.h"

#include "io/gps.h"

#include "pg/rx.h"
#include "pg/motor.h"

#include "rx/rx.h"

/* Undef conflicting macros from rx.h before protocol_xplane.h is parsed */
#undef PWM_RANGE

#include "target/XPLANE/xplane_link.h"
#include "target/XPLANE/protocol_xplane.h"

/* ============================================================================
 * FORWARD DECLARATIONS
 * ============================================================================ */

uint32_t millis(void);  // Implemented at end of file

/* ============================================================================
 * SYSTEM STATE
 * ============================================================================ */

uint32_t SystemCoreClock = 168000000;  // Dummy value for SITL

// Timer hardware (unused in SITL)
const timerHardware_t timerHardware[1] = {0};

/* ============================================================================
 * X-PLANE LINK STATE
 * ============================================================================ */

static xplane_link_t xplane_link;
static xplane_fdm_packet_t current_fdm;
static xplane_pwm_packet_t current_pwm;

// Connection statistics
static uint32_t packet_count = 0;
static uint32_t error_count = 0;
static uint32_t last_status_print = 0;

// Fake EEPROM backend for config streamer
extern uint8_t eepromData[EEPROM_SIZE];
static FILE *eepromFd = NULL;

// Configuration
#define XPLANE_FDM_LISTEN_PORT  5502   // Port to listen for FDM data
#define XPLANE_PWM_SEND_PORT    5503   // Port to send PWM commands
#define XPLANE_REMOTE_ADDR      "127.0.0.1"  // Localhost
#define XPLANE_CONNECTION_TIMEOUT_MS 5000

/* ============================================================================
 * CONVERSION CONSTANTS
 * ============================================================================ */

/* Use definitions from protocol_xplane.h for RAD_TO_DEG and DEG_TO_RAD */

// Sensor scaling factors
#define ACC_SCALE   (256.0f / 9.80665f)  // Convert m/s² to ADC units
#define GYRO_SCALE  16.4f                 // Convert rad/s to ADC units
#define MAG_SCALE   1.0f                  // Magnetic field units

static inline float clampf_local(float value, float minValue, float maxValue)
{
    return value < minValue ? minValue : (value > maxValue ? maxValue : value);
}

static inline int32_t clamp_i32(int32_t value, int32_t minValue, int32_t maxValue)
{
    return value < minValue ? minValue : (value > maxValue ? maxValue : value);
}

static uint16_t normalized_to_pwm(float normalized)
{
    const float clamped = clampf_local(normalized, -1.0f, 1.0f);
    return (uint16_t)(1500.0f + clamped * 500.0f);
}

static uint16_t throttle_to_pwm(float throttle)
{
    const float clamped = clampf_local(throttle, 0.0f, 1.0f);
    return (uint16_t)(1000.0f + clamped * 1000.0f);
}

/* ============================================================================
 * INITIALIZATION
 * ============================================================================ */

/**
 * Initialize X-Plane HITL communication
 */
static void xplane_init(void) {
    int result;
    
    printf("[XPLANE] Initializing X-Plane HITL link...\n");
    printf("[XPLANE] Listening on port %d, sending to %s:%d\n",
           XPLANE_FDM_LISTEN_PORT, XPLANE_REMOTE_ADDR, XPLANE_PWM_SEND_PORT);
    
    result = xplane_link_init(&xplane_link, XPLANE_FDM_LISTEN_PORT, 
                              XPLANE_PWM_SEND_PORT, XPLANE_REMOTE_ADDR);
    
    if (result != 0) {
        printf("[XPLANE] ERROR: Failed to initialize link\n");
        exit(1);
    }
    
    // Initialize packet structures
    xplane_init_fdm_packet(&current_fdm);
    xplane_init_pwm_packet(&current_pwm);

    // Initialize dyad for TCP-based serial ports (Configurator on UART1)
    dyad_init();
    
    printf("[XPLANE] Initialization complete. Waiting for X-Plane connection...\n\n");
}

/**
 * Cleanup X-Plane HITL communication
 */
static void xplane_cleanup(void) {
    printf("\n[XPLANE] Closing connection...\n");
    xplane_link_close(&xplane_link);
    dyad_shutdown();
    printf("[XPLANE] Cleanup complete\n");
}

/* ============================================================================
 * PACKET PROCESSING
 * ============================================================================ */

/**
 * Receive and process FDM packet from X-Plane
 * Updates fake sensor drivers with X-Plane flight data
 */
static void xplane_process_fdm_packet(void) {
    uint8_t buffer[256];
    int bytes_received;
    
    // Receive packet (non-blocking)
    bytes_received = xplane_link_recv_fdm(&xplane_link, buffer, sizeof(buffer), 0);
    
    if (bytes_received <= 0) {
        return;  // No packet received
    }
    
    if (bytes_received != (int)XPLANE_FDM_PACKET_SIZE) {
        error_count++;
        return;
    }
    
    // Cast to FDM packet
    xplane_fdm_packet_t* fdm = (xplane_fdm_packet_t*)buffer;
    if (!xplane_validate_fdm_packet(fdm)) {
        error_count++;
        return;
    }

    memcpy(&current_fdm, fdm, sizeof(xplane_fdm_packet_t));
    packet_count++;

    if (!fakeAccDev || !fakeGyroDev) {
        return;
    }

    // Inject IMU into fake sensors (matching legacy SITL axis/sign conventions)
    const int16_t accX = (int16_t)clamp_i32((int32_t)lrintf(-fdm->accel_x * ACC_SCALE), -32767, 32767);
    const int16_t accY = (int16_t)clamp_i32((int32_t)lrintf(-fdm->accel_y * ACC_SCALE), -32767, 32767);
    const int16_t accZ = (int16_t)clamp_i32((int32_t)lrintf(-fdm->accel_z * ACC_SCALE), -32767, 32767);
    fakeAccSet(fakeAccDev, accX, accY, accZ);

    const int16_t gyroX = (int16_t)clamp_i32((int32_t)lrintf(fdm->p * GYRO_SCALE * RAD_TO_DEG), -32767, 32767);
    const int16_t gyroY = (int16_t)clamp_i32((int32_t)lrintf(-fdm->q * GYRO_SCALE * RAD_TO_DEG), -32767, 32767);
    const int16_t gyroZ = (int16_t)clamp_i32((int32_t)lrintf(-fdm->r * GYRO_SCALE * RAD_TO_DEG), -32767, 32767);
    fakeGyroSet(fakeGyroDev, gyroX, gyroY, gyroZ);

    const int32_t pressurePa = clamp_i32((int32_t)lrintf(fdm->pressure_pa), 10000, 120000);
    fakeBaroSet(pressurePa, 2500);

    const int16_t magX = (int16_t)clamp_i32((int32_t)lrintf(fdm->mag_x * MAG_SCALE), -32767, 32767);
    const int16_t magY = (int16_t)clamp_i32((int32_t)lrintf(fdm->mag_y * MAG_SCALE), -32767, 32767);
    const int16_t magZ = (int16_t)clamp_i32((int32_t)lrintf(fdm->mag_z * MAG_SCALE), -32767, 32767);
    fakeMagSet(magX, magY, magZ);

    if (fdm->gps_fix_type > 0) {
        gpsSetFixState(true);
        gpsSol.llh.lat = (int32_t)lrint(fdm->latitude * 1e7);
        gpsSol.llh.lon = (int32_t)lrint(fdm->longitude * 1e7);
        gpsSol.llh.altCm = (int32_t)lrintf(fdm->altitude_msl * 100.0f);
        gpsSol.numSat = fdm->num_satellites;

        const float horizontalMs = sqrtf(fdm->velocity_n * fdm->velocity_n + fdm->velocity_e * fdm->velocity_e);
        const float speed3dMs = sqrtf(horizontalMs * horizontalMs + fdm->velocity_d * fdm->velocity_d);
        gpsSol.groundSpeed = (uint16_t)clamp_i32((int32_t)lrintf(horizontalMs * 10.0f), 0, 65535);
        gpsSol.speed3d = (uint16_t)clamp_i32((int32_t)lrintf(speed3dMs * 10.0f), 0, 65535);
        float courseDeg = atan2f(fdm->velocity_e, fdm->velocity_n) * RAD_TO_DEG;
        if (courseDeg < 0.0f) {
            courseDeg += 360.0f;
        }
        gpsSol.groundCourse = (uint16_t)clamp_i32((int32_t)lrintf(courseDeg * 10.0f), 0, 3600);
        gpsSol.hdop = 80;
        GPS_update |= GPS_MSP_UPDATE;
        onGpsNewData();
    } else {
        gpsSetFixState(false);
    }
}

/**
 * Send PWM commands to X-Plane
 * Takes motor mixer output and converts to X-Plane PWM format
 */
static void xplane_send_pwm_packet(void) {
    static uint32_t send_counter = 0;
    
    // Update timestamp
    current_pwm.timestamp_ms = send_counter * 50;  // 50ms frames at 20 Hz
    send_counter++;
    
    for (int i = 0; i < XPLANE_PWM_CHANNELS; i++) {
        current_pwm.pwm_values[i] = 1500;  // Center position
    }

    // Fixed-wing mapping (guarded by configured motor/servo counts)
    const uint8_t motorCount = getMotorCount();
    const uint8_t servoCount = getServoCount();

    if (motorCount > 0U) { current_pwm.pwm_values[0] = throttle_to_pwm(mixerGetMotorOutput(0)); }
    if (servoCount > 0U) { current_pwm.pwm_values[1] = normalized_to_pwm(mixerGetServoOutput(0)); }
    if (servoCount > 1U) { current_pwm.pwm_values[2] = normalized_to_pwm(mixerGetServoOutput(1)); }
    if (servoCount > 2U) { current_pwm.pwm_values[3] = normalized_to_pwm(mixerGetServoOutput(2)); }
    if (servoCount > 3U) { current_pwm.pwm_values[4] = normalized_to_pwm(mixerGetServoOutput(3)); }
    if (servoCount > 4U) { current_pwm.pwm_values[5] = normalized_to_pwm(mixerGetServoOutput(4)); }

    for (int channel = 6; channel < XPLANE_PWM_CHANNELS; channel++) {
        const uint8_t servoIndex = (uint8_t)(channel - 1);
        if (servoIndex < servoCount) {
            current_pwm.pwm_values[channel] = normalized_to_pwm(mixerGetServoOutput(servoIndex));
        }
    }

    xplane_update_packet_crc(&current_pwm, sizeof(current_pwm));
    
    // Send to X-Plane
    int result = xplane_link_send_pwm(&xplane_link, &current_pwm, 
                                      sizeof(xplane_pwm_packet_t));
    
    if (result < 0) {
        error_count++;
    }
}

/* ============================================================================
 * MAIN LOOP & THREADING
 * ============================================================================ */

/**
 * Main simulation loop update function
 * Called by Wingflight scheduler at flight loop rate (20 Hz for X-Plane)
 */
void xplane_update(void) {
    // Check connection health
    xplane_link_update_status(&xplane_link, XPLANE_CONNECTION_TIMEOUT_MS);
    
    // Receive and process FDM packet from X-Plane
    xplane_process_fdm_packet();
    
    // Send PWM commands to X-Plane
    xplane_send_pwm_packet();
    
    // Print status periodically (every 5 seconds)
    uint32_t current_time = millis();
    if (current_time - last_status_print > 5000) {
        uint32_t packets, errors;
        uint32_t ms_since = xplane_link_get_stats(&xplane_link, &packets, &errors);
        
        printf("[XPLANE] Status: %s, Packets: %u, Errors: %u, Last packet: %ums ago\n",
               xplane_link_is_connected(&xplane_link) ? "CONNECTED" : "DISCONNECTED",
               packets, errors, ms_since);
        
        last_status_print = current_time;
    }
}

/**
 * Lock main PID loop (thread synchronization)
 * Returns 1 if lock acquired, 0 otherwise
 */
int lockMainPID(void) {
    // SITL doesn't use actual PID locking, return success
    return 1;
}

/* ============================================================================
 * SYSTEM INITIALIZATION (Lifecycle)
 * ============================================================================ */

/**
 * Initialize target-specific systems
 * Called during system startup
 */
void targetInit(void) {
    printf("[XPLANE] Wingflight X-Plane HITL Target\n");
    printf("[XPLANE] Compiled: %s %s\n", __DATE__, __TIME__);
    
    xplane_init();
}

/**
 * Cleanup target systems
 * Called during system shutdown
 */
void targetCleanup(void) {
    xplane_cleanup();
}

/* ============================================================================
 * DIAGNOSTICS & DEBUG
 * ============================================================================ */

/**
 * Print connection diagnostics
 */
void xplane_print_diagnostics(void) {
    xplane_link_print_status(&xplane_link);
    printf("Total packets processed: %u\n", packet_count);
    printf("Total errors: %u\n", error_count);
}

/**
 * Reset statistics
 */
void xplane_reset_stats(void) {
    packet_count = 0;
    error_count = 0;
}

/* ============================================================================
 * TIMING & DELAY FUNCTIONS
 * ============================================================================ */

/**
 * High-resolution sleep function for microseconds (cross-platform)
 */
static void microsleep(uint32_t usec) {
#ifdef _WIN32
    HANDLE timer = CreateWaitableTimer(NULL, TRUE, NULL);
    if (timer == NULL) return;
    
    LARGE_INTEGER due;
    due.QuadPart = -(int64_t)usec * 10;  // Convert to 100-nanosecond units
    
    SetWaitableTimer(timer, &due, 0, NULL, NULL, 0);
    WaitForSingleObject(timer, INFINITE);
    CloseHandle(timer);
#else
    struct timespec ts, rem;
    ts.tv_sec = usec / 1000000UL;
    ts.tv_nsec = (usec % 1000000UL) * 1000UL;
    while (nanosleep(&ts, &rem) == -1 && errno == EINTR) {
        ts = rem;
    }
#endif
}

/**
 * Delay in microseconds (simulator rate adjustment)
 */
void delayMicroseconds(uint32_t us) {
    microsleep(us);  // No rate adjustment for X-Plane HITL
}

/**
 * Real-time delay in microseconds
 */
void delayMicroseconds_real(uint32_t us) {
    microsleep(us);
}

/**
 * Get system uptime in milliseconds
 */
uint32_t millis(void) {
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

void FLASH_Unlock(void)
{
    if (eepromFd != NULL) {
        return;
    }

    eepromFd = fopen(EEPROM_FILENAME, "r+");
    if (eepromFd != NULL) {
        fseek(eepromFd, 0, SEEK_END);
        size_t lSize = ftell(eepromFd);
        rewind(eepromFd);
        size_t n = fread(eepromData, 1, sizeof(eepromData), eepromFd);
        if (n != lSize) {
            fclose(eepromFd);
            eepromFd = NULL;
        }
    } else {
        eepromFd = fopen(EEPROM_FILENAME, "w+");
        if (eepromFd != NULL) {
            fwrite(eepromData, sizeof(eepromData), 1, eepromFd);
        }
    }
}

void FLASH_Lock(void)
{
    if (eepromFd != NULL) {
        fseek(eepromFd, 0, SEEK_SET);
        fwrite(eepromData, 1, sizeof(eepromData), eepromFd);
        fclose(eepromFd);
        eepromFd = NULL;
    }
}

FLASH_Status FLASH_ErasePage(uintptr_t Page_Address)
{
    UNUSED(Page_Address);
    return FLASH_COMPLETE;
}

FLASH_Status FLASH_ProgramWord(uintptr_t addr, uint32_t Data)
{
    if ((addr >= (uintptr_t)eepromData) && (addr < (uintptr_t)ARRAYEND(eepromData))) {
        *((uint32_t *)addr) = Data;
    }
    return FLASH_COMPLETE;
}
