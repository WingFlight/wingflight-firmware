/*
 * XPLANE host runtime shims.
 *
 * These fill low-level MCU/runtime APIs that are normally provided by
 * hardware-specific drivers but are absent in the host SITL build.
 */

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <math.h>

#ifdef _WIN32
#include <windows.h>
#endif

#include "platform.h"
#include "common/time.h"
#include "drivers/time.h"
#include "drivers/system.h"
#include "drivers/adc.h"
#include "drivers/io.h"
#include "drivers/pwm_output.h"
#include "drivers/timer.h"
#include "drivers/dma_impl.h"
#include "drivers/accgyro/accgyro_fake.h"
#include "drivers/barometer/barometer_fake.h"
#include "drivers/compass/compass_fake.h"
#include "drivers/accgyro/accgyro_mpu.h"
#include "drivers/fbus_sensor.h"
#include "sensors/adcinternal.h"
#include "flight/mixer.h"
#include "flight/motors.h"
#include "flight/servos.h"
#include "io/gps.h"
#include "pg/sbus_output.h"
#include "pg/fbus_master.h"
#include "pg/serial_pinconfig.h"
#include "dyad.h"
#include "target/XPLANE/xplane_link.h"
#include "target/XPLANE/protocol_xplane.h"

char _estack;
char _Min_Stack_Size;

dmaChannelDescriptor_t dmaDescriptors[1];
sbusOutConfig_t sbusOutConfig_System;
fbusMasterConfig_t fbusMasterConfig_System;

static void host_sleep_us(uint32_t us)
{
#ifdef _WIN32
    if (us == 0) {
        return;
    }
    Sleep((DWORD)((us + 999U) / 1000U));
#else
    struct timespec ts;
    ts.tv_sec = us / 1000000U;
    ts.tv_nsec = (long)(us % 1000000U) * 1000L;
    nanosleep(&ts, NULL);
#endif
}

static xplane_link_t xplaneLink;
static xplane_pwm_packet_t xplanePwm;
static bool xplaneReady = false;
static bool dyadReady = false;
static uint32_t xplaneLastTxMs = 0;
static uint32_t xplaneLastStatsMs = 0;
static uint32_t xplaneRxCount = 0;
static uint32_t xplaneTxCount = 0;

#define XPLANE_ACC_SCALE  (256.0f / 9.80665f)
#define XPLANE_GYRO_SCALE 16.4f

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

static void xplaneInjectFdm(const xplane_fdm_packet_t *fdm)
{
    if (!fdm) {
        return;
    }

    // During early startup, FDM can arrive before fake sensor devices are ready.
    // Skip this frame instead of dereferencing null device handles.
    if (!fakeAccDev || !fakeGyroDev) {
        return;
    }

    const int16_t accX = (int16_t)clamp_i32((int32_t)lrintf(-fdm->accel_x * XPLANE_ACC_SCALE), -32767, 32767);
    const int16_t accY = (int16_t)clamp_i32((int32_t)lrintf(-fdm->accel_y * XPLANE_ACC_SCALE), -32767, 32767);
    const int16_t accZ = (int16_t)clamp_i32((int32_t)lrintf(-fdm->accel_z * XPLANE_ACC_SCALE), -32767, 32767);
    fakeAccSet(fakeAccDev, accX, accY, accZ);

    const float radToDeg = 180.0f / 3.14159265359f;
    const int16_t gyroX = (int16_t)clamp_i32((int32_t)lrintf(fdm->p * XPLANE_GYRO_SCALE * radToDeg), -32767, 32767);
    const int16_t gyroY = (int16_t)clamp_i32((int32_t)lrintf(-fdm->q * XPLANE_GYRO_SCALE * radToDeg), -32767, 32767);
    const int16_t gyroZ = (int16_t)clamp_i32((int32_t)lrintf(-fdm->r * XPLANE_GYRO_SCALE * radToDeg), -32767, 32767);
    fakeGyroSet(fakeGyroDev, gyroX, gyroY, gyroZ);

    const int32_t pressurePa = clamp_i32((int32_t)lrintf(fdm->pressure_pa), 10000, 120000);
    fakeBaroSet(pressurePa, 2500);

    const int16_t magX = (int16_t)clamp_i32((int32_t)lrintf(fdm->mag_x), -32767, 32767);
    const int16_t magY = (int16_t)clamp_i32((int32_t)lrintf(fdm->mag_y), -32767, 32767);
    const int16_t magZ = (int16_t)clamp_i32((int32_t)lrintf(fdm->mag_z), -32767, 32767);
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
        float courseDeg = atan2f(fdm->velocity_e, fdm->velocity_n) * 57.2957795f;
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

static void xplaneFillPwmPacket(xplane_pwm_packet_t *pkt)
{
    if (!pkt) {
        return;
    }

    for (int i = 0; i < XPLANE_PWM_CHANNELS; i++) {
        pkt->pwm_values[i] = 1500;
    }

    const uint8_t motorCount = getMotorCount();
    const uint8_t servoCount = getServoCount();

    if (motorCount > 0U) {
        pkt->pwm_values[0] = throttle_to_pwm(mixerGetMotorOutput(0));
    }
    if (servoCount > 0U) { pkt->pwm_values[1] = normalized_to_pwm(mixerGetServoOutput(0)); }
    if (servoCount > 1U) { pkt->pwm_values[2] = normalized_to_pwm(mixerGetServoOutput(1)); }
    if (servoCount > 2U) { pkt->pwm_values[3] = normalized_to_pwm(mixerGetServoOutput(2)); }
    if (servoCount > 3U) { pkt->pwm_values[4] = normalized_to_pwm(mixerGetServoOutput(3)); }
    if (servoCount > 4U) { pkt->pwm_values[5] = normalized_to_pwm(mixerGetServoOutput(4)); }

    for (int channel = 6; channel < XPLANE_PWM_CHANNELS; channel++) {
        const uint8_t servoIndex = (uint8_t)(channel - 1);
        if (servoIndex < servoCount) {
            pkt->pwm_values[channel] = normalized_to_pwm(mixerGetServoOutput(servoIndex));
        }
    }
}

static uint32_t host_now_ms(void)
{
#ifdef _WIN32
    return (uint32_t)GetTickCount64();
#else
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint32_t)(ts.tv_sec * 1000ULL + ts.tv_nsec / 1000000ULL);
#endif
}

static void xplaneService(void)
{
    if (!xplaneReady) {
        return;
    }

    uint8_t rxBuffer[256];
    const int rx = xplane_link_recv_fdm(&xplaneLink, rxBuffer, sizeof(rxBuffer), 0);
    if (rx == (int)sizeof(xplane_fdm_packet_t)) {
        const xplane_fdm_packet_t *fdm = (const xplane_fdm_packet_t *)rxBuffer;
        if (xplane_validate_fdm_packet(fdm)) {
            xplaneRxCount++;
            xplaneInjectFdm(fdm);
        }
    }

    const uint32_t nowMs = host_now_ms();
    if ((uint32_t)(nowMs - xplaneLastTxMs) >= 50U) {
        xplanePwm.timestamp_ms = nowMs;
        xplaneFillPwmPacket(&xplanePwm);
        xplane_update_packet_crc(&xplanePwm, sizeof(xplanePwm));
        if (xplane_link_send_pwm(&xplaneLink, &xplanePwm, sizeof(xplanePwm)) > 0) {
            xplaneTxCount++;
        }
        xplaneLastTxMs = nowMs;
    }

    if ((uint32_t)(nowMs - xplaneLastStatsMs) >= 5000U) {
        fprintf(stderr, "[XPLANE] UDP RX=%u TX=%u Connected=%d\n", (unsigned)xplaneRxCount, (unsigned)xplaneTxCount, xplane_link_is_connected(&xplaneLink));
        xplaneLastStatsMs = nowMs;
    }
}

timeUs_t micros(void)
{
#ifdef _WIN32
    static ULONGLONG start = 0;
    ULONGLONG now = GetTickCount64();
    if (start == 0) {
        start = now;
    }
    xplaneService();
    return (timeUs_t)((now - start) * 1000ULL);
#else
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    xplaneService();
    return (timeUs_t)(ts.tv_sec * 1000000ULL + ts.tv_nsec / 1000ULL);
#endif
}

timeUs_t microsISR(void)
{
    return micros();
}

void delay(timeMs_t ms)
{
    host_sleep_us((uint32_t)ms * 1000U);
}

void systemReset(int reason)
{
    UNUSED(reason);
}

void systemInit(void)
{
    if (!dyadReady) {
        dyad_init();
        dyadReady = true;
    }

    xplane_link_set_debug(0);
    xplane_init_pwm_packet(&xplanePwm);
    if (xplane_link_init(&xplaneLink, 5502, 5503, "127.0.0.1") == 0) {
        xplaneReady = true;
        fprintf(stderr, "[XPLANE] UDP initialized: listen 5502, send 5503\n");
    } else {
        xplaneReady = false;
        fprintf(stderr, "[XPLANE] UDP initialization failed\n");
    }
}

void systemResetHard(void)
{
    // Host-mode reset requests can happen during config bootstrap.
    // Keep running instead of tearing down sockets and exiting.
}

void indicateFailure(failureMode_e mode, int repeatCount)
{
    UNUSED(mode);
    UNUSED(repeatCount);
}

void unusedPinsInit(void)
{
}

void timerInit(void)
{
}

void timerStart(void)
{
}

void failureMode(failureMode_e mode)
{
    UNUSED(mode);
}

void IOConfigGPIO(IO_t io, ioConfig_t cfg)
{
    UNUSED(io);
    UNUSED(cfg);
}

void IOConfigGPIOAF(IO_t io, ioConfig_t cfg, uint8_t af)
{
    UNUSED(io);
    UNUSED(cfg);
    UNUSED(af);
}

uint32_t timerClock(TIM_TypeDef *tim)
{
    UNUSED(tim);
    return 1000000U;
}

void pwmOutConfig(timerChannel_t *channel, const timerHardware_t *timerHardware, uint32_t hz, uint16_t period, uint16_t value, uint8_t inversion)
{
    UNUSED(timerHardware);
    UNUSED(hz);
    UNUSED(inversion);
    if (channel) {
        channel->tim = NULL;
        channel->ccr = NULL;
    }
    UNUSED(period);
    UNUSED(value);
}

int16_t getCoreTemperatureCelsius(void)
{
    return 25;
}

bool adcIsEnabled(uint8_t channel)
{
    UNUSED(channel);
    return false;
}

uint8_t mpuGyroReadRegister(const extDevice_t *dev, uint8_t reg)
{
    UNUSED(dev);
    UNUSED(reg);
    return 0;
}

static bool shimMotorEnable(void) { return true; }
static void shimMotorDisable(void) { }
static void shimMotorShutdown(void) { }
static bool shimMotorUpdateStart(void) { return true; }
static void shimMotorUpdateComplete(void) { }
static void shimMotorWrite(uint8_t index, uint8_t mode, float value) { UNUSED(index); UNUSED(mode); UNUSED(value); }
static void shimMotorWriteInt(uint8_t index, uint16_t value) { UNUSED(index); UNUSED(value); }
static bool shimIsMotorEnabled(uint8_t index) { UNUSED(index); return true; }

motorDevice_t *motorPwmDevInit(const struct motorDevConfig_s *motorDevConfig, uint8_t motorCount)
{
    UNUSED(motorDevConfig);
    static motorDevice_t shimMotorDevice;
    shimMotorDevice.count = motorCount;
    shimMotorDevice.initialized = true;
    shimMotorDevice.enabled = true;
    shimMotorDevice.vTable.postInit = motorPostInitNull;
    shimMotorDevice.vTable.enable = shimMotorEnable;
    shimMotorDevice.vTable.disable = shimMotorDisable;
    shimMotorDevice.vTable.shutdown = shimMotorShutdown;
    shimMotorDevice.vTable.updateStart = shimMotorUpdateStart;
    shimMotorDevice.vTable.updateComplete = shimMotorUpdateComplete;
    shimMotorDevice.vTable.write = shimMotorWrite;
    shimMotorDevice.vTable.writeInt = shimMotorWriteInt;
    shimMotorDevice.vTable.isMotorEnabled = shimIsMotorEnabled;
    return &shimMotorDevice;
}

int32_t clockCyclesTo10thMicros(int32_t clockCycles)
{
    return clockCycles * 10;
}

uint32_t clockMicrosToCycles(uint32_t inMicros)
{
    return inMicros;
}

uint32_t getCycleCounter(void)
{
    return (uint32_t)micros();
}

void fbusSensorUpdate(timeUs_t currentTimeUs)
{
    UNUSED(currentTimeUs);
}

bool fbusSensorProcessData(uint8_t physicalId, uint16_t appId, uint32_t data)
{
    UNUSED(physicalId);
    UNUSED(appId);
    UNUSED(data);
    return false;
}

void pgResetFn_serialPinConfig(serialPinConfig_t *serialPinConfig)
{
    UNUSED(serialPinConfig);
}
