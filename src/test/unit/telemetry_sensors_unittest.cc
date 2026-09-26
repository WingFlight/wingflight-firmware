#include <algorithm>
#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <set>
#include <vector>

#include "gtest/gtest.h"

extern "C" {
#include "platform.h"
#include "drivers/serial.h"
#include "drivers/time.h"
#include "fc/rc_modes.h"
#include "fc/runtime_config.h"
#include "config/feature.h"
#include "flight/imu.h"
#include "flight/mixer.h"
#include "flight/position.h"
#include "io/gps.h"
#include "rx/crsf.h"
#include "rx/crsf_protocol.h"
#include "sensors/acceleration.h"
#include "sensors/battery.h"
#include "common/crc.h"
#include "common/printf.h"
#include "telemetry/telemetry.h"
#include "telemetry/smartport.h"
#include "telemetry/crsf.h"
#include "telemetry/frsky_hub.h"
#include "telemetry/hott.h"
#include "telemetry/ibus.h"
#include "telemetry/jetiexbus.h"
#include "telemetry/ltm.h"
#include "telemetry/mavlink.h"

telemetryConfig_t telemetryConfig_System;
void initSmartPortSensors(void);

uint8_t armingFlags;
gpsSolutionData_t gpsSol;
static sensor_id_e inactiveSensor;
static int sensorValue;
static bool crsfActive = true;
static std::vector<std::vector<uint8_t>> crsfFrames;
static timeUs_t crsfTestTime;
uint16_t flightModeFlags;
uint8_t stateFlags;
attitudeEulerAngles_t attitude;
acc_t acc;

bool telemetrySensorActive(sensor_id_e id) { return id != TELEM_NONE && id != inactiveSensor; }
int telemetrySensorValue(sensor_id_e) { return sensorValue; }
const char *getAdjustmentsRangeName(void) { return "Test"; }
int getAdjustmentsRangeFunc(void) { return 12; }
int getAdjustmentsRangeValue(void) { return -34; }
void legacySensorInit(void) {}
bool isModeActivationConditionPresent(boxId_e) { return false; }
bool IS_RC_MODE_ACTIVE(boxId_e) { return false; }
timeUs_t micros(void) { return 0; }
void serialWrite(serialPort_t *, uint8_t) {}
uint32_t serialRxBytesWaiting(const serialPort_t *) { return 0; }
uint8_t serialRead(serialPort_t *) { return 0; }
const serialPortConfig_t *findSerialPortConfig(serialPortFunction_e) { return nullptr; }
portSharing_e determinePortSharing(const serialPortConfig_t *, serialPortFunction_e) { return PORTSHARING_NOT_SHARED; }
serialPort_t *openSerialPort(serialPortIdentifier_e, serialPortFunction_e, serialReceiveCallbackPtr,
    void *, uint32_t, portMode_e, portOptions_e) { return nullptr; }
void closeSerialPort(serialPort_t *) {}
bool crsfRxIsActive(void) { return crsfActive; }
void crsfRxTransmitTelemetryData(const void *data, int len)
{
    const auto bytes = static_cast<const uint8_t *>(data);
    crsfFrames.emplace_back(bytes, bytes + len);
}
bool featureIsEnabled(uint32_t) { return false; }
bool isArmingDisabled(void) { return false; }
armingDisableFlags_e getArmingDisableFlags(void) { return armingDisableFlags_e(0); }
uint16_t getLegacyBatteryVoltage(void) { return 120; }
uint16_t getLegacyBatteryCurrent(void) { return 10; }
uint32_t getBatteryCapacityUsed(void) { return 100; }
uint8_t getBatteryChargeLevel(void) { return 90; }
uint8_t getBatteryCellCount(void) { return 3; }
uint16_t getBatteryCellVoltage(uint8_t) { return 400; }
int getEstimatedAltitudeCm(void) { return 100; }
int getEstimatedVarioCms(void) { return 0; }
float mixerGetInput(uint8_t) { return 0; }
int tfp_sprintf(char *dest, const char *format, ...)
{
    va_list args;
    va_start(args, format);
    const int result = vsprintf(dest, format, args);
    va_end(args);
    return result;
}

// The unit-test target also enables these unrelated protocols.
void handleFrSkyHubTelemetry(timeUs_t) {}
void checkFrSkyHubTelemetryState(void) {}
bool initFrSkyHubTelemetry(void) { return false; }
void handleHoTTTelemetry(timeUs_t) {}
void checkHoTTTelemetryState(void) {}
void initHoTTTelemetry(void) {}
void handleLtmTelemetry(void) {}
void checkLtmTelemetryState(void) {}
void initLtmTelemetry(void) {}
void handleJetiExBusTelemetry(void) {}
void checkJetiExBusTelemetryState(void) {}
void initJetiExBusTelemetry(void) {}
void handleMAVLinkTelemetry(void) {}
void checkMAVLinkTelemetryState(void) {}
void initMAVLinkTelemetry(void) {}
void handleIbusTelemetry(void) {}
bool checkIbusTelemetryState(void) { return false; }
void initIbusTelemetry(void) {}
}

class TelemetrySensorsTest : public ::testing::Test {
protected:
    void SetUp() override
    {
        memset(telemetryConfigMutable(), 0, sizeof(telemetryConfig_t));
        memset(&gpsSol, 0, sizeof(gpsSol));
        inactiveSensor = TELEM_NONE;
        sensorValue = 12340;
    }

    // Drain one scheduling round using the production scheduler. Committing makes
    // each sensor ineligible until the next update, so duplicates cannot hide.
    std::vector<telemetrySensor_t *> scheduledSensors()
    {
        std::vector<telemetrySensor_t *> sensors;
        while (telemetrySensor_t *sensor = telemetryScheduleNext()) {
            sensors.push_back(sensor);
            telemetryScheduleCommit(sensor);
            if (sensors.size() > TELEM_SENSOR_SLOT_COUNT + 2) {
                ADD_FAILURE() << "Too many scheduled sensors";
                break;
            }
        }
        std::sort(sensors.begin(), sensors.end(), [](auto a, auto b) { return a->index < b->index; });
        return sensors;
    }
};

using SmartPortSensorsTest = TelemetrySensorsTest;

TEST_F(SmartPortSensorsTest, EmptyUnsupportedAndInactiveSelections)
{
    initSmartPortSensors();
    EXPECT_EQ(nullptr, telemetryScheduleNext());
    telemetryScheduleUpdate(1000);
    EXPECT_EQ(nullptr, telemetryScheduleNext());

    telemetryConfigMutable()->telemetry_sensors[0] = UINT16_MAX;
    telemetryConfigMutable()->telemetry_sensors[1] = TELEM_BATTERY_VOLTAGE;
    inactiveSensor = TELEM_BATTERY_VOLTAGE;
    initSmartPortSensors();
    EXPECT_EQ(nullptr, telemetryScheduleNext());
}

TEST_F(SmartPortSensorsTest, ScalingOverridesAndReinitialization)
{
    telemetryConfigMutable()->telemetry_sensors[0] = TELEM_BATTERY_CURRENT;
    telemetryConfigMutable()->telemetry_interval[0] = 250;
    initSmartPortSensors();
    telemetryScheduleUpdate(1000);
    auto sensors = scheduledSensors();
    ASSERT_EQ(1u, sensors.size());
    auto sensor = sensors[0];
    EXPECT_EQ(0x0200, sensor->app_id);
    EXPECT_EQ(250, sensor->fast_interval);
    EXPECT_GE(sensor->slow_interval, 3000);
    EXPECT_LT(sensor->slow_interval, 3100);
    EXPECT_EQ(1234, sensor->value);
    smartPortPayload_t payload = {};
    sensor->encode(sensor, &payload);
    EXPECT_EQ(1234u, payload.data);

    telemetryConfigMutable()->telemetry_interval[0] = 0;
    initSmartPortSensors();
    sensors = scheduledSensors();
    ASSERT_EQ(1u, sensors.size());
    EXPECT_EQ(100, sensors[0]->fast_interval);
    EXPECT_EQ(0, sensors[0]->value);
    EXPECT_LT(sensors[0]->slow_interval, 3100);
}

TEST_F(SmartPortSensorsTest, DuplicateSelectionsShareOneRuntimeEntry)
{
    for (unsigned i = 0; i < TELEM_SENSOR_SLOT_COUNT; i++) {
        telemetryConfigMutable()->telemetry_sensors[i] = TELEM_BATTERY_VOLTAGE;
        telemetryConfigMutable()->telemetry_interval[i] = 100 + i;
    }
    initSmartPortSensors();
    auto sensors = scheduledSensors();
    ASSERT_EQ(1u, sensors.size());
    EXPECT_EQ(139, sensors[0]->fast_interval);
}

TEST_F(SmartPortSensorsTest, MultipartSensorsKeepBothEncoders)
{
    telemetryConfigMutable()->telemetry_sensors[0] = TELEM_GPS_COORD;
    telemetryConfigMutable()->telemetry_sensors[1] = TELEM_ADJFUNC;
    gpsSol.llh.lat = 10000000;
    gpsSol.llh.lon = -20000000;
    initSmartPortSensors();
    auto sensors = scheduledSensors();
    ASSERT_EQ(4u, sensors.size());
    const uint16_t ids[] = {0x0800, 0x0800, 0x5110, 0x5111};
    const uint32_t values[] = {600000, 1200000u | (1u << 30) | (1u << 31), 12, uint32_t(-34)};
    for (unsigned i = 0; i < sensors.size(); i++) {
        smartPortPayload_t payload = {};
        EXPECT_EQ(ids[i], sensors[i]->app_id);
        sensors[i]->encode(sensors[i], &payload);
        EXPECT_EQ(values[i], payload.data);
    }
}

TEST_F(SmartPortSensorsTest, FullConfigurationAndCatalogueExpansionBound)
{
    std::vector<sensor_id_e> supported;
    unsigned extraEntries = 0;
    // Audit every ID so future catalogue additions that expand a selection into
    // multiple wire entries must also update the runtime capacity.
    for (int id = 1; id < TELEM_SENSOR_COUNT; id++) {
        telemetryConfigMutable()->telemetry_sensors[0] = id;
        initSmartPortSensors();
        auto sensors = scheduledSensors();
        if (!sensors.empty()) {
            supported.push_back(sensor_id_e(id));
            extraEntries += sensors.size() - 1;
        }
    }
    EXPECT_EQ(2u, extraEntries);
    ASSERT_GE(supported.size(), size_t(TELEM_SENSOR_SLOT_COUNT));

    telemetryConfigMutable()->telemetry_sensors[0] = TELEM_GPS_COORD;
    telemetryConfigMutable()->telemetry_sensors[1] = TELEM_ADJFUNC;
    unsigned slot = 2;
    for (auto id : supported) {
        if (id != TELEM_GPS_COORD && id != TELEM_ADJFUNC && slot < TELEM_SENSOR_SLOT_COUNT)
            telemetryConfigMutable()->telemetry_sensors[slot++] = id;
    }
    initSmartPortSensors();
    auto sensors = scheduledSensors();
    ASSERT_EQ(size_t(TELEM_SENSOR_SLOT_COUNT + 2), sensors.size());
    std::set<unsigned> selectedIds;
    for (unsigned i = 0; i < sensors.size(); i++) {
        EXPECT_EQ(i, sensors[i]->index);
        EXPECT_TRUE(sensors[i]->active);
        selectedIds.insert(sensors[i]->sensor_id);
    }
    EXPECT_EQ(size_t(TELEM_SENSOR_SLOT_COUNT), selectedIds.size());
}

class CrsfSensorsTest : public TelemetrySensorsTest {
protected:
    void SetUp() override
    {
        TelemetrySensorsTest::SetUp();
        crsfActive = true;
        crsfFrames.clear();
        telemetryConfigMutable()->crsf_telemetry_mode = CRSF_TELEMETRY_MODE_CUSTOM;
        telemetryConfigMutable()->crsf_telemetry_link_rate = 1000;
        telemetryConfigMutable()->crsf_telemetry_link_ratio = 1;
    }

    void sendNextFrame()
    {
        crsfTestTime += 10000;
        handleCrsfTelemetry(crsfTestTime);
    }

    void expectCustomFrame(const std::vector<uint8_t>& frame, uint8_t sequence,
        const std::vector<uint8_t>& payload)
    {
        std::vector<uint8_t> expected = {CRSF_SYNC_BYTE, uint8_t(payload.size() + 5),
            CRSF_FRAMETYPE_CUSTOM_TELEM, CRSF_ADDRESS_RADIO_TRANSMITTER,
            CRSF_ADDRESS_FLIGHT_CONTROLLER, sequence};
        expected.insert(expected.end(), payload.begin(), payload.end());
        uint8_t crc = 0;
        for (size_t i = 2; i < expected.size(); i++)
            crc = crc8_dvb_s2(crc, expected[i]);
        expected.push_back(crc);
        EXPECT_EQ(expected, frame);
    }
};

TEST_F(CrsfSensorsTest, EmptyUnsupportedAndInactiveSelections)
{
    initCrsfTelemetry();
    EXPECT_EQ(nullptr, telemetryScheduleNext());
    telemetryScheduleUpdate(1000);
    EXPECT_EQ(nullptr, telemetryScheduleNext());

    telemetryConfigMutable()->telemetry_sensors[0] = UINT16_MAX;
    telemetryConfigMutable()->telemetry_sensors[1] = TELEM_BATTERY_VOLTAGE;
    inactiveSensor = TELEM_BATTERY_VOLTAGE;
    initCrsfTelemetry();
    EXPECT_EQ(nullptr, telemetryScheduleNext());
}

TEST_F(CrsfSensorsTest, CustomScalingDuplicateOverridesAndReinitialization)
{
    for (unsigned i = 0; i < TELEM_SENSOR_SLOT_COUNT; i++) {
        telemetryConfigMutable()->telemetry_sensors[i] = TELEM_ESC1_CURRENT;
        telemetryConfigMutable()->telemetry_interval[i] = 200 + i;
    }
    initCrsfTelemetry();
    telemetryScheduleUpdate(1000);
    auto sensors = scheduledSensors();
    ASSERT_EQ(1u, sensors.size());
    EXPECT_EQ(0x1042, sensors[0]->app_id);
    EXPECT_EQ(239, sensors[0]->fast_interval);
    EXPECT_EQ(1234, sensors[0]->value);
    uint8_t data[2] = {};
    sbuf_t buf = {data, data + sizeof(data)};
    sensors[0]->encode(sensors[0], &buf);
    EXPECT_EQ(0x04, data[0]);
    EXPECT_EQ(0xD2, data[1]);

    memset(telemetryConfigMutable()->telemetry_sensors, 0, sizeof(telemetryConfig()->telemetry_sensors));
    telemetryConfigMutable()->telemetry_sensors[0] = TELEM_ESC1_CURRENT;
    telemetryConfigMutable()->telemetry_interval[0] = 0;
    initCrsfTelemetry();
    sensors = scheduledSensors();
    ASSERT_EQ(1u, sensors.size());
    EXPECT_EQ(200, sensors[0]->fast_interval);
    EXPECT_GE(sensors[0]->slow_interval, 3000);
    EXPECT_LT(sensors[0]->slow_interval, 3100);
    EXPECT_EQ(0, sensors[0]->value);
}

TEST_F(CrsfSensorsTest, NativeCatalogueIntervalsAndModeSwitching)
{
    const sensor_id_e ids[] = {TELEM_FLIGHT_MODE, TELEM_BATTERY, TELEM_ATTITUDE,
        TELEM_ALTITUDE, TELEM_GPS, TELEM_RPM, TELEM_TEMP};
    telemetryConfigMutable()->crsf_telemetry_mode = CRSF_TELEMETRY_MODE_NATIVE;
    for (unsigned i = 0; i < sizeof(ids) / sizeof(ids[0]); i++) {
        telemetryConfigMutable()->telemetry_sensors[i] = ids[i];
        telemetryConfigMutable()->telemetry_interval[i] = i ? 100 + i : 0;
    }
    initCrsfTelemetry();
    auto sensors = scheduledSensors();
    ASSERT_EQ(7u, sensors.size());
    for (unsigned i = 0; i < sensors.size(); i++) {
        EXPECT_EQ(ids[i], sensors[i]->sensor_id);
        EXPECT_EQ(100 + i, sensors[i]->fast_interval);
        EXPECT_EQ(100 + i, sensors[i]->slow_interval);
    }

    memset(telemetryConfigMutable()->telemetry_sensors, 0, sizeof(telemetryConfig()->telemetry_sensors));
    telemetryConfigMutable()->telemetry_sensors[0] = TELEM_ESC1_CURRENT;
    telemetryConfigMutable()->crsf_telemetry_mode = CRSF_TELEMETRY_MODE_CUSTOM;
    initCrsfTelemetry();
    sensors = scheduledSensors();
    ASSERT_EQ(1u, sensors.size());
    EXPECT_EQ(TELEM_ESC1_CURRENT, sensors[0]->sensor_id);

    telemetryConfigMutable()->crsf_telemetry_mode = CRSF_TELEMETRY_MODE_NATIVE;
    initCrsfTelemetry();
    EXPECT_EQ(nullptr, telemetryScheduleNext());
}

TEST_F(CrsfSensorsTest, FullCustomConfigurationRetainsDiscoverySlot)
{
    std::vector<sensor_id_e> supported;
    for (int id = 1; id < TELEM_SENSOR_COUNT; id++) {
        telemetryConfigMutable()->telemetry_sensors[0] = id;
        initCrsfTelemetry();
        auto sensors = scheduledSensors();
        if (!sensors.empty()) {
            // One entry per selectable ID is the runtime allocation invariant.
            ASSERT_EQ(1u, sensors.size());
            supported.push_back(sensor_id_e(id));
        }
    }
    EXPECT_EQ(82u, supported.size());
    ASSERT_GE(supported.size(), size_t(TELEM_SENSOR_SLOT_COUNT));
    for (unsigned i = 0; i < TELEM_SENSOR_SLOT_COUNT; i++)
        telemetryConfigMutable()->telemetry_sensors[i] = supported[i];
    initCrsfTelemetry();
    auto sensors = scheduledSensors();
    ASSERT_EQ(size_t(TELEM_SENSOR_SLOT_COUNT), sensors.size());
    for (unsigned i = 0; i < sensors.size(); i++) {
        EXPECT_EQ(i + 1, sensors[i]->index); // index zero is the inactive discovery marker
        EXPECT_TRUE(sensors[i]->active);
    }
}

TEST_F(CrsfSensorsTest, CustomDiscoveryAndScheduledFramesPreserveWireFormat)
{
    telemetryConfigMutable()->telemetry_sensors[7] = TELEM_ESC1_CURRENT;
    telemetryConfigMutable()->telemetry_sensors[9] = TELEM_ESC1_CURRENT;
    telemetryConfigMutable()->telemetry_sensors[12] = TELEM_GPS_COORD;
    telemetryConfigMutable()->telemetry_sensors[39] = TELEM_BATTERY_CURRENT;
    gpsSol.llh.lat = 1;
    gpsSol.llh.lon = -2;
    initCrsfTelemetry();

    // Discovery starts with ten reset markers, independent of selected sensors.
    for (unsigned i = 0; i < 10; i++) {
        sendNextFrame();
        ASSERT_EQ(i + 1, crsfFrames.size());
        expectCustomFrame(crsfFrames.back(), 0, {0x10, 0x00});
    }
    const std::vector<std::vector<uint8_t>> payloads = {
        {0x10, 0x42, 0x04, 0xD2},
        {0x10, 0x42, 0x04, 0xD2},
        {0x11, 0x25, 0, 0, 0, 1, 0xFF, 0xFF, 0xFF, 0xFE},
        {0x10, 0x12, 0x30, 0x34},
    };
    for (unsigned i = 0; i < payloads.size(); i++) {
        sendNextFrame();
        ASSERT_EQ(11 + i, crsfFrames.size());
        expectCustomFrame(crsfFrames.back(), i, payloads[i]);
    }
    sendNextFrame(); // complete discovery
    ASSERT_EQ(14u, crsfFrames.size());
    sendNextFrame();
    ASSERT_EQ(15u, crsfFrames.size());
    expectCustomFrame(crsfFrames.back(), 4, {
        0x10, 0x12, 0x30, 0x34, // battery current
        0x10, 0x42, 0x04, 0xD2, // ESC current (duplicate selection sends once)
        0x11, 0x25, 0, 0, 0, 1, 0xFF, 0xFF, 0xFF, 0xFE,
    });
}

TEST_F(CrsfSensorsTest, NativeBatteryFrameAndDisabledReceiver)
{
    telemetryConfigMutable()->crsf_telemetry_mode = CRSF_TELEMETRY_MODE_NATIVE;
    telemetryConfigMutable()->telemetry_sensors[0] = TELEM_BATTERY;
    crsfActive = false;
    initCrsfTelemetry();
    EXPECT_FALSE(checkCrsfTelemetryState());
    sendNextFrame();
    EXPECT_TRUE(crsfFrames.empty());

    crsfActive = true;
    initCrsfTelemetry();
    sendNextFrame();
    ASSERT_EQ(1u, crsfFrames.size());
    const std::vector<uint8_t> expected = {CRSF_SYNC_BYTE, 10, CRSF_FRAMETYPE_BATTERY_SENSOR,
        0, 120, 0, 10, 0, 0, 100, 90};
    ASSERT_EQ(expected.size() + 1, crsfFrames[0].size());
    EXPECT_TRUE(std::equal(expected.begin(), expected.end(), crsfFrames[0].begin()));
}
