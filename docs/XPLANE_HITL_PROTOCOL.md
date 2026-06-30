# Wingflight X-Plane 11/12 HITL Protocol Specification

**Version**: 1.0  
**Status**: Phase 1 - Protocol Definition  
**Last Updated**: 2026-06-30

---

## Table of Contents

1. [Overview](#overview)
2. [Protocol Architecture](#protocol-architecture)
3. [Packet Formats](#packet-formats)
4. [FDM Packet Details](#fdm-packet-details)
5. [PWM Packet Details](#pwm-packet-details)
6. [Heartbeat & Keep-Alive](#heartbeat--keep-alive)
7. [Communication Parameters](#communication-parameters)
8. [Error Handling](#error-handling)
9. [Channel Mappings](#channel-mappings)
10. [Examples](#examples)

---

## Overview

The Wingflight X-Plane HITL Protocol enables hardware-in-the-loop testing of Wingflight flight control firmware using X-Plane 11 or 12 as the physics engine and flight dynamics simulator.

### Goals

- **Low-Latency Communication**: Sub-50ms frame latency for responsive aircraft control
- **Reliable Data Transfer**: CRC16 checksums and validation for error detection
- **Cross-Platform**: Windows, macOS, Linux compatibility via UDP
- **Real-Time Flight Dynamics**: Accurate sensor data from X-Plane to firmware
- **Motor Command Execution**: Responsive motor/servo control of simulated aircraft

### Key Characteristics

| Property | Value |
|----------|-------|
| Transport Protocol | UDP |
| Update Rate | ~20 Hz (50ms frames) |
| Packet Format | Binary, fixed-size structures |
| Error Detection | CRC16 checksum on all packets |
| Timeout Detection | 5-second heartbeat timeout |
| Architecture | Request/Response (unidirectional flow) |

---

## Protocol Architecture

### Communication Flow Diagram

```
┌─────────────────────────────────────────────────────────────┐
│                    X-Plane Application                      │
│                (Flight Simulator Engine)                    │
└───────────────────────┬─────────────────────────────────────┘
                        │
                  UDP Socket
                 (Port 5502 out)
                        │
                        ▼
        ┌───────────────────────────────────┐
        │   FDM Packet (Sensor Data)        │
        │ ~20 Hz, 180 bytes                 │
        │ • Attitude, Rates, Acceleration  │
        │ • GPS, Airspeed, Altitude        │
        │ • Magnetometer, Battery          │
        └───────────────────────────────────┘
                        │
                        ▼
┌─────────────────────────────────────────────────────────────┐
│                Wingflight SITL Firmware                     │
│              (Flight Control Algorithm)                     │
│                                                             │
│ • Processes sensor data                                    │
│ • Executes flight control logic                            │
│ • Computes motor commands                                 │
└──────────────────────┬──────────────────────────────────────┘
                       │
                 UDP Socket
                (Port 5503 out)
                       │
                       ▼
        ┌───────────────────────────────────┐
        │   PWM Packet (Motor Commands)     │
        │ ~20 Hz, 40 bytes                  │
        │ • 16 PWM channels (1000-2000 μs)  │
        │ • ARM/Failsafe status             │
        └───────────────────────────────────┘
                       │
                       ▼
┌─────────────────────────────────────────────────────────────┐
│                    X-Plane Application                      │
│              (Aircraft Animation & Physics)                │
│ • Apply servo/motor commands                              │
│ • Update aircraft state                                   │
│ • Loop back to FDM data generation                        │
└─────────────────────────────────────────────────────────────┘
```

### Network Configuration

**FDM Packet Stream (X-Plane → Firmware)**
- Source: X-Plane Plugin
- Destination: `localhost:5502` (UDP)
- Frequency: 20 Hz (every 50ms)
- Payload Size: ~180 bytes
- Direction: One-way (X-Plane broadcasts)

**PWM Packet Stream (Firmware → X-Plane)**
- Source: Wingflight SITL
- Destination: `localhost:5503` (UDP)
- Frequency: 20 Hz (every 50ms)
- Payload Size: ~40 bytes
- Direction: One-way (Firmware broadcasts)

**Heartbeat (Bidirectional)**
- Sent every 1 second by both endpoints
- Purpose: Connection health monitoring
- Timeout: 5 seconds without packets → Connection lost

---

## Packet Formats

### Packet Structure Overview

All packets follow a consistent header format for compatibility and error detection:

```
┌─────────────────────────────────────────┐
│         Packet Header (8 bytes)         │
├─────────────────────────────────────────┤
│         Magic Number (4 bytes)          │
│         Version (1 byte)                │
│         Flags/Channel Count (1 byte)    │
│         CRC16 Checksum (2 bytes)        │
├─────────────────────────────────────────┤
│                                         │
│       Packet-Specific Payload           │
│   (size depends on packet type)         │
│                                         │
└─────────────────────────────────────────┘
```

### Magic Numbers

Each packet type is identified by a 4-byte magic number (32-bit unsigned integer):

| Packet Type | Magic (Hex) | Magic (ASCII) | Purpose |
|-------------|------------|---------------|---------|
| FDM Packet | `0x4658504E` | "FXPN" | Flight Data Model (Sensors) |
| PWM Packet | `0x5058504E` | "PXPN" | PWM Output (Motor Commands) |
| Heartbeat | `0x4558504E` | "HXPN" | Keep-Alive / Status |

---

## FDM Packet Details

### Packet Purpose

The FDM packet contains flight dynamics data from X-Plane. It includes all sensor readings necessary for the Wingflight firmware to operate: attitude, angular rates, acceleration, GPS position/velocity, airspeed, barometric altitude, compass heading, and battery status.

### Packet Size

- **Total Size**: 180 bytes (fixed)
- **Header**: 8 bytes
- **Payload**: 172 bytes

### Packet Structure (C Struct)

```c
typedef struct {
    // === PACKET HEADER (8 bytes) ===
    uint32_t magic;           // 0x4658504E ("FXPN")
    uint8_t version;          // Protocol version
    uint8_t flags;            // Reserved for future use
    uint16_t crc16;           // CRC16 checksum

    // === TIMING (8 bytes) ===
    uint32_t timestamp_ms;    // X-Plane timestamp
    float sim_speed;          // Simulation speed factor

    // === ATTITUDE (12 bytes) ===
    float roll;               // Radians (-π to +π)
    float pitch;              // Radians (-π/2 to +π/2)
    float yaw;                // Radians (-π to +π)

    // === ANGULAR RATES (12 bytes) ===
    float p;                  // Roll rate (rad/s)
    float q;                  // Pitch rate (rad/s)
    float r;                  // Yaw rate (rad/s)

    // === LINEAR ACCELERATION (12 bytes) ===
    float accel_x;            // X-axis acceleration (m/s²)
    float accel_y;            // Y-axis acceleration (m/s²)
    float accel_z;            // Z-axis acceleration (m/s²)

    // === GPS POSITION (16 bytes) ===
    double latitude;          // Decimal degrees
    double longitude;         // Decimal degrees
    float altitude_msl;       // Meters (mean sea level)
    uint8_t gps_fix_type;     // 0=no fix, 1=2D, 2=3D
    uint8_t num_satellites;   // Satellites in view
    uint16_t reserved1;

    // === GPS VELOCITY (12 bytes) ===
    float velocity_n;         // North (NED frame, m/s)
    float velocity_e;         // East (NED frame, m/s)
    float velocity_d;         // Down (NED frame, m/s)

    // === AIRSPEED & WIND (12 bytes) ===
    float indicated_airspeed; // Pitot tube reading (m/s)
    float true_airspeed;      // True airspeed (m/s)
    float ground_speed;       // Ground speed (m/s)

    // === BAROMETER & ALTITUDE (8 bytes) ===
    float pressure_pa;        // Pascals
    float pressure_alt;       // Pressure altitude (meters)

    // === MAGNETOMETER (12 bytes) ===
    float mag_x;              // X-axis field (μT)
    float mag_y;              // Y-axis field (μT)
    float mag_z;              // Z-axis field (μT)

    // === BATTERY / POWER (12 bytes) ===
    float battery_voltage;    // Volts
    float battery_current;    // Amperes
    float battery_capacity;   // mAh consumed

    // === ENGINE / THROTTLE STATE (8 bytes) ===
    float throttle;           // 0.0 to 1.0
    float rpm;                // Engine RPM
    float fuel_total;         // Liters
    uint32_t engine_flags;    // Status flags

    // === LANDING GEAR / FLAPS (8 bytes) ===
    float flap_position;      // 0.0 to 1.0
    float gear_deploy;        // 0.0=retracted, 1.0=deployed
    uint16_t reserved2;
    uint16_t reserved3;

    // === PAYLOAD / MISC (16 bytes) ===
    float wind_speed;         // m/s
    float wind_direction;     // Degrees (0-359)
    float air_density;        // kg/m³
    uint32_t system_time;     // System uptime (ms)

} xplane_fdm_packet_t;
```

### Sensor Data Mapping

#### Attitude (Roll, Pitch, Yaw)

- **Source**: X-Plane flight model
- **Coordinate System**: Body frame
- **Range**: 
  - Roll: -π to +π radians (-180° to +180°)
  - Pitch: -π/2 to +π/2 radians (-90° to +90°)
  - Yaw: -π to +π radians (-180° to +180°, magnetic north reference)
- **Usage**: Primary attitude source for INS/AHRS
- **Note**: Directly use or feed to AHRS as reference depending on firmware configuration

#### Angular Rates (P, Q, R)

- **Source**: X-Plane flight model derivatives
- **Coordinate System**: Body frame
- **Units**: radians per second (rad/s)
- **Range**: ±25 rad/s (typical for general aviation aircraft)
- **Usage**: Gyro data for rotational rate estimation
- **Note**: May be used directly or fed through gyro calibration filter

#### Linear Acceleration (X, Y, Z)

- **Source**: X-Plane aerodynamic forces + gravity
- **Coordinate System**: Body frame
- **Units**: meters per second squared (m/s²)
- **Range**: ±3g to ±5g (typical for aircraft maneuvers)
- **Usage**: Accelerometer data for attitude and specific force
- **Note**: Includes gravity component (9.81 m/s² DOWN in level flight)
- **Conversion**: To achieve ADC counts, apply scale factor specific to hardware (e.g., 4096 counts/g for 16-bit sensor)

#### GPS Position & Velocity

- **Latitude/Longitude**: WGS84 (World Geodetic System 1984)
- **Altitude**: Above mean sea level (MSL), meters
- **GPS Fix Type**: 
  - 0 = No fix
  - 1 = 2D fix (horizontal only)
  - 2 = 3D fix (horizontal + vertical)
- **Satellites**: Number of satellites in view
- **Velocity**: North-East-Down (NED) frame, m/s
- **Typical Accuracy**: 
  - Horizontal: 5-10 meters
  - Vertical: 10-20 meters
- **Note**: Simulated GPS data; accuracy can be tuned or degraded for testing

#### Airspeed & Wind

- **Indicated Airspeed (IAS)**: Read from simulated pitot tube (m/s)
  - This is what the aircraft instruments would display
  - Equal to TAS at sea level
- **True Airspeed (TAS)**: Actual speed relative to air mass (m/s)
- **Ground Speed**: Speed relative to ground (m/s)
  - Formula: `ground_speed = sqrt(velocity_n^2 + velocity_e^2)`
- **Wind Speed**: Magnitude of wind vector (m/s)
- **Wind Direction**: Direction FROM which wind blows, degrees (0-359)
  - 0° = from North
  - 90° = from East
- **Relationship**: `TAS² = (ground_speed_x - wind_x)² + (ground_speed_y - wind_y)²`

#### Barometric Pressure & Altitude

- **Pressure**: Barometric pressure in Pascals (Pa)
  - Typical sea level: 101,325 Pa
  - Formula converts to altitude: `altitude = 44330 * (1 - (pressure/101325)^(1/5.255))`
- **Pressure Altitude**: Pre-calculated altitude from pressure
  - Matches what altimeter would display
- **Usage**: Barometer/altimeter simulation

#### Magnetometer (Compass)

- **Values**: Magnetic field strength in each axis (microtesla, μT)
- **Coordinate System**: Body frame (aircraft reference)
- **Source**: Derived from X-Plane heading + local magnetic declination
- **Typical Range**: ±60 μT (Earth's magnetic field ≈ 25-65 μT depending on latitude)
- **Usage**: Compass heading estimation, magnetometer simulation
- **Note**: May include heading reference or be derived from attitude + heading

#### Battery Status

- **Voltage**: Battery cell voltage or pack voltage (volts)
  - 3S LiPo: 9-12.6V
  - 4S LiPo: 12-16.8V
- **Current**: Instantaneous draw (amperes)
  - Positive = discharge, Negative = charge
- **Capacity Consumed**: Total mAh drawn since simulation start
- **Usage**: Battery monitoring, voltage-based failsafe, power budget analysis

#### Engine / Throttle

- **Throttle**: Normalized throttle input (0.0 = idle, 1.0 = full throttle)
- **RPM**: Simulated engine RPM
- **Fuel**: Total available fuel (liters)
- **Flags**: Engine status bits (running, overtemp, etc.)

#### Landing Gear & Flaps

- **Flap Position**: Normalized flap deflection (0.0 = retracted, 1.0 = deployed)
- **Gear Deploy**: Landing gear position (0.0 = retracted, 1.0 = deployed)
- **Usage**: Aerodynamic configuration, weight distribution

---

## PWM Packet Details

### Packet Purpose

The PWM packet contains motor and servo command values from the Wingflight firmware. These values are interpreted by the X-Plane plugin to control aircraft surfaces (ailerons, elevators, rudder) and engine throttle.

### Packet Size

- **Total Size**: 40 bytes (fixed)
- **Header**: 8 bytes
- **PWM Channels**: 16 × 2 bytes = 32 bytes

### Packet Structure (C Struct)

```c
typedef struct {
    // === PACKET HEADER (8 bytes) ===
    uint32_t magic;           // 0x5058504E ("PXPN")
    uint8_t version;          // Protocol version
    uint8_t num_channels;     // Active PWM channels (1-16)
    uint16_t crc16;           // CRC16 checksum

    // === TIMING (4 bytes) ===
    uint32_t timestamp_ms;    // Firmware timestamp

    // === PWM VALUES (32 bytes) ===
    uint16_t pwm_values[16];  // 16 channels × 2 bytes each

    // === STATUS & FLAGS (4 bytes) ===
    uint8_t arm_state;        // 0=disarmed, 1=armed
    uint8_t flight_mode;      // Flight mode indicator
    uint8_t failsafe_active;  // 0=normal, 1=failsafe
    uint8_t reserved;

} xplane_pwm_packet_t;
```

### PWM Value Range & Mapping

#### Standard PWM Specification

| Parameter | Value | Unit |
|-----------|-------|------|
| Minimum | 1000 | μs (microseconds) |
| Center | 1500 | μs |
| Maximum | 2000 | μs |
| Range | 1000 | μs (2000-1000) |
| Update Rate | ~20 Hz | Hz |

#### Fixed-Wing Aircraft Channel Mapping

The following is the **recommended** channel mapping for fixed-wing aircraft. Individual configurations may vary based on aircraft design and control surface layout.

| Channel | Purpose | Typical Range | Remarks |
|---------|---------|---------------|---------|
| 0 | Throttle / Motor | 1000-2000 μs | Motor speed control, 1000=off, 2000=full |
| 1 | Aileron Left | 1000-2000 μs | Left wing trailing edge down = roll right |
| 2 | Aileron Right | 1000-2000 μs | Right wing trailing edge down = roll left |
| 3 | Elevator | 1000-2000 μs | Up = positive pitch |
| 4 | Rudder | 1000-2000 μs | Left = negative yaw (nose left) |
| 5 | Flaps | 1000-2000 μs | 1000=retracted, 2000=deployed |
| 6-15 | Auxiliary | 1000-2000 μs | Camera pan/tilt, landing gear, etc. |

#### PWM to Physical Mapping Example

**Elevator Servo** (Channel 3):
```
PWM Value → Elevator Deflection
1000 μs   → -25° (full pitch down)
1500 μs   → 0° (neutral)
2000 μs   → +25° (full pitch up)
```

**Throttle Motor** (Channel 0):
```
PWM Value → Motor Speed
1000 μs   → 0% (stopped)
1250 μs   → 25%
1500 μs   → 50%
1750 μs   → 75%
2000 μs   → 100%
```

### Status Flags

#### Arm State

```
0 = Disarmed
    → Motors will not spin
    → Servos may not respond
    → Safe mode for setup/testing
    
1 = Armed
    → Motors ready to spin on throttle command
    → Servos respond to control inputs
    → Active flight mode engaged
```

#### Flight Mode

```
Typical values:
0 = Manual / Stabilize
1 = Altitude Hold
2 = GPS Hold / Auto
3 = Return to Launch
4 = Mission
5 = Emergency / Failsafe
6-7 = Reserved
```

#### Failsafe Active

```
0 = Normal Operation
    → Firmware operating normally
    → Responding to RC input / autonomous commands
    
1 = Failsafe Triggered
    → Loss of signal or critical error
    → Aircraft operating on failsafe mode
    → May execute return-to-base or land sequence
```

---

## Heartbeat & Keep-Alive

### Heartbeat Packet

Heartbeat packets are lightweight keep-alive messages to detect connection loss and synchronize timing.

```c
typedef struct {
    uint32_t magic;           // 0x4558504E ("HXPN")
    uint32_t timestamp_ms;    // Sender's timestamp
    uint16_t crc16;           // Checksum
    uint16_t reserved;
} xplane_heartbeat_packet_t;
```

### Timing & Timeout

| Parameter | Value | Unit |
|-----------|-------|------|
| Heartbeat Interval | 1000 | ms (sent every 1 second) |
| Connection Timeout | 5000 | ms (no packets → connection lost) |
| Max Allowed Gap | 5 seconds | before declaring timeout |

### Connection State Machine

```
START
  ↓
[Idle] ← No packets received
  ↓ (wait 1s)
[Send Heartbeat]
  ↓
[Connected] ← FDM or PWM packet received
  ↓ (if no packets for 5s)
[Timeout / Disconnected]
  ↓
[Attempt Reconnect / Reset]
  ↓
[Idle]
```

---

## Communication Parameters

### Network Configuration (UDP)

| Parameter | FDM Stream | PWM Stream | Heartbeat |
|-----------|------------|------------|-----------|
| **Protocol** | UDP | UDP | UDP |
| **Port (Out)** | 5502 | 5503 | Variable |
| **Port (In)** | Variable | Variable | Variable |
| **Address** | 127.0.0.1 (localhost) | 127.0.0.1 | 127.0.0.1 |
| **Frequency** | 20 Hz | 20 Hz | 1 Hz |
| **Payload Size** | ~180 bytes | ~40 bytes | ~12 bytes |

### Timing & Synchronization

| Parameter | Value | Notes |
|-----------|-------|-------|
| **Frame Rate** | 20 Hz | 50ms per frame |
| **Packet Jitter Tolerance** | ±10% | Acceptable variation in timing |
| **Maximum Latency** | 100ms | End-to-end one-way latency acceptable |
| **Sync Method** | Timestamp in packet | Both FDM and PWM contain timestamp_ms |
| **Clock Reference** | X-Plane simulation time | FDM timestamp is authoritative |

### Byte Order (Endianness)

- **All packets use Little-Endian byte order** (Intel x86/x64 native format)
- Multi-byte integers (uint32_t, float, double) are stored with least significant byte first
- CRC16 calculation performed on raw binary data without byte-swapping

---

## Error Handling

### CRC16 Checksum Calculation

CRC16-CCITT is used for error detection on all packets. The checksum is computed over the entire packet excluding the checksum field itself.

```c
uint16_t crc16_ccitt(const uint8_t* data, size_t size) {
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
```

### Packet Validation

All received packets must pass validation before processing:

1. **Magic Number Check**: Verify packet type identifier (FXPN, PXPN, HXPN)
2. **Version Check**: Ensure protocol version matches (currently v1)
3. **Size Check**: Validate packet size is correct for packet type
4. **CRC16 Check**: Verify checksum against computed value
5. **Timestamp Check**: Warn if timestamp gap exceeds expected interval

### Error Recovery

| Error | Action |
|-------|--------|
| **Invalid Magic Number** | Discard packet, log warning |
| **Version Mismatch** | Log error, attempt to continue |
| **Bad CRC16** | Discard packet, count error |
| **Wrong Packet Size** | Discard packet, log error |
| **Timestamp Inversion** | Warn (clock adjustment) |
| **Timeout (no packets)** | Declare connection lost after 5s |

### Graceful Degradation

If connection is lost:

1. **Firmware → X-Plane**: Aircraft maintains last control inputs until timeout (pauses simulation or holds altitude)
2. **X-Plane → Firmware**: Firmware enters failsafe mode after 5-second timeout (may land autonomously or hold position)

---

## Channel Mappings

### X-Plane Control Mappings

The following table maps Wingflight PWM output channels to X-Plane control surface inputs:

| Wingflight Channel | X-Plane Control | PWM Range | Effect |
|--------------------|-----------------|-----------|--------|
| 0 (Throttle) | Throttle | 1000-2000 | 0% to 100% engine power |
| 1 (Aileron L) | Roll Input | 1000-2000 | -1.0 to +1.0 normalized |
| 2 (Aileron R) | Roll Input | 1000-2000 | -1.0 to +1.0 (differential) |
| 3 (Elevator) | Pitch Input | 1000-2000 | -1.0 to +1.0 normalized |
| 4 (Rudder) | Yaw Input | 1000-2000 | -1.0 to +1.0 normalized |
| 5 (Flaps) | Flap Lever | 1000-2000 | 0% to 100% deflection |
| 6-15 | Aux Inputs | 1000-2000 | Various (camera, gear, etc.) |

### Conversion Formula

```
Normalized Control = (PWM_value - 1000) / 1000 - 1.0

Example:
PWM 1000 → -1.0 (full deflection one direction)
PWM 1500 → 0.0 (neutral)
PWM 2000 → +1.0 (full deflection other direction)
```

---

## Examples

### Example 1: FDM Packet Initialization (C Code)

```c
#include "protocol_xplane.h"
#include <time.h>
#include <string.h>

void create_fdm_packet(xplane_fdm_packet_t* packet) {
    memset(packet, 0, sizeof(xplane_fdm_packet_t));
    
    // Header
    packet->magic = XPLANE_FDM_MAGIC;
    packet->version = XPLANE_PROTOCOL_VERSION;
    packet->flags = 0;
    
    // Timing
    packet->timestamp_ms = 1000;  // 1 second into simulation
    packet->sim_speed = 1.0f;     // Normal speed (1x)
    
    // Attitude (level flight, heading north)
    packet->roll = 0.0f;
    packet->pitch = 0.0f;
    packet->yaw = 0.0f;
    
    // Angular rates (steady)
    packet->p = 0.0f;
    packet->q = 0.0f;
    packet->r = 0.0f;
    
    // Acceleration (1g down in level flight)
    packet->accel_x = 0.0f;
    packet->accel_y = 0.0f;
    packet->accel_z = 9.81f;
    
    // GPS Position (Somewhere in the world)
    packet->latitude = 37.7749;   // San Francisco latitude
    packet->longitude = -122.4194; // San Francisco longitude
    packet->altitude_msl = 100.0f; // 100 meters AGL
    packet->gps_fix_type = 2;      // 3D fix
    packet->num_satellites = 12;
    
    // GPS Velocity (hovering)
    packet->velocity_n = 0.0f;
    packet->velocity_e = 0.0f;
    packet->velocity_d = 0.0f;
    
    // Airspeed (not moving)
    packet->indicated_airspeed = 0.0f;
    packet->true_airspeed = 0.0f;
    packet->ground_speed = 0.0f;
    
    // Barometer (standard conditions)
    packet->pressure_pa = 101325.0f;
    packet->pressure_alt = 100.0f;
    
    // Compass (pointing north)
    packet->mag_x = 25.0f;   // Earth's field ~25-65 μT
    packet->mag_y = 0.0f;
    packet->mag_z = -45.0f;
    
    // Battery (fresh)
    packet->battery_voltage = 11.4f;  // 3S LiPo = 11.4V nominal
    packet->battery_current = 5.0f;   // 5 amps draw
    packet->battery_capacity = 0.0f;  // No consumption yet
    
    // Engine state
    packet->throttle = 0.0f;
    packet->rpm = 0.0f;
    packet->fuel_total = 1000.0f;  // 1 liter available
    packet->engine_flags = 0;
    
    // Flaps and gear
    packet->flap_position = 0.0f;  // Retracted
    packet->gear_deploy = 1.0f;    // Down/deployed
    
    // Wind and misc
    packet->wind_speed = 5.0f;     // 5 m/s wind
    packet->wind_direction = 180.0f; // From south
    packet->air_density = 1.225f;  // Sea level
    packet->system_time = 1000;
    
    // Calculate CRC16 (placeholder)
    uint8_t* packet_bytes = (uint8_t*)packet;
    size_t crc_start = offsetof(xplane_fdm_packet_t, magic);
    size_t crc_end = offsetof(xplane_fdm_packet_t, crc16);
    size_t crc_size = XPLANE_FDM_PACKET_SIZE - sizeof(uint16_t);
    
    packet->crc16 = xplane_crc16(packet_bytes + crc_start, crc_size);
}
```

### Example 2: PWM Packet for Neutral Flight

```c
void create_pwm_packet(xplane_pwm_packet_t* packet) {
    memset(packet, 0, sizeof(xplane_pwm_packet_t));
    
    // Header
    packet->magic = XPLANE_PWM_MAGIC;
    packet->version = XPLANE_PROTOCOL_VERSION;
    packet->num_channels = 6;  // Using 6 channels
    
    // Timing
    packet->timestamp_ms = 1000;
    
    // All channels at neutral (1500 μs)
    for (int i = 0; i < XPLANE_PWM_CHANNELS; i++) {
        packet->pwm_values[i] = PWM_CENTER_VALUE;  // 1500 μs
    }
    
    // Override specific channels
    packet->pwm_values[0] = 1000;  // Throttle: OFF
    packet->pwm_values[3] = 1500;  // Elevator: Neutral
    packet->pwm_values[4] = 1500;  // Rudder: Neutral
    
    // Status
    packet->arm_state = 0;         // Disarmed
    packet->flight_mode = 0;       // Manual mode
    packet->failsafe_active = 0;   // Normal
    
    // Compute CRC
    uint8_t* packet_bytes = (uint8_t*)packet;
    size_t crc_size = XPLANE_PWM_PACKET_SIZE - sizeof(uint16_t);
    packet->crc16 = xplane_crc16(packet_bytes, crc_size);
}
```

### Example 3: UDP Packet Reception (Pseudocode)

```c
void receive_and_validate_fdm() {
    uint8_t buffer[256];
    int recv_size = udpRecv(&xplane_link, buffer, sizeof(buffer), 100);
    
    if (recv_size < XPLANE_FDM_PACKET_SIZE) {
        return;  // Incomplete packet
    }
    
    xplane_fdm_packet_t* fdm = (xplane_fdm_packet_t*)buffer;
    
    // Validate
    if (fdm->magic != XPLANE_FDM_MAGIC) {
        log_error("Invalid FDM magic: 0x%x", fdm->magic);
        return;
    }
    
    if (fdm->version != XPLANE_PROTOCOL_VERSION) {
        log_warning("Version mismatch: got %d, expected %d", 
                    fdm->version, XPLANE_PROTOCOL_VERSION);
    }
    
    uint16_t computed_crc = xplane_crc16((uint8_t*)fdm, 
                                        XPLANE_FDM_PACKET_SIZE - sizeof(uint16_t));
    if (computed_crc != fdm->crc16) {
        log_error("CRC16 mismatch");
        return;
    }
    
    // Process valid packet
    update_sensors_from_fdm(fdm);
}
```

---

## Conclusion

This protocol specification provides a robust, low-latency communication framework for Wingflight X-Plane HITL simulation. The use of fixed packet sizes, CRC16 validation, and heartbeat monitoring ensures reliable operation across different platforms and network conditions.

For implementation details and API usage, refer to `protocol_xplane.h` and the Phase 2-3 implementation documentation.

---

**Document Version**: 1.0  
**Last Modified**: 2026-06-30  
**Status**: DRAFT (Ready for Phase 1 Implementation Review)
