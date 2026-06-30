# Wingflight X-Plane 11/12 HITL Target

## Overview

This target enables Wingflight firmware to run in **Hardware-in-the-Loop (HITL)** simulation mode connected to X-Plane 11 or 12 flight simulator via UDP protocol.

The system allows testing complete Wingflight flight control logic with realistic flight physics from X-Plane without requiring actual hardware.

## Architecture

### Target Structure

```
src/main/target/XPLANE/
├── target.h              # Target configuration (sensors, features)
├── target.c              # Target initialization and main loop
├── target.mk             # Build configuration
├── protocol_xplane.h     # X-Plane protocol packet definitions
├── protocol_xplane.c     # Protocol utility functions (CRC, validation)
├── xplane_link.h         # UDP communication API
├── xplane_link.c         # UDP implementation
└── README.md             # This file
```

### Communication Flow

```
X-Plane 11/12
    │
    └─→ UDP FDM Packet (port 5502)
           ↓
    [Wingflight XPLANE Target]
           │ Process sensors
           │ Run flight control
           │
           └─→ UDP PWM Packet (port 5503)
                  ↓
          Update aircraft controls
```

## Building

### Prerequisites

- **Host System**: Windows, macOS, or Linux
- **X-Plane SDK v4.0+**: For plugin development
- **C/C++ Compiler**: GCC, Clang, or MSVC
- **CMake**: 3.10+
- **Network**: UDP capable (localhost or LAN)

### Build Command

```bash
# Build Wingflight XPLANE target
make TARGET=XPLANE

# Clean build
make TARGET=XPLANE clean

# Build with debug symbols
make TARGET=XPLANE DEBUG=1
```

## Configuration

### Network Ports

| Direction | Port | Purpose |
|-----------|------|---------|
| ← Input | 5502 | FDM packets from X-Plane |
| → Output | 5503 | PWM commands to X-Plane |

These can be modified in `target.c`:
- `XPLANE_FDM_LISTEN_PORT` - Incoming FDM data
- `XPLANE_PWM_SEND_PORT` - Outgoing PWM commands

### Configurator Connection

The Wingflight Configurator can connect to XPLANE target via:
- **Port**: TCP 5761 (UART1 on host SITL)
- **Address**: localhost (127.0.0.1)
- **Protocol**: MSP over TCP

Configuration is handled by `serial_tcp.c` driver.

## Protocol Details

### FDM Packet (X-Plane → Wingflight)

Sent at ~20 Hz (50ms frames), contains:
- **Attitude**: Roll, Pitch, Yaw (radians)
- **Rates**: P, Q, R (rad/s)
- **Acceleration**: X, Y, Z body frame (m/s²)
- **GPS**: Position, velocity, fix quality
- **Airspeed**: Indicated, True, Ground speed
- **Barometer**: Pressure, altitude
- **Compass**: Magnetic field (X, Y, Z - μT)
- **Battery**: Voltage, current, capacity
- **Engine**: Throttle, RPM, fuel

**Packet Size**: 180 bytes
**Magic Number**: 0x4658504E ("FXPN")

### PWM Packet (Wingflight → X-Plane)

Sent at ~20 Hz (50ms frames), contains:
- **16 PWM Channels**: 1000-2000 μs each
- **Status**: Armed state, flight mode, failsafe
- **Timestamp**: Milliseconds since startup

**Packet Size**: 40 bytes
**Magic Number**: 0x5058504E ("PXPN")

### Channel Mapping (Fixed-Wing Aircraft)

| Channel | Purpose | Control Surface |
|---------|---------|------------------|
| 0 | Throttle | Motor/Engine speed |
| 1 | Aileron L | Left wing differential |
| 2 | Aileron R | Right wing differential |
| 3 | Elevator | Pitch control |
| 4 | Rudder | Yaw control |
| 5 | Flaps | Flap extension |
| 6-15 | Aux | Camera, gear, payload, etc. |

## Sensor Integration

### Accelerometer/Gyro (IMU)

- **Driver**: `accgyro_fake.c`
- **Source**: X-Plane FDM packet acceleration and rates
- **Conversion**: 
  - Acceleration: m/s² → ADC counts (scale by ACC_SCALE)
  - Angular rates: rad/s → ADC counts (scale by GYRO_SCALE)

### Barometer

- **Driver**: `barometer_fake.c`
- **Source**: X-Plane FDM pressure data
- **Conversion**: Pascals → altitude

### Magnetometer

- **Driver**: `compass_fake.c`
- **Source**: X-Plane FDM magnetic field
- **Conversion**: microtesla → compass ADC counts

### GPS

- **Status**: Implemented via FDM packet injection into `gpsSol`
- **Source**: X-Plane FDM GPS position and velocity
- **Details**: Fix state, LLH, speed/course, and satellite count are updated per packet

## Usage

### Starting XPLANE Target

1. **Run Wingflight XPLANE firmware**:
```bash
./obj/XPLANE_TARGET/wingflight_xplane
```

Firmware will print:
```
[XPLANE] Wingflight X-Plane HITL Target
[XPLANE] Initializing X-Plane HITL link...
[XPLANE] Listening on port 5502, sending to 127.0.0.1:5503
[XPLANE] Initialization complete. Waiting for X-Plane connection...
```

2. **Run X-Plane with HITL plugin**:
- Load X-Plane 11 or 12
- Load Wingflight X-Plane plugin (see plugin README)
- Plugin will connect to firmware at localhost:5502/5503

3. **Connect Configurator** (optional):
- Open Wingflight Configurator
- Connect to: 127.0.0.1:5761 (TCP)
- Configure flight modes, mixing, etc.

### Connection Status

Monitor connection status in firmware output:
```
[XPLANE] Status: CONNECTED, Packets: 1234, Errors: 0, Last packet: 12ms ago
```

### Troubleshooting

| Issue | Diagnosis | Solution |
|-------|-----------|----------|
| "Connection timeout" | No FDM packets received | Check X-Plane plugin is running and sending to port 5502 |
| Sensor values all zero | Firmware not receiving data | Verify network configuration (port 5502 open) |
| PWM not applied to aircraft | Plugin not receiving commands | Check port 5503 is accessible from plugin |
| Configurator won't connect | MSP connection issue | Verify TCP:5761 is available; check firewall |

## Development

### Adding New Sensors

To add a new sensor type (e.g., airspeed sensor):

1. **Create fake driver**: `drivers/airspeed/airspeed_xplane.c`
2. **Define header**: `drivers/airspeed/airspeed_xplane.h`
3. **Extract data**: In `xplane_process_fdm_packet()`, read from FDM packet
4. **Update driver**: Call update function with sensor data
5. **Update target.mk**: Add new source file

Example:
```c
// In target.c
airspeed_fake_t airspeed_data;
airspeed_data.airspeed = fdm->indicated_airspeed;
airspeed_fake_update(&airspeed_data);
```

### Protocol Extension

To extend the protocol (e.g., add new sensor fields):

1. **Update packet structure** in `protocol_xplane.h`
2. **Increment version**: `XPLANE_PROTOCOL_VERSION`
3. **Update documentation**: `XPLANE_HITL_PROTOCOL.md`
4. **Add validation**: In `xplane_validate_fdm_packet()`

## Reference Implementation

See INAV-X-Plane-XITL for reference:
- https://github.com/Scavanger/INAV-X-Plane-XITL

## Performance

### Frame Rates

- **FDM Update Rate**: 20 Hz (50ms)
- **PWM Update Rate**: 20 Hz (50ms)
- **Total Latency**: Typically <50ms (acceptable for HITL)

### CPU Usage

- **Wingflight XPLANE**: <5% CPU (single core)
- **X-Plane Plugin**: <3% CPU overhead

### Network Bandwidth

- **FDM Stream**: 20 × 180 bytes/s = 3.6 KB/s
- **PWM Stream**: 20 × 40 bytes/s = 0.8 KB/s
- **Total**: ~4.4 KB/s (negligible)

## Limitations

1. **No Real-Time Hardware**: SITL runs on desktop OS, timing may be subject to OS jitter
2. **Sensor Accuracy**: X-Plane sensor simulation is idealized (no realistic noise)
3. **No Actual PWM Output**: PWM values update X-Plane model, not real servos
4. **Single Aircraft**: Cannot test multi-aircraft scenarios
5. **Protocol Version**: Currently v1, backward compatibility needed for updates

## Future Enhancements

- [ ] Support for RealFlight simulator
- [ ] Multi-aircraft testing
- [ ] Distributed HITL (X-Plane on different machine)
- [ ] Log replay through HITL
- [ ] Automated testing framework
- [ ] Virtual RC receiver (RSSI simulation)
- [ ] Battery discharge simulation
- [ ] Rangefinder sensor support

## See Also

- [XPLANE_HITL_PROTOCOL.md](../docs/XPLANE_HITL_PROTOCOL.md) - Detailed protocol specification
- [XPLANE_HITL_IMPLEMENTATION_PLAN.md](../../XPLANE_HITL_IMPLEMENTATION_PLAN.md) - Full implementation roadmap
- X-Plane SDK Documentation: https://developer.x-plane.com/sdk/

## License

Wingflight is free software licensed under the GNU General Public License v3.
