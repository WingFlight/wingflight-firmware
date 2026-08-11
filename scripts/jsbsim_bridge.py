#!/usr/bin/env python
"""JSBSim <-> Wingflight SITL bridge.

Acts as the "simulator" side of Wingflight's TARGET=SITL UDP protocol
(src/main/target/SITL/target.c):
  - receives Wingflight's `servo_packet` (motor_speed[4] + servo[8] us) on
    <--host>:<--recv-port> (Wingflight sends here, see pwmLink in target.c),
  - drives a JSBSim flight dynamics model with those actuator values,
  - sends JSBSim's resulting state back as an `fdm_packet` to
    <--host>:<--send-port> (Wingflight listens here, see stateLink in
    target.c).

Wingflight's default fixed-wing mixer (see AGENTS.md) is:
    S1 = left aileron, S2 = right aileron (opposite sign),
    S3 = elevator, S4 = rudder, M1 = throttle.

Usage:
    tools/jsbsim-venv/Scripts/python.exe scripts/jsbsim_bridge.py

Requires the `jsbsim` package (see tools/jsbsim-venv, created with
`python -m venv tools/jsbsim-venv` then `pip install jsbsim`).
"""
import argparse
import math
import os
import socket
import struct
import sys
import time

os.environ.setdefault("JSBSIM_DEBUG", "0")  # suppress verbose model-load logging

try:
    import jsbsim
except ImportError:
    sys.exit(
        "The 'jsbsim' package is not installed in this Python environment.\n"
        "Create/use the venv and install it, e.g.:\n"
        "  python -m venv tools/jsbsim-venv\n"
        "  tools/jsbsim-venv/Scripts/pip.exe install jsbsim\n"
        "  tools/jsbsim-venv/Scripts/python.exe scripts/jsbsim_bridge.py"
    )

# Must match fdm_packet / servo_packet in src/main/target/SITL/target.h exactly
# (field order, types, native/little-endian layout).
FDM_PACKET_FMT = "<17d"
FDM_PACKET_SIZE = struct.calcsize(FDM_PACKET_FMT)
SERVO_PACKET_FMT = "<12f"
SERVO_PACKET_SIZE = struct.calcsize(SERVO_PACKET_FMT)

G_MPS2 = 9.80665
FT_TO_M = 0.3048

# S1-S4 servo array indices (0-based) in servo_packet.servo[8], per the
# default fixed-wing mixer (AGENTS.md).
S_LEFT_AILERON = 0
S_RIGHT_AILERON = 1
S_ELEVATOR = 2
S_RUDDER = 3


def euler_to_quat(phi, theta, psi):
    """Standard aerospace ZYX Euler (roll,pitch,yaw) -> earth-to-body quaternion (w,x,y,z)."""
    cphi, sphi = math.cos(phi * 0.5), math.sin(phi * 0.5)
    cth, sth = math.cos(theta * 0.5), math.sin(theta * 0.5)
    cpsi, spsi = math.cos(psi * 0.5), math.sin(psi * 0.5)
    qw = cphi * cth * cpsi + sphi * sth * spsi
    qx = sphi * cth * cpsi - cphi * sth * spsi
    qy = cphi * sth * cpsi + sphi * cth * spsi
    qz = cphi * cth * spsi - sphi * sth * cpsi
    return qw, qx, qy, qz


def us_to_surface_norm(us, lo=1000.0, mid=1500.0, hi=2000.0):
    """Servo pulse (us) -> JSBSim [-1, 1] surface command, split around mid."""
    if us <= 0:
        return 0.0
    if us >= mid:
        return max(0.0, min(1.0, (us - mid) / (hi - mid)))
    return max(-1.0, min(0.0, (us - mid) / (mid - lo)))


class ActuatorState:
    """Latest actuator values received from Wingflight (safe idle defaults)."""

    def __init__(self):
        self.motor_speed = [0.0, 0.0, 0.0, 0.0]
        self.servo = [1500.0] * 8

    def update_from_packet(self, data):
        values = struct.unpack(SERVO_PACKET_FMT, data)
        self.motor_speed = list(values[0:4])
        self.servo = list(values[4:12])


def drain_actuator_updates(sock, state):
    """Non-blocking: apply the most recent servo_packet available, if any."""
    received = False
    while True:
        try:
            data, _ = sock.recvfrom(SERVO_PACKET_SIZE)
        except BlockingIOError:
            break
        if len(data) == SERVO_PACKET_SIZE:
            state.update_from_packet(data)
            received = True
    return received


def apply_actuators(fdm, state):
    aileron = 0.5 * (
        us_to_surface_norm(state.servo[S_LEFT_AILERON])
        - us_to_surface_norm(state.servo[S_RIGHT_AILERON])
    )
    fdm["fcs/aileron-cmd-norm"] = aileron
    fdm["fcs/elevator-cmd-norm"] = us_to_surface_norm(state.servo[S_ELEVATOR])
    fdm["fcs/rudder-cmd-norm"] = us_to_surface_norm(state.servo[S_RUDDER])
    fdm["fcs/throttle-cmd-norm"] = max(0.0, min(1.0, state.motor_speed[0]))


def build_fdm_packet(fdm, initial_altitude_ft):
    phi = fdm.get_property_value("attitude/phi-rad")
    theta = fdm.get_property_value("attitude/theta-rad")
    psi = fdm.get_property_value("attitude/psi-rad")
    qw, qx, qy, qz = euler_to_quat(phi, theta, psi)

    # JSBSim's n-pilot-*-norm are load factors (specific force / g) in the
    # standard aerospace body frame (X-fwd, Y-right, Z-down) -- directly what
    # an accelerometer senses, matching the m/s^2 NED-body-frame value
    # target.c's updateState() expects (it applies its own sign convention).
    return struct.pack(
        FDM_PACKET_FMT,
        fdm.get_sim_time(),
        fdm.get_property_value("velocities/p-rad_sec"),
        fdm.get_property_value("velocities/q-rad_sec"),
        fdm.get_property_value("velocities/r-rad_sec"),
        fdm.get_property_value("accelerations/n-pilot-x-norm") * G_MPS2,
        fdm.get_property_value("accelerations/n-pilot-y-norm") * G_MPS2,
        fdm.get_property_value("accelerations/n-pilot-z-norm") * G_MPS2,
        qw, qx, qy, qz,
        fdm.get_property_value("velocities/v-north-fps") * FT_TO_M,
        fdm.get_property_value("velocities/v-east-fps") * FT_TO_M,
        fdm.get_property_value("velocities/v-down-fps") * FT_TO_M,
        # Position relative to the initial condition, NED, meters. JSBSim's
        # distance-from-start properties are an approximation (magnitude
        # along lat/lon directions, not a rigorous local-tangent-plane
        # projection) -- good enough since target.c doesn't currently
        # consume position/velocity for attitude/IMU purposes.
        fdm.get_property_value("position/distance-from-start-lat-mt"),
        fdm.get_property_value("position/distance-from-start-lon-mt"),
        -(fdm.get_property_value("position/h-sl-ft") - initial_altitude_ft) * FT_TO_M,
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--host", default="127.0.0.1", help="Wingflight SITL host (default: 127.0.0.1)")
    parser.add_argument("--recv-port", type=int, default=9002, help="Port to receive servo_packet on (default: 9002)")
    parser.add_argument("--send-port", type=int, default=9003, help="Port to send fdm_packet to (default: 9003)")
    parser.add_argument("--aircraft", default="c172p", help="JSBSim aircraft model name (default: c172p)")
    parser.add_argument("--rate", type=float, default=120.0, help="Simulation/send rate in Hz (default: 120)")
    parser.add_argument("--altitude-ft", type=float, default=3000.0, help="Initial altitude, ft MSL (default: 3000)")
    parser.add_argument("--airspeed-kts", type=float, default=90.0, help="Initial calibrated airspeed, kts (default: 90)")
    parser.add_argument("--no-realtime", action="store_true", help="Run as fast as possible instead of pacing to wall-clock time")
    parser.add_argument("--status-interval", type=float, default=2.0, help="Seconds between status lines (default: 2)")
    args = parser.parse_args()

    dt = 1.0 / args.rate

    fdm = jsbsim.FGFDMExec(None)
    fdm.set_dt(dt)
    if not fdm.load_model(args.aircraft):
        sys.exit(f"Failed to load JSBSim aircraft model '{args.aircraft}'")

    fdm["ic/h-sl-ft"] = args.altitude_ft
    fdm["ic/vc-kts"] = args.airspeed_kts
    fdm["ic/gamma-deg"] = 0.0
    fdm.run_ic()
    initial_altitude_ft = fdm.get_property_value("position/h-sl-ft")

    recv_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    recv_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    recv_sock.bind((args.host, args.recv_port))
    recv_sock.setblocking(False)

    send_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    send_addr = (args.host, args.send_port)

    state = ActuatorState()

    print(f"[jsbsim-bridge] aircraft={args.aircraft} rate={args.rate}Hz "
          f"recv={args.host}:{args.recv_port} send={args.host}:{args.send_port}")
    print("[jsbsim-bridge] waiting for Wingflight servo_packet updates (idle defaults until then)...")

    next_status = time.monotonic() + args.status_interval
    next_step_wall = time.monotonic()
    packets_received = 0
    try:
        while True:
            if drain_actuator_updates(recv_sock, state):
                packets_received += 1
            apply_actuators(fdm, state)
            fdm.run()
            send_sock.sendto(build_fdm_packet(fdm, initial_altitude_ft), send_addr)

            now = time.monotonic()
            if now >= next_status:
                print(
                    f"[jsbsim-bridge] t={fdm.get_sim_time():7.2f}s "
                    f"alt={fdm.get_property_value('position/h-sl-ft'):7.1f}ft "
                    f"ias={fdm.get_property_value('velocities/vc-kts'):5.1f}kts "
                    f"throttle={state.motor_speed[0]:.2f} "
                    f"packets_rx={packets_received}"
                )
                next_status = now + args.status_interval

            if not args.no_realtime:
                next_step_wall += dt
                sleep_for = next_step_wall - time.monotonic()
                if sleep_for > 0:
                    time.sleep(sleep_for)
                else:
                    next_step_wall = time.monotonic()
    except KeyboardInterrupt:
        print("\n[jsbsim-bridge] stopped")


if __name__ == "__main__":
    main()
