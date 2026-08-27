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

Requires the `jsbsim` package (see tools/jsbsim-venv; create it with
`scripts/sitl-jsbsim-flightgear-launch.ps1 -SetupVenv`, or manually with
`python -m venv tools/jsbsim-venv` then
`tools/jsbsim-venv/Scripts/pip.exe install -r scripts/requirements-jsbsim.txt`).
"""
import argparse
import atexit
import math
import os
import signal
import socket
import struct
import sys
import tempfile
import time

os.environ.setdefault("JSBSIM_DEBUG", "0")  # suppress verbose model-load logging

try:
    import jsbsim
except ImportError:
    sys.exit(
        "The 'jsbsim' package is not installed in this Python environment.\n"
        "Create/use the venv and install it, e.g.:\n"
        "  python -m venv tools/jsbsim-venv\n"
        "  tools/jsbsim-venv/Scripts/pip.exe install -r scripts/requirements-jsbsim.txt\n"
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

# Wingflight's M1 (throttle) does NOT land in motor_speed[0]: target.c's
# pwmCompleteMotorUpdate() applies the legacy Gazebo ArduCopterPlugin motor
# remap, so motorsPwm[0] (== M1) is written to motor_speed[3]:
#     pwmPkt.motor_speed[3] = motorsPwm[0] / 1000.0;   <-- M1, the wing throttle
#     pwmPkt.motor_speed[0] = motorsPwm[1] / 1000.0;
#     pwmPkt.motor_speed[1] = motorsPwm[2] / 1000.0;
#     pwmPkt.motor_speed[2] = motorsPwm[3] / 1000.0;
# A single-motor fixed-wing build never writes motorsPwm[1..3] at all, so
# reading motor_speed[0] here yields a permanent 0.0 throttle. Override with
# --throttle-motor-index if you change that remap in target.c.
DEFAULT_THROTTLE_MOTOR_INDEX = 3

# Default initial position: KSFO, which the FlightGear base package ships
# scenery for. JSBSim's own default IC is lat/lon 0/0 (open ocean, no
# FlightGear scenery), which renders as a featureless blue void.
DEFAULT_LAT_DEG = 37.6136
DEFAULT_LON_DEG = -122.3572


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


class SurfaceScaler:
    """Servo pulse (us) -> JSBSim [-1, 1] surface command, split around mid.

    min/mid/max default to the usual 1000/1500/2000 us servo range; override
    them (--servo-min/--servo-mid/--servo-max) if the airframe's servo config
    uses a different range, otherwise full stick deflection won't map to full
    JSBSim surface deflection.
    """

    def __init__(self, lo=1000.0, mid=1500.0, hi=2000.0):
        self.lo, self.mid, self.hi = lo, mid, hi

    def __call__(self, us):
        if us <= 0:  # unpopulated channel (target.c writes 0 past getServoCount())
            return 0.0
        if us >= self.mid:
            span = max(1e-6, self.hi - self.mid)
            return max(0.0, min(1.0, (us - self.mid) / span))
        span = max(1e-6, self.mid - self.lo)
        return max(-1.0, min(0.0, (us - self.mid) / span))


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
        except (ConnectionResetError, OSError):
            # Windows: a UDP send to a port with no listener can surface later
            # as WSAECONNRESET on this socket. Harmless here - keep going.
            break
        if len(data) == SERVO_PACKET_SIZE:
            state.update_from_packet(data)
            received = True
    return received


def apply_actuators(fdm, state, scale, cfg):
    aileron = 0.5 * (scale(state.servo[S_LEFT_AILERON]) - scale(state.servo[S_RIGHT_AILERON]))
    fdm["fcs/aileron-cmd-norm"] = cfg.aileron_sign * aileron
    fdm["fcs/elevator-cmd-norm"] = cfg.elevator_sign * scale(state.servo[S_ELEVATOR])
    fdm["fcs/rudder-cmd-norm"] = cfg.rudder_sign * scale(state.servo[S_RUDDER])
    fdm["fcs/throttle-cmd-norm"] = max(0.0, min(1.0, state.motor_speed[cfg.throttle_index]))


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
        # projection) -- good enough since target.c only consumes the "down"
        # component (as a relative barometric altitude).
        fdm.get_property_value("position/distance-from-start-lat-mt"),
        fdm.get_property_value("position/distance-from-start-lon-mt"),
        -(fdm.get_property_value("position/h-sl-ft") - initial_altitude_ft) * FT_TO_M,
    )


def state_is_sane(fdm, min_altitude_ft):
    """False once JSBSim's state has gone non-finite or hit the ground.

    Either way the FDM stops producing usable IMU data (NaNs propagate
    straight into Wingflight's fake gyro/acc via updateState()), so the
    caller re-runs the initial conditions instead of streaming garbage.
    """
    for prop in ("attitude/phi-rad", "attitude/theta-rad", "attitude/psi-rad",
                 "velocities/p-rad_sec", "velocities/q-rad_sec", "velocities/r-rad_sec",
                 "accelerations/n-pilot-x-norm", "accelerations/n-pilot-z-norm",
                 "position/h-sl-ft"):
        if not math.isfinite(fdm.get_property_value(prop)):
            return False
    return fdm.get_property_value("position/h-agl-ft") > min_altitude_ft


def apply_initial_conditions(fdm, args):
    fdm["ic/lat-gc-deg"] = args.lat_deg
    fdm["ic/long-gc-deg"] = args.lon_deg
    fdm["ic/psi-true-deg"] = args.heading_deg
    fdm["ic/h-sl-ft"] = args.altitude_ft
    fdm["ic/vc-kts"] = args.airspeed_kts
    fdm["ic/gamma-deg"] = 0.0
    fdm.run_ic()

    if not args.no_engine_start:
        # A JSBSim piston model (c172p and friends) boots with the engine cold
        # and stopped, so fcs/throttle-cmd-norm produces exactly zero thrust no
        # matter what Wingflight's M1 output does. -1 means "all engines".
        fdm["propulsion/set-running"] = -1

    if args.trim:
        try:
            fdm.do_trim(args.trim_mode)
            print(f"[jsbsim-bridge] trimmed (mode {args.trim_mode})")
        except Exception as exc:  # JSBSim raises a plain RuntimeError on failure
            print(f"[jsbsim-bridge] WARNING: trim failed ({exc}); continuing untrimmed")


def write_flightgear_output_directive(host, port, rate):
    """Write a JSBSim output-directives file streaming to FlightGear's native
    FDM UDP protocol (JSBSim serializes FlightGear's net_fdm struct itself --
    see FGOutputFG in JSBSim). Must be registered via set_output_directive()
    *before* load_model() (JSBSim ignores it if called after).
    """
    xml = (
        '<?xml version="1.0"?>\n'
        f'<output name="{host}" type="FLIGHTGEAR" protocol="UDP" port="{port}" rate="{rate}"/>\n'
    )
    fd, path = tempfile.mkstemp(prefix="jsbsim_fg_output_", suffix=".xml")
    with os.fdopen(fd, "w") as f:
        f.write(xml)
    atexit.register(lambda: os.path.exists(path) and os.unlink(path))
    return path


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--host", default="127.0.0.1", help="Wingflight SITL host (default: 127.0.0.1)")
    parser.add_argument("--recv-port", type=int, default=9002, help="Port to receive servo_packet on (default: 9002)")
    parser.add_argument("--send-port", type=int, default=9003, help="Port to send fdm_packet to (default: 9003)")
    parser.add_argument("--aircraft", default="c172p", help="JSBSim aircraft model name (default: c172p)")
    parser.add_argument("--rate", type=float, default=120.0, help="Simulation/send rate in Hz (default: 120)")
    parser.add_argument("--altitude-ft", type=float, default=3000.0, help="Initial altitude, ft MSL (default: 3000)")
    parser.add_argument("--airspeed-kts", type=float, default=90.0, help="Initial calibrated airspeed, kts (default: 90)")
    parser.add_argument("--lat-deg", type=float, default=DEFAULT_LAT_DEG,
                        help=f"Initial latitude, deg (default: {DEFAULT_LAT_DEG}, KSFO - FlightGear ships scenery there)")
    parser.add_argument("--lon-deg", type=float, default=DEFAULT_LON_DEG,
                        help=f"Initial longitude, deg (default: {DEFAULT_LON_DEG})")
    parser.add_argument("--heading-deg", type=float, default=0.0, help="Initial true heading, deg (default: 0)")
    parser.add_argument("--trim", action="store_true",
                        help="Trim the aircraft for steady flight at the initial conditions before running")
    parser.add_argument("--no-engine-start", action="store_true",
                        help="Leave the engine(s) stopped (by default they are started, otherwise throttle does nothing)")
    parser.add_argument("--trim-mode", type=int, default=1,
                        help="JSBSim trim mode passed to do_trim() (default: 1 = full trim)")
    parser.add_argument("--throttle-motor-index", type=int, default=DEFAULT_THROTTLE_MOTOR_INDEX,
                        choices=[0, 1, 2, 3],
                        help="servo_packet.motor_speed[] index carrying M1/throttle "
                             f"(default: {DEFAULT_THROTTLE_MOTOR_INDEX} - target.c's Gazebo motor remap puts M1 there)")
    parser.add_argument("--servo-min", type=float, default=1000.0, help="Servo pulse at full negative deflection, us (default: 1000)")
    parser.add_argument("--servo-mid", type=float, default=1500.0, help="Servo pulse at neutral, us (default: 1500)")
    parser.add_argument("--servo-max", type=float, default=2000.0, help="Servo pulse at full positive deflection, us (default: 2000)")
    parser.add_argument("--invert-aileron", action="store_true", help="Flip the aileron command sign (if roll response is backwards)")
    parser.add_argument("--invert-elevator", action="store_true", help="Flip the elevator command sign")
    parser.add_argument("--invert-rudder", action="store_true", help="Flip the rudder command sign")
    parser.add_argument("--min-agl-ft", type=float, default=-5.0,
                        help="Reset the FDM when the aircraft drops below this AGL altitude (default: -5)")
    parser.add_argument("--no-auto-reset", action="store_true",
                        help="Keep streaming after a crash/NaN instead of re-running the initial conditions")
    parser.add_argument("--no-realtime", action="store_true", help="Run as fast as possible instead of pacing to wall-clock time")
    parser.add_argument("--status-interval", type=float, default=2.0, help="Seconds between status lines (default: 2)")
    parser.add_argument("--flightgear", action="store_true", help="Also stream state to FlightGear via JSBSim's native FDM UDP output")
    parser.add_argument("--fg-host", default="127.0.0.1", help="FlightGear host (default: 127.0.0.1)")
    parser.add_argument("--fg-port", type=int, default=5550, help="FlightGear --native-fdm UDP port (default: 5550)")
    parser.add_argument("--fg-rate", type=float, default=30.0, help="FlightGear output rate in Hz (default: 30)")
    args = parser.parse_args(argv)

    args.aileron_sign = -1.0 if args.invert_aileron else 1.0
    args.elevator_sign = -1.0 if args.invert_elevator else 1.0
    args.rudder_sign = -1.0 if args.invert_rudder else 1.0
    args.throttle_index = args.throttle_motor_index
    return args


def main():
    args = parse_args()
    dt = 1.0 / args.rate

    # Launcher scripts redirect this process's stdout to a log file; without
    # line buffering nothing shows up there until the process exits, which
    # makes "is the bridge alive?" impossible to answer while it runs.
    try:
        sys.stdout.reconfigure(line_buffering=True)
    except (AttributeError, ValueError):
        pass

    # Stop-Process/SIGTERM should unwind through the same path as Ctrl+C so the
    # temp output-directive file is cleaned up and the sockets are closed.
    signal.signal(signal.SIGTERM, lambda *_: sys.exit(0))

    fdm = jsbsim.FGFDMExec(None)
    fdm.set_dt(dt)

    if args.flightgear:
        directive_path = write_flightgear_output_directive(args.fg_host, args.fg_port, args.fg_rate)
        fdm.set_output_directive(directive_path)
        print(
            f"[jsbsim-bridge] FlightGear output enabled -> {args.fg_host}:{args.fg_port} "
            f"@ {args.fg_rate}Hz. Launch FlightGear with:\n"
            f"  fgfs --aircraft={args.aircraft} --fdm=null "
            f"--native-fdm=socket,in,{int(args.fg_rate)},,{args.fg_port},udp "
            f"--lat={args.lat_deg} --lon={args.lon_deg} --altitude={args.altitude_ft} "
            f"--timeofday=noon --disable-real-weather-fetch --disable-clouds3d"
        )

    if not fdm.load_model(args.aircraft):
        sys.exit(f"Failed to load JSBSim aircraft model '{args.aircraft}'")

    apply_initial_conditions(fdm, args)
    initial_altitude_ft = fdm.get_property_value("position/h-sl-ft")

    recv_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    recv_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        recv_sock.bind((args.host, args.recv_port))
    except OSError as exc:
        sys.exit(
            f"Could not bind UDP {args.host}:{args.recv_port} ({exc}).\n"
            "Another jsbsim_bridge.py (or Gazebo) is probably still running - stop it first."
        )
    recv_sock.setblocking(False)

    send_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    send_addr = (args.host, args.send_port)

    scale = SurfaceScaler(args.servo_min, args.servo_mid, args.servo_max)
    state = ActuatorState()

    print(f"[jsbsim-bridge] aircraft={args.aircraft} rate={args.rate}Hz "
          f"recv={args.host}:{args.recv_port} send={args.host}:{args.send_port} "
          f"throttle=motor_speed[{args.throttle_index}] "
          f"ic=({args.lat_deg},{args.lon_deg}) {args.altitude_ft:.0f}ft {args.airspeed_kts:.0f}kts")
    print("[jsbsim-bridge] waiting for Wingflight servo_packet updates (idle defaults until then)...")

    next_status = time.monotonic() + args.status_interval
    next_step_wall = time.monotonic()
    packets_received = 0
    resets = 0
    try:
        while True:
            if drain_actuator_updates(recv_sock, state):
                packets_received += 1
            apply_actuators(fdm, state, scale, args)
            fdm.run()

            if not state_is_sane(fdm, args.min_agl_ft):
                if args.no_auto_reset:
                    print("[jsbsim-bridge] FDM state invalid (crashed or non-finite) and "
                          "--no-auto-reset given - stopping.")
                    break
                resets += 1
                print(f"[jsbsim-bridge] FDM state invalid (crashed or non-finite) - "
                      f"re-running initial conditions (reset #{resets})")
                apply_initial_conditions(fdm, args)

            send_sock.sendto(build_fdm_packet(fdm, initial_altitude_ft), send_addr)

            now = time.monotonic()
            if now >= next_status:
                print(
                    f"[jsbsim-bridge] t={fdm.get_sim_time():7.2f}s "
                    f"alt={fdm.get_property_value('position/h-sl-ft'):7.1f}ft "
                    f"ias={fdm.get_property_value('velocities/vc-kts'):5.1f}kts "
                    f"thr={fdm.get_property_value('fcs/throttle-cmd-norm'):.2f} "
                    f"ail={fdm.get_property_value('fcs/aileron-cmd-norm'):+.2f} "
                    f"ele={fdm.get_property_value('fcs/elevator-cmd-norm'):+.2f} "
                    f"rud={fdm.get_property_value('fcs/rudder-cmd-norm'):+.2f} "
                    f"packets_rx={packets_received}"
                )
                if packets_received == 0:
                    print("[jsbsim-bridge]   (no servo_packet received yet - is wingflight_SITL.elf running?)")
                next_status = now + args.status_interval

            if not args.no_realtime:
                next_step_wall += dt
                sleep_for = next_step_wall - time.monotonic()
                if sleep_for > 0:
                    time.sleep(sleep_for)
                else:
                    next_step_wall = time.monotonic()
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        recv_sock.close()
        send_sock.close()
        print("\n[jsbsim-bridge] stopped")


if __name__ == "__main__":
    main()
