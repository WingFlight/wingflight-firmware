#!/usr/bin/env python3
"""SITL checks for Wingflight bidirectional (4D) throttle and reverse flight.

Drives a running wingflight_SITL.elf:
  RC      -> MSP_SET_RAW_RC on TCP 5761 (streamed at 50 Hz)
  queries -> MSP v1/v2 on TCP 5763
  sensors -> fdm_packet to UDP 9003 (fake gyro: rates follow the sticks, so the
             airborne latch can be triggered; level attitude otherwise)
  outputs <- servo_packet from UDP 9002 (motor_speed[4], servo[8] us)

Usage: harness.py config   (applies the 4D test config via CLI and saves; SITL exits)
       harness.py test     (runs the checks against the saved config)
"""
import socket, struct, sys, threading, time

HOST = "127.0.0.1"
RC_PORT, MSP_PORT, FDM_PORT, OUT_PORT = 5761, 5763, 9003, 9002
G = 9.80665

MSP_STATUS, MSP_SET_RAW_RC, MSP_SET_MODE_RANGE, MSP_ACC_CAL = 101, 200, 35, 205
MSP2_RF_CONFIG, MSP2_RF_STATUS = 0x5F37, 0x5F39
FLAG_NAMES = ["NO_GYRO", "FAILSAFE", "RX_FAILSAFE", "BAD_RX_RECOVERY", "BOXFAILSAFE",
              "GOVERNOR", "RPM_SIGNAL", "THROTTLE", "ANGLE", "BOOT_GRACE_TIME",
              "NOPREARM", "LOAD", "CALIBRATING", "CLI", "NO_ARM_SWITCH", "BST", "MSP",
              "PARALYZE", "GPS", "RESC", "RPMFILTER", "REBOOT_REQUIRED",
              "DSHOT_BITBANG", "ACC_CALIBRATION", "MOTOR_PROTOCOL", "OVERRIDE",
              "RX_INPUT_BACKUP", "ARM_SWITCH"]
STATES = ["off", "disarmed", "ground", "motorStopped", "forward", "handover", "reverse"]

results = []


def check(name, ok, detail=""):
    results.append((name, ok))
    print(f"[{'PASS' if ok else 'FAIL'}] {name}" + (f"  -- {detail}" if detail else ""), flush=True)


def flags_str(f):
    return "|".join(n for i, n in enumerate(FLAG_NAMES) if f & (1 << i)) or "(none)"


def crc8_dvb_s2(data):
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ 0xD5) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


class Msp:
    def __init__(self, port):
        self.sock = socket.create_connection((HOST, port), timeout=2)
        self.lock = threading.Lock()
        self.buf = b""

    def send_v1(self, cmd, payload=b""):
        crc = len(payload) ^ cmd
        for b in payload:
            crc ^= b
        self.sock.sendall(b"$M<" + bytes([len(payload), cmd]) + payload + bytes([crc]))

    def send_v2(self, cmd, payload=b""):
        body = struct.pack("<BHH", 0, cmd, len(payload)) + payload
        self.sock.sendall(b"$X<" + body + bytes([crc8_dvb_s2(body)]))

    def _read(self, n):
        while len(self.buf) < n:
            chunk = self.sock.recv(4096)
            if not chunk:
                raise ConnectionError("MSP socket closed")
            self.buf += chunk
        out, self.buf = self.buf[:n], self.buf[n:]
        return out

    def recv(self, want, timeout=1.0):
        """Next reply for command `want`; skips others."""
        end = time.time() + timeout
        while time.time() < end:
            # resync on '$'
            while True:
                c = self._read(1)
                if c == b"$":
                    break
            kind = self._read(2)
            if kind[0:1] == b"M":
                size, cmd = self._read(2)
                payload = self._read(size)
                self._read(1)
                err = kind[1:2] == b"!"
            elif kind[0:1] == b"X":
                _, cmd, size = struct.unpack("<BHH", self._read(5))
                payload = self._read(size)
                self._read(1)
                err = kind[1:2] == b"!"
            else:
                continue
            if cmd == want:
                return None if err else payload
        raise TimeoutError(f"no reply for {want:#x}")

    def query(self, cmd, payload=b"", v2=False):
        with self.lock:
            (self.send_v2 if v2 or cmd > 254 else self.send_v1)(cmd, payload)
            return self.recv(cmd)


class Sim:
    """RC streamer, fake FDM and output listener."""

    def __init__(self):
        self.ch = [1500] * 18
        self.ch[4:8] = [1000, 1000, 1000, 1000]   # AUX1-4 low
        self.rc_on = True
        self.gyro_follow = 0.0      # rad/s per unit of roll stick, to latch "airborne"
        self.out = None             # (motor[4], servo[8])
        self.out_t = 0
        self.stop = False
        self.rc = Msp(RC_PORT)
        self.fdm = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.listen = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.listen.bind((HOST, OUT_PORT))
        self.listen.settimeout(0.2)
        for fn in (self._rc_loop, self._fdm_loop, self._out_loop):
            threading.Thread(target=fn, daemon=True).start()

    def set(self, roll=None, pitch=None, thr=None, yaw=None, aux1=None, aux2=None, aux3=None):
        for i, v in ((0, roll), (1, pitch), (2, thr), (3, yaw), (4, aux1), (5, aux2), (6, aux3)):
            if v is not None:
                self.ch[i] = int(v)

    def _rc_loop(self):
        while not self.stop:
            if self.rc_on:
                try:
                    self.rc.send_v1(MSP_SET_RAW_RC, struct.pack("<18H", *self.ch))
                    self.rc.sock.setblocking(False)
                    try:
                        while self.rc.sock.recv(4096):
                            pass
                    except (BlockingIOError, OSError):
                        pass
                    self.rc.sock.setblocking(True)
                except OSError:
                    pass
            time.sleep(0.02)

    def _fdm_loop(self):
        t0 = time.time()
        while not self.stop:
            roll = (self.ch[0] - 1500) / 500.0
            p = self.gyro_follow * roll
            pkt = struct.pack("<17d", time.time() - t0, p, 0.0, 0.0, 0.0, 0.0, -G,
                              1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
            try:
                self.fdm.sendto(pkt, (HOST, FDM_PORT))
            except OSError:
                pass
            time.sleep(0.0025)

    def _out_loop(self):
        while not self.stop:
            try:
                data, _ = self.listen.recvfrom(256)
            except socket.timeout:
                continue
            if len(data) == 48:
                v = struct.unpack("<12f", data)
                self.out = (v[:4], v[4:])
                self.out_t = time.time()

    def motor(self):
        return self.out[0][0] if self.out else None

    def servo(self, i):
        return self.out[1][i] if self.out else None


def status(msp):
    p = msp.query(MSP_STATUS)
    armed = struct.unpack_from("<I", p, 6)[0] & 1
    flags = struct.unpack_from("<I", p, 17)[0]
    return bool(armed), flags


def rf_status(msp):
    p = msp.query(MSP2_RF_STATUS, v2=True)
    ver, en, st, dr, indb, ch = struct.unpack_from("<BBBbBH", p, 0)
    vals = [v / 1000 for v in struct.unpack_from("<6h", p, 7)]
    return dict(enabled=en, state=STATES[st] if st < len(STATES) else st, dir=dr, indb=indb,
                ch=ch, stick=vals[0], motor=vals[1], thrust=vals[2], speed=vals[3],
                vec=vals[4], surf=vals[5])


def step(us):
    return struct.pack("<b", int((us - 1500) / 5))


def mode_range(msp, slot, box, aux, lo, hi):
    msp.query(MSP_SET_MODE_RANGE, bytes([slot, box, aux]) + step(lo) + step(hi))


def cli(cmds):
    s = socket.create_connection((HOST, MSP_PORT), timeout=3)
    s.sendall(b"#\r\n")
    time.sleep(0.5)
    for c in cmds:
        s.sendall(c.encode() + b"\r\n")
        time.sleep(0.15)
    time.sleep(0.5)
    out = b""
    s.settimeout(0.5)
    try:
        while True:
            chunk = s.recv(65536)
            if not chunk:
                break
            out += chunk
    except (socket.timeout, OSError):
        pass
    s.close()
    return out.decode(errors="replace")


def do_config():
    out = cli([
        "set throttle_mode = BIDIRECTIONAL",
        "set motor_control_mode = 1,1,1,1",   # SITL has four motor outputs
        "set reverse_coupled_axes = 4",
        "mixer rule 5 set TP S5 1000 0",
        "get throttle_mode",
        "get reverse_coupled_axes",
        "save",
    ])
    print(out[-1500:])


def wait(sim, msp, secs, sample=None):
    end = time.time() + secs
    while time.time() < end:
        if sample:
            sample()
        time.sleep(0.01)


def do_test():
    sim = Sim()
    time.sleep(1.0)
    msp = Msp(MSP_PORT)

    rf = rf_status(msp)
    check("4D enabled in the firmware", rf["enabled"] == 1, str(rf))
    check("disarmed state, factors 1", rf["state"] == "disarmed" and rf["vec"] == 1 and rf["surf"] == 1)

    # ---- arming prerequisites: wait for sensors, calibrate acc ----
    sim.set(thr=1500, aux1=1000)
    acc_req = False
    last = None
    for _ in range(160):
        armed, flags = status(msp)
        if flags != last:
            print("  waiting, flags:", flags_str(flags))
            last = flags
        if flags & (1 << 23) and not acc_req:
            msp.query(MSP_ACC_CAL)
            acc_req = True
        # only the missing ARM switch may remain
        if not flags & ~(1 << 14) or flags == (1 << 14):
            break
        time.sleep(0.25)
    armed, flags = status(msp)
    print("flags before any ARM switch:", flags_str(flags))
    check("no ARM switch -> NO_ARM_SWITCH", bool(flags & (1 << 14)), flags_str(flags))

    # ARM on AUX1 high, THRUST VECTOR always on (AUX1 full range), MANUAL on AUX2 high
    mode_range(msp, 0, 0, 0, 1700, 2100)
    mode_range(msp, 1, 63, 0, 900, 2100)
    mode_range(msp, 2, 59, 1, 1700, 2100)
    time.sleep(0.5)
    armed, flags = status(msp)
    check("ARM switch set -> NO_ARM_SWITCH cleared", not flags & (1 << 14), flags_str(flags))

    # ---- throttle off = deadband around mid stick ----
    for us, expect_throttle_flag in ((1000, True), (1600, True), (1530, False), (1500, False)):
        sim.set(thr=us)
        time.sleep(0.4)
        armed, flags = status(msp)
        check(f"stick {us}us: THROTTLE flag {'set' if expect_throttle_flag else 'clear'}",
              bool(flags & (1 << 7)) == expect_throttle_flag, flags_str(flags))

    # ---- arm at mid stick ----
    sim.set(thr=1500)
    for _ in range(40):
        armed, flags = status(msp)
        if not flags:
            break
        time.sleep(0.25)
    check("all arming flags clear at mid stick", flags == 0, flags_str(flags))
    sim.set(aux1=2000)
    for _ in range(40):
        armed, flags = status(msp)
        if armed:
            break
        time.sleep(0.1)
    check("armed with the switch at mid stick", armed, flags_str(flags))
    if not armed:
        return
    time.sleep(0.3)
    rf = rf_status(msp)
    check("on the ground after arming", rf["state"] == "ground", str(rf))
    m = sim.motor()
    check("motor stopped at mid stick", m is not None and abs(m) < 0.005, f"motor_speed={m}")

    # ---- deadband and hysteresis on the motor output ----
    def motor_at(us, hold=0.4):
        sim.set(thr=us)
        time.sleep(hold)
        return sim.motor()

    m = motor_at(1535); check("1535us (+35, in deadband): motor 0", abs(m) < 0.005, f"{m:.3f}")
    m = motor_at(1550); check("1550us (+50): small forward", 0 < m < 0.05, f"{m:.3f}")
    m = motor_at(1450); check("1450us (-50, latched forward, before flip point): motor 0", abs(m) < 0.005, f"{m:.3f}")
    m = motor_at(1430); check("1430us (-70, past -60): reverse", m < 0, f"{m:.3f}")
    m = motor_at(1550); check("1550us (+50, latched reverse, before flip point): motor 0", abs(m) < 0.005, f"{m:.3f}")
    m = motor_at(1959); check("1959us (full forward): motor ~1", m > 0.98, f"{m:.3f}")
    m = motor_at(1041); check("1041us (full reverse): motor ~-1", m < -0.98, f"{m:.3f}")
    rf = rf_status(msp)
    check("ground: factors follow the latched direction", rf["vec"] == -1 and rf["surf"] == -1, str(rf))

    # ---- latch airborne: roll stick with a gyro that follows it ----
    sim.set(thr=1800)
    sim.gyro_follow = 2.0
    for _ in range(30):
        sim.set(roll=1800); time.sleep(0.4)
        sim.set(roll=1200); time.sleep(0.4)
        if rf_status(msp)["state"] != "ground":
            break
    sim.set(roll=1500)
    sim.gyro_follow = 0.0
    time.sleep(2.0)
    rf = rf_status(msp)
    check("airborne latched, forward flight", rf["state"] == "forward", str(rf))
    check("forward: gimbal and surface factor +1", rf["vec"] == 1 and rf["surf"] == 1, str(rf))

    # ---- the reversal: hold pitch and yaw stick so the outputs show the signs ----
    sim.set(pitch=1700, yaw=1700, thr=1862)       # ~70 % forward
    time.sleep(2.5)
    base_elev, base_gim, base_rud = sim.servo(2), sim.servo(4), sim.servo(3)
    print(f"forward, stick held: elevator S3={base_elev:.0f} gimbal S5={base_gim:.0f} rudder+gimbal S4={base_rud:.0f}")
    t0 = time.time()
    sim.set(thr=1138)                               # ~70 % reverse
    log = []
    while time.time() - t0 < 1.2:
        r = rf_status(msp)
        log.append((time.time() - t0, r, sim.servo(2), sim.servo(3), sim.servo(4)))
    first = lambda cond: next((t for t, r, *_ in log if cond(r)), None)
    t_handover = first(lambda r: r["state"] == "handover")
    t_vec = first(lambda r: r["vec"] < 0)
    t_surf = first(lambda r: r["surf"] < 0)
    print(f"handover at {t_handover}, gimbal flips at {t_vec}, surfaces back at {t_surf} (s after the stick)")
    check("handover state seen", t_handover is not None)
    check("gimbal flips ~55 ms after the stick (40-150 ms)", t_vec is not None and 0.03 <= t_vec <= 0.15, f"{t_vec}")
    check("surfaces back reversed ~280 ms after the stick (200-400 ms)", t_surf is not None and 0.2 <= t_surf <= 0.4, f"{t_surf}")
    wrong = [t for t, r, *_ in log if t_surf and t < t_surf - 0.01 and r["surf"] > 0.05 and t > 0.03]
    check("no forward-sign surface while stopping", not wrong, f"{len(wrong)} samples")

    # outputs: during the handover the elevator returns to centre while the coupled
    # rudder servo follows the gimbal sign; afterwards everything is reversed
    mid = [x for x in log if x[1]["state"] == "handover" and x[0] > 0.1]
    if mid:
        _, r, elev, rud, gim = mid[len(mid) // 2]
        print(f"mid-handover: elevator {elev:.0f} rudder+gimbal {rud:.0f} gimbal {gim:.0f}  (factors {r['vec']:+.2f}/{r['surf']:+.2f})")
    time.sleep(2.0)
    r = rf_status(msp)
    end_elev, end_gim, end_rud = sim.servo(2), sim.servo(4), sim.servo(3)
    print(f"reverse, stick held: elevator S3={end_elev:.0f} gimbal S5={end_gim:.0f} rudder+gimbal S4={end_rud:.0f}")
    check("reverse flight state, factors -1", r["state"] == "reverse" and r["vec"] == -1 and r["surf"] == -1, str(r))
    sgn = lambda v: (v > 1505) - (v < 1495)
    check("elevator deflection reversed", sgn(base_elev) == -sgn(end_elev) != 0, f"{base_elev:.0f} -> {end_elev:.0f}")
    check("gimbal deflection reversed", sgn(base_gim) == -sgn(end_gim) != 0, f"{base_gim:.0f} -> {end_gim:.0f}")
    check("coupled rudder deflection reversed", sgn(base_rud) == -sgn(end_rud) != 0, f"{base_rud:.0f} -> {end_rud:.0f}")

    # ---- MANUAL (AUX2 high): same flips without the gyro ----
    sim.set(aux2=2000)
    time.sleep(1.0)
    man_elev, man_gim = sim.servo(2), sim.servo(4)
    sim.set(thr=1862)
    time.sleep(2.0)
    man_elev_f, man_gim_f = sim.servo(2), sim.servo(4)
    print(f"MANUAL reverse: elevator {man_elev:.0f} gimbal {man_gim:.0f} | forward: elevator {man_elev_f:.0f} gimbal {man_gim_f:.0f}")
    check("MANUAL: elevator flips with direction", sgn(man_elev) == -sgn(man_elev_f) != 0)
    check("MANUAL: gimbal driven and flips with direction", sgn(man_gim) == -sgn(man_gim_f) != 0)
    sim.set(aux2=1000, pitch=1500, yaw=1500)

    # ---- motor stopped in the deadband: no auto-disarm, gimbal fades ----
    sim.set(thr=1500)
    time.sleep(3.0)
    armed, _ = status(msp)
    r = rf_status(msp)
    check("still armed with the stick parked in the deadband", armed)
    check("motor stopped state, gimbal factor ~0", r["state"] == "motorStopped" and abs(r["vec"]) < 0.05, str(r))

    # ---- failsafe while in reverse: zero thrust, never full reverse ----
    sim.set(thr=1138)
    time.sleep(1.5)
    sim.rc_on = False
    peak = 0.0
    t0 = time.time()
    while time.time() - t0 < 2.5:
        m = sim.motor() or 0.0
        peak = min(peak, m) if time.time() - t0 > 1.2 else peak
        time.sleep(0.02)
    m = sim.motor()
    armed, flags = status(msp)
    print(f"failsafe: motor {m}, armed {armed}, flags {flags_str(flags)}")
    check("failsafe: motor at zero thrust (not reverse)", m is not None and abs(m) < 0.01, f"{m}")
    sim.rc_on = True
    sim.set(thr=1500, aux1=1000)
    time.sleep(1.5)
    armed, _ = status(msp)
    check("disarmed after failsafe / switch off", not armed)

    sim.stop = True
    print()
    print(f"{sum(ok for _, ok in results)}/{len(results)} checks passed")


if __name__ == "__main__":
    {"config": do_config, "test": do_test}[sys.argv[1]]()
