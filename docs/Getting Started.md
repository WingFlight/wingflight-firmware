# Getting Started

Wingflight is flight controller firmware for fixed-wing aircraft: trainers, sport and 3D
aeroplanes, gliders, flying wings and jets. It is a fork of Rotorflight, which is in turn
built on Betaflight 4.3, but it flies wings only. It does not fly multirotors or
helicopters, and nothing in this guide applies to them.

This guide takes a bare flight controller and the aeroplane around it to a trimmed,
flight-ready model in eight stages. The Wingflight Configurator's **Setup journey** tab is
the guided version of exactly these stages: it shows which stage you are in, checks what it
can check for you, and asks you to confirm what only you can see. The full tab tree is one
click away under **All settings**, and everything in this guide can also be done there
directly.

Basic RC knowledge is assumed: how a transmitter works, how to solder, what a servo and an
ESC do. If that is new, spend an evening with an RC forum or a beginners' video series
first.

DISCLAIMER: this document is a helping guide, not an authoritative checklist. We cannot
guarantee the safety or success of your project. Always exercise common sense, critical
thinking and caution.

## Before you start

**Hardware.** Wingflight runs on four boards, all Matek wing controllers: `MATEKF405`,
`MATEKF411`, `MATEKF722` and `MATEKH743`. See [Boards](Boards.md) for what each offers.
You also need a receiver, one servo per control surface, and, unless you are building a
glider, an ESC and motor. Handle the bare board carefully: the accelerometer is a shock
sensor, and a drop onto a bench can damage it before it ever flies.

**Software.** Download the [Wingflight Configurator](https://github.com/WingFlight/wingflight-configurator/releases)
and flash the firmware for your board through its Firmware Flasher tab; see
[Installation](Installation.md). On Windows, if the board is not detected, the driver
fixer linked from the configurator's README installs the correct USB driver.

**Two ways to change a setting.** Almost everything is on a configurator tab. The
[Command Line Interface](Cli.md) covers the rest, and is how forum advice of the form
"set X to Y" is applied. Both change the same settings.

**Propeller off.** Remove the propeller before you connect anything and leave it off until
stage 8 tells you to put it back. For the failsafe test in stage 6, disconnect the motor
from the ESC as well. Read [Safety](Safety.md) now, not later.

## The eight stages

Each stage answers one question, and each has a clear definition of done. The order is not
arbitrary: you cannot watch channels move in stage 4 on a receiver whose pad you have not
assigned in stage 3, and you cannot check a surface direction in stage 5 on a mixer you have
not chosen in stage 2.

| Stage | The question it answers | Counts as done when |
| --- | --- | --- |
| 1 Board | Is the right firmware on it, and does it know which way is up? | Firmware version supported, board alignment applied, accelerometer calibrated and level within tolerance |
| 2 Airframe | What am I flying? | Model type is not Custom, and every commanded axis reaches at least one output in the mixer rules |
| 3 Wiring | What is plugged into which pad? | Every mixer output resolves to a pad, no timer or DMA conflict remains, the receiver pad matches the chosen protocol |
| 4 Link | Does the radio reach the plane, and which stick is which? | Four primary channels observed moving, channel map set, endpoints inside the valid window, an arm switch bound |
| 5 Outputs | Do the surfaces move the right way, by the right amount? | Per-surface direction, centre and travel confirmed against the airframe drawing; throttle range calibrated |
| 6 Safety | What happens when the link drops, or I flip a switch? | Arm mode bound, failsafe procedure and stage timings set, throttle-off on failsafe verified with the motor disconnected |
| 7 Power | Can it read the battery, and will it warn me? | Voltage scale calibrated against a measured pack, current scale set or the sensor explicitly disabled, warning thresholds set |
| 8 Pre-flight | Is it actually safe to throw? | Every physical check acknowledged: surface directions, stabilisation correction direction, centre of gravity, control throws, range check |

The journey shows each stage as *Not started*, *In progress*, *Needs attention*, *Verified*
or *Not applicable*. A stage is verified only while its checks hold right now. Reverse a
servo later and the direction check for that surface, and only that one, drops back to
*Needs attention*.

### Stage 1: Board

*Is the right firmware on it, and does it know which way is up?*

1. Flash the firmware for your exact board with the Firmware Flasher, then connect. The
   **Status** tab shows the target name and firmware version; the configurator warns if the
   version is not one it supports.
2. Mount the board in the fuselage, or at least decide how it will be mounted. If the arrow
   on the board will not point forward and flat, set the **Board alignment** on the
   **Configuration** tab. There is an auto-align wizard there: hold the aeroplane level and
   nose-forward and it works out the rotation for you.
3. Set the aeroplane level on the bench and calibrate the accelerometer on the **Setup**
   tab. The 3D model on that tab should now follow the board when you tilt it: nose up on
   the bench is nose up on screen, left wing down is left wing down. If it is not, the
   alignment is wrong; fix it before going on.

**Done when** the firmware version is supported, the board alignment is applied, and the
accelerometer is calibrated and reads level within tolerance.

### Stage 2: Airframe

*What am I flying?*

Wingflight needs to know the shape of the aeroplane before anything else can be judged
relevant. On the **Mixer** tab choose a model type:

* **Regular airplane**: fuselage, wing, conventional tail. Choose no ailerons, a single
  aileron servo or independent left and right, and elevator only or elevator and rudder.
* **Flying wing** and **Delta wing**: two elevons, optionally a rudder.
* **V-tail airplane**: ailerons as above, two ruddervators mixed from pitch and yaw.
* **Rudder/elevator trainer**: no ailerons, elevator and rudder only.
* **Custom**: write the mixer rule table yourself.

Picking a type generates the mixer rules and draws the aeroplane, with each surface labelled
by the output that drives it. The default layout is S1 left aileron, S2 right aileron, S3
elevator, S4 rudder and M1 throttle. You can look at, and later edit, the generated rules in
the rule table on the same tab. The old [Mixer](Mixer.md) chapter describes the rule table
concepts; ignore its multirotor mixer list.

**Done when** the model type is not Custom and every commanded axis (roll, pitch, yaw,
throttle) reaches at least one output in the mixer rules.

### Stage 3: Wiring

*What is plugged into which pad?*

Now you know how many outputs you need. Every Matek board has pads labelled `S1`, `S2`, ...
that can drive a servo or an ESC, one or more UARTs for the receiver and other serial
devices, and pads for battery voltage, current and an LED strip. [Boards](Boards.md) lists
how many of each your board has and which UART a serial receiver is expected on by default
(UART2 on the F405 and F722, UART1 on the F411, UART6 on the H743).

1. In the journey, the Wiring stage draws your board. Click a pad to say what is soldered
   to it; only functions that pad can actually do are offered, and timer and DMA clashes are
   resolved or refused before anything is written. Nothing is written until you accept the
   diff, and a full configuration dump is saved first.
2. Without the journey, the same assignments are the CLI `resource`, `timer` and `dma`
   commands, see [Custom Board Configuration](Custom%20Board%20Configuration.md). The
   default assignments of each board are usually right: S1 to S4 are already motor/servo
   outputs, and the default receiver UART is already set up for a serial receiver.
3. Solder. Servos and the ESC signal wire to the `S` pads the mixer expects, the receiver
   to the TX/RX pads of the UART you chose. Power the servos from the board's BEC rail or a
   separate BEC as the board's manual says, never from a USB port.
4. Assign the receiver's UART to **Serial RX** on the **Configuration → Ports** tab, and set
   the receiver protocol on the **Receiver** tab. Read the [Receiver](Rx.md) chapter for
   which pad each protocol needs (some are inverted, some are half-duplex on a single wire).

**Done when** every mixer output resolves to a pad, no timer or DMA conflict remains, and
the receiver pad matches the chosen protocol.

### Stage 4: Link

*Does the radio reach the plane, and which stick is which?*

1. Set up the transmitter model: at least four channels (aileron, elevator, throttle,
   rudder), preferably more, with two or three switches on channels 5 and up for arming and
   flight modes. Bind the receiver.
2. Power the receiver (USB power to the board is usually enough for the receiver alone) and
   open the **Receiver** tab. Move each stick and watch the bars. All four primary channels
   must move, and each must move the bar you expect. Fix the **channel map** until they do.
3. Check the range. Each channel should rest at about 1500 in the centre and reach about
   1000 and 2000 at the ends of stick travel. Adjust the transmitter's endpoints and
   sub-trims to get there. The valid pulse window the flight controller accepts is on the
   **Failsafe** tab (`rx_min_usec` / `rx_max_usec`); anything outside it is treated as a
   lost link, so make sure your endpoints stay inside it with margin.
4. Bind an **ARM** switch on the **Modes** tab. See [Modes](Modes.md). Do not arm yet.
5. Optional: a second receiver on another UART as a **backup RX input** (CRSF, EXBUS or
   FBUS). The Receiver tab shows both receivers' channel values side by side once the port
   is assigned.

**Done when** all four primary channels have been observed moving, the channel map is set,
the endpoints sit inside the valid window and an arm switch is bound.

### Stage 5: Outputs

*Do the surfaces move the right way, by the right amount?*

Propeller off. Battery connected (the servos need more power than USB provides), board
disarmed.

1. On the **Servos** tab, or by clicking a surface on the airframe drawing in the journey,
   set for each surface: **Reverse** if it moves the wrong way, **Mid** so the surface is
   neutral with the stick and trims centred, and **Min**, **Max** and **Rate** so full stick
   gives the throw the aeroplane's manual asks for without the servo hitting a mechanical
   stop and buzzing. Roll left: the left aileron rises and the right aileron drops. Pull
   back: the elevator rises. Rudder left: the rudder swings left. On a flying wing both
   elevons rise on pull-back and move opposite ways on roll.
2. On the **Motors** tab, choose the ESC protocol your ESC speaks (PWM is the safe default
   for aeroplane ESCs; DShot and the others need an ESC that supports them). Set minimum
   throttle, maximum throttle and minimum command as the ESC's manual describes, and
   calibrate the ESC's throttle range if the ESC needs it, following its manual.

**Done when** direction, centre and travel of every surface have been confirmed against the
airframe drawing and the throttle range is calibrated.

### Stage 6: Safety

*What happens when the link drops, or I flip a switch?*

This stage is why the propeller is still off. Read [Failsafe](Failsafe.md) in full before
setting anything.

1. Confirm the **ARM** switch from stage 4 arms and disarms on the **Modes** tab, and that
   the board refuses to arm when it should (a flashing status LED gives the reason, see
   [Controls](Controls.md)). Consider a **PREARM** switch as well.
2. On the **Failsafe** tab set the valid pulse range and the stage 1 channel fallback (Auto,
   Hold or Set a value) for each channel. The stage 2 procedure (auto-land, drop, or GPS
   rescue where a GPS is fitted) and its timings (`failsafe_delay`, `failsafe_off_delay`,
   `failsafe_throttle`) are set in the [CLI](Cli.md); `get failsafe` lists them all.
   Whatever procedure you choose must bring the throttle to off.
3. Test it. **Disconnect the motor from the ESC**, connect the battery, arm, raise the
   throttle, and switch the transmitter off. Watch the **Receiver** tab and the ESC: the
   throttle output must drop to off within the delay you set and stay there, and the board
   must not re-arm on its own when the transmitter comes back. Repeat with the receiver's
   own failsafe (see its manual), because both layers must agree.

**Done when** the arm mode is bound, the failsafe procedure and stage timings are set, and
throttle-off on failsafe has been verified with the motor disconnected.

### Stage 7: Power

*Can it read the battery, and will it warn me?*

On the **Power** tab, see [Battery](Battery.md):

1. Set the **voltage meter source** to the board's ADC. Connect a battery, measure it with a
   multimeter and adjust the voltage scale until the tab reads the same. Check the detected
   cell count.
2. Set the **current meter source** if the board or ESC has a current sensor and adjust its
   scale against a known load; otherwise set it to none so that nothing downstream believes
   a reading that does not exist.
3. Set the minimum, warning and maximum cell voltages. The warning voltage drives the
   [buzzer](Buzzer.md), the LED strip and telemetry alarms.

**Done when** the voltage scale has been calibrated against a measured pack, the current
scale is set or the sensor explicitly disabled, and warning thresholds are set.

### Stage 8: Pre-flight

*Is it actually safe to throw?*

Nothing here can be read from the board. The journey asks you to acknowledge each check,
and remembers the acknowledgment together with the settings it depends on, so a later change
to those settings asks you again.

1. **Surface directions**, once more, with the finished aeroplane in your hands and the
   transmitter in your other hand.
2. **Stabilisation correction direction.** Arm with the propeller still off, choose a
   stabilised mode (ANGLE or HORIZON is easiest to see), and roll the aeroplane to the
   right by hand. The left aileron must drop and the right must rise, i.e. the surfaces move
   to bring the wing back to level. Pitch nose-up by hand: the elevator must drop. Yaw
   nose-right: the rudder must swing left. If any surface moves the wrong way, do not fly;
   the board alignment or that surface's mixer rule is wrong. **PASSTHROUGH** is the mode
   that bypasses stabilisation entirely and behaves like a plain receiver; it is a good
   emergency switch to have bound as well.
3. **Centre of gravity** where the aeroplane's manual says, with the flight battery in its
   flying position.
4. **Control throws** as the manual says, measured at the trailing edge.
5. **Range check** with the transmitter in range-check mode, walking away from the model
   while a helper watches the surfaces.

Only now fit the propeller. Check it is the right way round and tight.

**Done when** every physical check has been acknowledged: surface directions, stabilisation
correction direction, centre of gravity, control throws, range check.

## After setup: tuning

Stage 8 is where the aeroplane stops being a configuration exercise. What follows is
tuning, and it is deliberately not a ninth stage: it happens at the field, over several
flights, and each item has a symptom that tells you whether you need it at all.

| Tune | When | Symptom that says you need it | Where |
| --- | --- | --- | --- |
| Mechanical trim | Before flight 1 | Surfaces not neutral with sticks and trims centred | Bench |
| Sub-trim and travel | Before flight 1 | A surface hits its mechanical stop before full stick, or binds | Bench |
| Rates and expo | Flight 1 | Too twitchy or too lazy around centre | Field |
| Roll and pitch gains | Flights 2-4 | Slow to hold an attitude, or a fast wobble after a correction | Field |
| Yaw and coordination | Flights 2-4 | Nose slides in turns, or hunts in level flight | Field |
| Filters | After a blackbox log | Warm motor, buzzing servos, gyro noise in the log | Field |
| Re-verify failsafe | After any change above | Always: it is the check most often invalidated | Bench |

Where these live: rates and expo on the **Rates** tab, gains on the **Profiles** tab
(with throttle-based gain attenuation for the extra surface authority you get in prop
wash), filters on the **Gyro** tab, and switch-assigned in-flight tuning on the
**Adjustments** tab. Fly the first flight in a self-levelling mode with moderate throws,
trim, land, and change one thing at a time.

## Further reading

The chapters below are inherited from Rotorflight and Betaflight and still say so in
places, but their content applies unless noted above.

* [Boards](Boards.md): the four supported boards
* [Installation](Installation.md): flashing and upgrading
* [Receiver](Rx.md): receiver types, wiring and protocols
* [Mixer](Mixer.md): the mixer rule table
* [Battery](Battery.md): voltage and current monitoring
* [Failsafe](Failsafe.md): stages, procedures and how to test them
* [Modes](Modes.md): flight and function modes
* [Controls](Controls.md): arming, stick commands and arming-disabled reasons
* [Safety](Safety.md)
* [CLI](Cli.md): every setting by name
* [Blackbox](Blackbox.md), [Telemetry](Telemetry.md), [GPS](Gps.md), [LED strip](LedStrip.md)
