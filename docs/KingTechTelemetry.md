# KingTech turbine ECU telemetry

Wingflight can receive the unsolicited serial telemetry stream produced by a
KingTech turbine ECU directly, without a KingTech telemetry converter or GSU.
The ECU connection is receive-only: Wingflight never sends configuration or
control commands to the ECU.

## Wiring and serial configuration

Connect the KingTech ECU telemetry signal to the RX pin of a free flight
controller UART and connect the grounds. The protocol is idle-high,
19200-baud, 8 data bits, no parity, and 1 stop bit.

Flight-controller inputs are 3.3 V logic. Confirm the ECU signal voltage before
connecting it and use appropriate level shifting if it can exceed the selected
pin's rating. If the disconnected signal floats, an external weak pull-up to
3.3 V may be used where electrically appropriate.

Assign `ESC Sensor` to that UART, then configure:

```text
feature ESC_SENSOR
set esc_sensor_protocol = KINGTECH
set esc_sensor_halfduplex = OFF
save
```

Pin swap remains available for boards where the accessible pad is the UART
TX pin.

Set the first motor's pole count to 2 if Wingflight should treat the reported
turbine RPM as mechanical motor RPM. Wingflight's shared ESC telemetry path
otherwise applies its normal electrical-RPM pole conversion.

## Published fields

| KingTech ECU value | Wingflight ESC telemetry field | Units/handling |
|---|---|---|
| turbine RPM | ESC1 eRPM | direct RPM; use a pole count of 2 for 1:1 mechanical RPM |
| EGT | ESC1 temperature 1 | 0.1 degrees C |
| turbine battery | ESC1 voltage | mV |
| ECU/receiver supply | ESC1 BEC voltage | mV |
| pump power (`Pw`) | ESC1 power | raw `Pw` represented in the existing 0.1% field; `435` appears as 43.5% |
| throttle | ESC1 throttle | 0.1%; `Th:OFF` and `Th:Idle` publish 0% |
| ECU operating state | ESC1 status | KingTech status code, including start stages, running, cooling, and faults |

The compact and extended turbine packets provide RPM, EGT, and pump power.
Extended packets additionally provide throttle, both voltage values, and the
text operating state. Receiver/control packets are checksum validated but do
not overwrite turbine data.

The direct KingTech ECU stream has no confirmed fuel-remaining, fuel-flow,
current, or consumed-capacity value. Wingflight leaves those ESC telemetry
fields at zero rather than estimating fuel use from pump power.

Frames must have the observed `FC C5` header, a supported length (`0x0C`,
`0x30`, or `0x52`), type `0x00`, a valid 16-bit additive checksum, and the
`0x5A` trailer. Invalid frames never update telemetry.
