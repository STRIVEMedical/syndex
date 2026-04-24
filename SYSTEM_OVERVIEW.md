# STRIVE Robotic Arm — System Overview

> Last updated: 2026-04-24 | Branch: `MVP_2026` | Firmware: Teensy 4.1 / PlatformIO | Software: Unity C#

---

## What This System Is

STRIVE is a 7-DOF surgical robotic arm designed for teleoperation and teaching. It uses **admittance control** to feel weightless when a human applies force to it — the arm senses the force through motor current and computes a virtual velocity response, making it compliant and easy to guide by hand.

---

## Hardware Topology

```
[Operator] ─── applies force ──→ [Arm links and joints]
                                         │
                           ┌─────────────┴──────────────┐
                     Joints 0-2                    Joints 3-6
                     (ODrive Micro)                (passive, no motor)
                           │                             │
                      CAN bus 250kbps              I2C mux TCA9548A
                           │                             │
                    [Teensy 4.1]  ◄─────────────────────┘
                    (firmware / syndex project)
                           │
                      USB Serial
                           │
                    [Windows PC]
                    (Unity app / STRIVE Software)
```

**Joints 0–2** are each driven by a BLDC motor through a gearbox, controlled by an ODrive Micro motor controller over CAN bus. Position and velocity are read from the ODrive's onboard encoder. Current (torque) feedback is read via the `Iq_Measured` CAN message.

**Joints 3–6** are passive — no motors, no drive electronics. Position is read from AS5600 12-bit absolute magnetic encoders connected to a TCA9548A I2C multiplexer. No velocity or force information is available for these joints.

| Joint | Label | Drive | Sensor |
|---|---|---|---|
| 0 | ROTATE | ODrive 0 (CAN node 0) | ODrive onboard encoder |
| 1 | REACH | ODrive 1 (CAN node 1) | ODrive onboard encoder |
| 2 | LIFT | ODrive 2 (CAN node 2) | ODrive onboard encoder |
| 3 | EXT_CH3 | None | AS5600 on I2C mux channel 3 |
| 4 | EXT_CH2 | None | AS5600 on I2C mux channel 2 |
| 5 | EXT_CH1 | None | AS5600 on I2C mux channel 1 |
| 6 | EXT_CH0 | None | AS5600 on I2C mux channel 0 |

---

## Firmware (syndex — Teensy 4.1)

The firmware is a bare-metal C++ application running on a Teensy 4.1 microcontroller, built with PlatformIO.

### Key responsibilities
- Run the global state machine that controls all system behavior
- Communicate with 3 ODrive Micro controllers over CAN bus (FlexCAN_T4 library)
- Read 4 external AS5600 encoders over I2C (via TCA9548A mux)
- Run the admittance control loop at ~200 Hz
- Send joint telemetry to the Unity PC over USB serial at 200 Hz (joint data) and 10 Hz (status)
- Process commands from Unity: ping, confirm home, return to home, e-stop

### Source files

| File | Responsibility |
|---|---|
| `src/main.cpp` | Arduino `setup()` and `loop()`: initializes hardware, pumps CAN events, polls USB, calls state machine |
| `src/states.cpp` | Global state machine: BOOTUP → IDLE → CONNECTED → HOMING → READY → POWERINGOFF / ERROR_STATE |
| `src/joint.cpp` | Joint struct array, angle reading (ODrive or AS5600), homing confirmation, `isHomed()` |
| `src/odrive.cpp` | ODrive initialization, control mode functions (`enable_velocity_control`, etc.), emergency stop |
| `src/admittance.cpp` | Physics model: torque estimation from motor current, admittance integrator |
| `src/admittance_controller.cpp` | Per-joint admittance loop: polls current, estimates torque, integrates model, sends velocity commands |
| `src/comms.cpp` | CAN bus setup, I2C encoder reading, ODrive CAN callbacks, multi-turn angle tracking |
| `src/USB.cpp` | USB serial packet framing, CRC-16 verification, command dispatch |

---

## Unity Software (STRIVE Software — C#)

The Unity application runs on a Windows PC. It:
- Discovers and connects to the Teensy via USB serial
- Sends `CMD_PING` on startup to wake the firmware from IDLE
- Receives joint telemetry at 200 Hz and uses it to animate a 3D arm model
- Computes forward kinematics from 7 joint angles using Denavit-Hartenberg parameters
- Sends `CMD_CONFIRM_HOME` when the operator presses **S** (sets current position as home)
- Sends `CMD_START_HOMING` when the operator presses **H** (commands arm to return to saved home)

---

## USB Communication Protocol

Packets are framed with sync bytes `0x7F 0xFE`, followed by a 1-byte type, 1-byte payload length, variable-length payload, and a 2-byte CRC-16/ARC checksum. Float values are compressed to 16-bit unsigned integers mapped over the ±180° range.

```
[0x7F] [0xFE] [TYPE] [LENGTH] [PAYLOAD...] [CRC_LO] [CRC_HI]
```

### Key packet types

| Direction | Type | Code | Description |
|---|---|---|---|
| PC → Teensy | `CMD_PING` | `0x01` | Connection handshake |
| Teensy → PC | `RESP_PONG` | `0x81` | Response to ping |
| PC → Teensy | `CMD_CONFIRM_HOME` | `0x09` | Operator confirms arm is at home position |
| PC → Teensy | `CMD_START_HOMING` | `0x06` | Request automated return to home |
| PC → Teensy | `CMD_ESTOP` | `0x08` | Emergency stop |
| Teensy → PC | `TELEM_JOINT_DATA` | `0x84` | 7× (angle + velocity) as float16 pairs, at 200 Hz |
| Teensy → PC | `TELEM_STATUS` | `0x85` | System health summary, at 10 Hz |
| Teensy → PC | `LOG_MESSAGE` | `0x86` | Debug string |
| Teensy → PC | `ERROR_MESSAGE` | `0xF0` | Fault string |

---

## Global State Machine

```
BOOTUP ──[hardware OK]──→ IDLE ──[CMD_PING]──→ CONNECTED
  │                                                  │
  └──[hardware fail]──→ ERROR              ┌─[not homed]──→ HOMING ──[CMD_CONFIRM_HOME]──┐
                                           │                                              │
                                           └─[already homed]──────────────────────────→ READY ──→ POWERINGOFF
                                                                                          │
                                                                                        ERROR
                                                                                       (latched)
```

### State descriptions

**BOOTUP** — Runs hardware verification: ODrives online via CAN, I2C mux responding, all AS5600 encoders readable, LEDs functional. Transitions to IDLE on success, ERROR_STATE on any failure.

**IDLE** — Waits for `CMD_PING` from the Unity application. Drives are powered and in velocity mode at zero command. Safe to remain here indefinitely.

**CONNECTED** — Verifies all communication layers are healthy (USB active, CAN heartbeats from all 3 ODrives, I2C encoders responding). Routes to HOMING (if arm not yet homed) or READY (if re-connecting after a prior homing).

**HOMING** — Arm is back-driveable (velocity control, soft gains). Operator manually positions the arm at the desired home pose. When the operator presses **S** in Unity, `CMD_CONFIRM_HOME` is sent and the firmware latches the current position as zero. Transitions to READY.

**READY** — Normal operating state. Has three internal sub-states:

| Sub-state | Description |
|---|---|
| `ADMITTANCE` | Admittance control loop running at ~200 Hz; arm is compliant and weightless |
| `MOVING_TO_HOME` | Arm returns to home under position control when operator presses **H** |
| `RESTORING` | One-cycle transition: restores velocity control and re-calibrates admittance bias |

**POWERINGOFF** — Graceful shutdown: all motors idled, all LEDs off.

**ERROR_STATE** — Latched fault state. Motors are immediately idled. Error LED on. Requires device reset to exit.

---

## Homing System

Homing establishes the zero-position reference for the arm. It is required on every power-on.

### Procedure
1. System enters HOMING state. Drives are in velocity mode with deliberately soft gains — the arm is easy to move by hand.
2. Operator manually guides the arm to the desired home pose.
3. Operator presses **S** in Unity → sends `CMD_CONFIRM_HOME`.
4. Firmware calls `confirmHome()`: sets the current encoder position as zero on each ODrive, re-arms the drives, marks joints as homed.
5. System transitions to READY.

### Return to home (after homing)
When the operator presses **H** in Unity, `CMD_START_HOMING` is sent. The firmware switches to position control and drives all three ODrive joints to `home_pos = 0.0f` turns. Once all joints are within 0.02 turns of home (or after a 15-second timeout), velocity/admittance mode is restored.

### Important design note — Custom User Reference Frame
The zeroing step should use ODrive's **Custom User Reference Frame** (write `0` to `pos_estimate`) rather than `setAbsolutePosition()`. The latter resets the encoder count, which causes an instantaneous position discontinuity, an apparent velocity spike, and a velocity limit fault. The Custom Reference Frame approach sets a smooth positional offset with no discontinuity and no fault.

For additional safety, configure `axis.controller.config.absolute_setpoints = true` in ODrive flash. This causes the ODrive to reject all position control commands until `pos_estimate` has been explicitly written after boot — preventing the arm from moving to a stale position if return-to-home is triggered before homing is complete.

---

## ODrive Control

Each ODrive Micro runs one joint motor over CAN bus at 250 kbps, using the `ODriveCAN` C++ library.

### Control modes

| Mode | When active | How commanded |
|---|---|---|
| Velocity + passthrough | All of HOMING, all of READY/ADMITTANCE, all of READY/RESTORING | `setVelocity(vel_turns_per_s, 0.0f)` |
| Position + passthrough | READY/MOVING_TO_HOME only | `setPosition(home_pos, 0.0f, 0.0f)` |

### Velocity control parameters (set once at init)

| Parameter | Value | Purpose |
|---|---|---|
| `vel_gain` | 0.001 | Very soft — joint is easy to backdrive by hand |
| `vel_int_gain` | 0.0 | No integral term during admittance |
| `vel_limit` | 50 turns/s | Safety clamp on motor velocity |
| `current_soft_max` | 6 A | Implicit torque / force limit |

### Initialization sequence (per ODrive, at power-on)
1. Pre-register CAN callbacks for heartbeat, encoder feedback, motor current
2. Wait for heartbeat → ODrive is powered and responding on CAN
3. Clear any stale errors from previous session
4. Enter closed-loop control
5. Set velocity control mode with soft gains

---

## Admittance Control

Admittance control makes the arm feel weightless by sensing the force the operator applies and computing a velocity response. It runs only in the READY/ADMITTANCE sub-state, at ~200 Hz, for joints 0–2 only (joints 3–6 have no motors and cannot be commanded).

### How it works

```
Iq_Measured (CAN) ──→ subtract iq_bias ──→ × Kt × gear_ratio ──→ tau_ext
                                                                      │
                                              if |tau_ext| < 0.10 Nm → zero (deadband)
                                                                      │
                                              M·dv/dt = tau_ext - B·v  (admittance model)
                                                                      │
                                              vel_cmd = v / (2π × gear_ratio)
                                                                      │
                                              setVelocity(vel_cmd) ──→ ODrive
```

**`iq_bias`** is calibrated automatically each time the system enters READY or finishes a return-to-home cycle. The arm is sampled 10 times at rest to measure the baseline current (gravity + friction). Only deviations above this baseline are treated as human-applied force.

### Tunable parameters

| Parameter | Value | Effect |
|---|---|---|
| M (virtual mass) | 0.03 kg·m² | Higher = arm feels heavier, responds more slowly |
| B (virtual damping) | 0.05 N·m·s/rad | Higher = arm decelerates faster after force is removed |
| Deadband | 0.10 Nm | Minimum force before arm starts moving; filters noise and gravity residual |
| Max velocity | 1.0 turns/s | Safety clamp on admittance output |
| Gear ratio | 5.0 (verify against hardware) | Used in torque estimation |
| Motor Kt | 0.087 N·m/A (verify against hardware) | Used in torque estimation |

---

## Known Issues

| # | Description | Root cause |
|---|---|---|
| 1 | Joint 0 returns to home during admittance | ODrive re-arms after `confirmHome()` without explicitly setting velocity mode; may re-enter position mode |
| 2 | Joints 1 and 2 do not move | Drives faulted due to issue #3; no current telemetry received; admittance loop skips joints with no data |
| 3 | ODrive 1 velocity limit error | `setAbsolutePosition(0.0f)` creates a position step, causing an apparent velocity spike that exceeds the limit |
| 4 | Redundant re-arm in return-to-home | `setControllerMode()` does not disarm the drive; the extra `clearErrors()` + `setState()` is unnecessary and can itself trigger faults |
| 5 | Admittance state not reset on READY re-entry | `static` local variables in the READY case persist across state exits; `resetAdmittanceController()` is skipped on re-entry |
| 6 | External encoder zeros never set during homing | `zeroOffset[]` in `comms.cpp` is never updated; joints 3–6 report absolute position from magnetic zero, not from the operator-defined home |
| 7 | Gear ratio and Kt defined in two places with conflicting values | `admittance.h` defines `GEAR_RATIO = 23.0` and `TORQUE_CONST = 100.0`; controller uses `5.0f` and `0.087f` |
