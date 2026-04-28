
# STRIVE Robotic Arm — Syndex Firmware Overview

> Last updated: 2026-04-24 | Branch: `MVP_2026` | Firmware: Teensy 4.1 / PlatformIO | Software: Unity C#


## What This System Is

STRIVE is a 7-DOF surgical robotic arm for teleoperation and teaching. It uses **admittance control** to feel weightless when a human applies force to it — the arm senses force through motor current and computes a virtual velocity response, making it compliant and easy to guide by hand.

---

## Hardware & Communication Topology

```
[Operator] ──→ [Arm links and joints]
			 │
	 ┌─────────┴─────────┐
 Joints 0-2           Joints 3-6
 (ODrive Micro)       (passive, no motor)
	 │                     │
 CAN bus 250kbps      I2C mux TCA9548A
	 │                     │
[Teensy 4.1] ◄──────────────┘
 (syndex firmware)
	 │
 USB Serial
	 │
[Windows PC]
 (Unity app)
```

| Joint | Label    | Drive                | Sensor                        |
|-------|----------|----------------------|-------------------------------|
| 0     | ROTATE   | ODrive 0 (CAN node0) | ODrive onboard encoder        |
| 1     | REACH    | ODrive 1 (CAN node1) | ODrive onboard encoder        |
| 2     | LIFT     | ODrive 2 (CAN node2) | ODrive onboard encoder        |
| 3     | EXT_CH3  | None                 | AS5600 on I2C mux channel 3   |
| 4     | EXT_CH2  | None                 | AS5600 on I2C mux channel 2   |
| 5     | EXT_CH1  | None                 | AS5600 on I2C mux channel 1   |
| 6     | EXT_CH0  | None                 | AS5600 on I2C mux channel 0   |

---

## Firmware (syndex — Teensy 4.1)

Bare-metal C++ application running on Teensy 4.1, built with PlatformIO.

### Key responsibilities
- Run the global state machine for all system behavior
- Communicate with 3 ODrive Micro controllers over CAN (FlexCAN_T4)
- Read 4 external AS5600 encoders over I2C (TCA9548A mux)
- Run admittance control loop at ~200 Hz
- Send joint telemetry to Unity PC over USB serial at 200 Hz (joint data) and 10 Hz (status)
- Process commands from Unity: ping, confirm home, return to home, e-stop

### Source files

| File                      | Responsibility                                                        |
|---------------------------|-----------------------------------------------------------------------|
| `src/main.cpp`            | Arduino `setup()`/`loop()`: hardware init, CAN events, USB, state machine |
| `src/states.cpp`          | Global state machine: BOOTUP → ... → ERROR_STATE                     |
| `src/joint.cpp`           | Joint struct array, angle reading, homing, `isHomed()`                |
| `src/odrive.cpp`          | ODrive init, control mode, emergency stop                             |
| `src/admittance.cpp`      | Physics model: torque estimation, admittance integrator               |
| `src/admittance_controller.cpp` | Per-joint admittance loop, velocity commands                     |
| `src/comms.cpp`           | CAN bus, I2C encoder reading, ODrive CAN callbacks                    |
| `src/USB.cpp`             | USB serial packet framing, CRC-16, command dispatch                   |

---

## Library Architecture & Dependencies

This project uses three key header files to communicate with ODrive controllers via CAN bus:

### FlexCAN_T4.h
- Low-level CAN bus driver for Teensy 4.x ([tonton81/FlexCAN_T4](https://github.com/tonton81/FlexCAN_T4.git))
- Direct hardware interface to Teensy's CAN controller
- Mailbox management, FIFO buffers, baudrate, interrupts
- Example: `FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can_intf;`

### ODriveCAN.h
- ODrive's official CAN protocol library ([odriverobotics/ODriveArduino](https://github.com/odriverobotics/ODriveArduino.git))
- Implements ODrive-specific CAN messages and commands
- Protocol definitions, message parsing, ODrive state management
- Example: `ODriveCAN odrv0`, `odrv1`

### ODriveFlexCAN.hpp
- Adapter between FlexCAN_T4 and ODriveCAN
- Wraps FlexCAN_T4 for ODriveCAN's interface (`wrap_can_intf()`)

#### Communication Flow
```
FlexCAN_T4 (hardware) → ODriveFlexCAN (adapter) → ODriveCAN (protocol)
```

#### PlatformIO dependencies
- `FlexCAN_T4` - Teensy CAN bus driver
- `ODriveArduino` - ODrive CAN protocol
- `TCA9548` - I2C multiplexer support

---

## ODrive Configuration

**Motor type is NOT configurable via CAN.** Set it via the ODrive web interface before deployment:
1. Connect to ODrive web interface (`<odrive-ip>:8080`)
2. Go to **Motor Configuration**
3. Set **Motor Type** to `PMSM CURRENT CONTROL` (0)
4. Save and reboot
Once set, the motor type persists. The `configureOdrive()` function handles encoder offset calibration and runtime setup via CAN.

---

## Unity Software (STRIVE Software — C#)

Runs on Windows PC:
- Discovers/connects to Teensy via USB serial
- Sends `CMD_PING` to wake firmware from IDLE
- Receives joint telemetry at 200 Hz for 3D arm animation
- Computes forward kinematics from 7 joint angles
- Sends `CMD_CONFIRM_HOME` (**S** key) and `CMD_START_HOMING` (**H** key)

---

## USB Communication Protocol

Packets are framed with sync bytes `0x7F 0xFE`, followed by a 1-byte type, 1-byte payload length, variable-length payload, and a 2-byte CRC-16/ARC checksum. Float values are compressed to 16-bit unsigned integers mapped over the ±180° range.

```
[0x7F] [0xFE] [TYPE] [LENGTH] [PAYLOAD...] [CRC_LO] [CRC_HI]
```

### Key packet types

| Direction      | Type              | Code   | Description                                 |
|----------------|-------------------|--------|---------------------------------------------|
| PC → Teensy    | `CMD_PING`        | 0x01   | Connection handshake                        |
| Teensy → PC    | `RESP_PONG`       | 0x81   | Response to ping                            |
| PC → Teensy    | `CMD_CONFIRM_HOME`| 0x09   | Operator confirms arm is at home position   |
| PC → Teensy    | `CMD_START_HOMING`| 0x06   | Request automated return to home            |
| PC → Teensy    | `CMD_ESTOP`       | 0x08   | Emergency stop                              |
| Teensy → PC    | `TELEM_JOINT_DATA`| 0x84   | 7× (angle + velocity) as float16 pairs, 200Hz|
| Teensy → PC    | `TELEM_STATUS`    | 0x85   | System health summary, 10Hz                 |
| Teensy → PC    | `LOG_MESSAGE`     | 0x86   | Debug string                                |
| Teensy → PC    | `ERROR_MESSAGE`   | 0xF0   | Fault string                                |

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

**BOOTUP** — Hardware verification: ODrives online via CAN, I2C mux, AS5600 encoders, LEDs. → IDLE on success, ERROR_STATE on failure.

**IDLE** — Waits for `CMD_PING` from Unity. Drives powered, velocity mode at zero. Safe to remain indefinitely.

**CONNECTED** — Verifies all comms (USB, CAN, I2C). Routes to HOMING (if not homed) or READY (if re-connecting).

**HOMING** — Arm is back-driveable (velocity, soft gains). Operator positions arm at home pose. **S** in Unity sends `CMD_CONFIRM_HOME`, firmware latches current position as zero. → READY.

**READY** — Normal operation. Sub-states:
| Sub-state      | Description                                              |
|---------------|----------------------------------------------------------|
| ADMITTANCE    | Admittance control loop at ~200 Hz; arm is compliant     |
| MOVING_TO_HOME| Arm returns to home under position control (**H** key)   |
| RESTORING     | Restores velocity control, re-calibrates admittance bias |

**POWERINGOFF** — Graceful shutdown: motors idled, LEDs off.

**ERROR_STATE** — Latched fault. Motors idled, error LED on. Requires reset to exit.

---

## Homing System

Homing establishes the zero-position reference for the arm. Required on every power-on.

### Procedure
1. System enters HOMING. Drives in velocity mode, soft gains — arm easy to move.
2. Operator guides arm to home pose.
3. Operator presses **S** in Unity → sends `CMD_CONFIRM_HOME`.
4. Firmware calls `confirmHome()`: sets current encoder as zero, re-arms drives, marks joints as homed.
5. System → READY.

### Return to home (after homing)
Operator presses **H** in Unity, sends `CMD_START_HOMING`. Firmware switches to position control, drives all ODrive joints to `home_pos = 0.0f`. Once all joints are within 0.02 turns of home (or after timeout), velocity/admittance mode is restored.

**Design note:** Use ODrive's Custom User Reference Frame (write `0` to `pos_estimate`) for zeroing, not `setAbsolutePosition()`. The latter causes a position discontinuity and velocity fault. Custom Reference Frame sets a smooth offset.

For safety, set `axis.controller.config.absolute_setpoints = true` in ODrive flash. This prevents position commands until `pos_estimate` is written after boot.

---

## ODrive Control

Each ODrive Micro runs one joint motor over CAN at 250 kbps, using `ODriveCAN` C++ library.

### Control modes

| Mode                  | When active                                 | How commanded                        |
|-----------------------|---------------------------------------------|--------------------------------------|
| Velocity + passthrough| HOMING, READY/ADMITTANCE, READY/RESTORING   | `setVelocity(vel_turns_per_s, 0.0f)` |
| Position + passthrough| READY/MOVING_TO_HOME only                   | `setPosition(home_pos, 0.0f, 0.0f)`  |

### Velocity control parameters (set once at init)

| Parameter         | Value         | Purpose                                 |
|-------------------|--------------|-----------------------------------------|
| `vel_gain`        | 0.001        | Very soft — joint is easy to backdrive   |
| `vel_int_gain`    | 0.0          | No integral term during admittance       |
| `vel_limit`       | 50 turns/s   | Safety clamp on motor velocity           |
| `current_soft_max`| 6 A          | Implicit torque / force limit            |

### Initialization sequence (per ODrive, at power-on)
1. Register CAN callbacks for heartbeat, encoder, current
2. Wait for heartbeat (ODrive powered/responding)
3. Clear stale errors
4. Enter closed-loop control
5. Set velocity control mode with soft gains

---

## Admittance Control

Admittance control makes the arm feel weightless by sensing operator force and computing a velocity response. Runs only in READY/ADMITTANCE at ~200 Hz for joints 0–2.

### How it works

```
Iq_Measured (CAN) → subtract iq_bias → × Kt × gear_ratio → tau_ext
									│
				    if |tau_ext| < 0.10 Nm → zero (deadband)
									│
				    M·dv/dt = tau_ext - B·v  (admittance model)
									│
				    vel_cmd = v / (2π × gear_ratio)
									│
				    setVelocity(vel_cmd) → ODrive
```

**`iq_bias`** is calibrated each time READY is entered or after return-to-home. The arm is sampled at rest to measure baseline current. Only deviations above this are treated as human-applied force.

### Tunable parameters

| Parameter      | Value         | Effect                                         |
|---------------|--------------|------------------------------------------------|
| M (mass)      | 0.03 kg·m²   | Higher = arm feels heavier, slower response     |
| B (damping)   | 0.05 N·m·s/rad| Higher = arm decelerates faster after force     |
| Deadband      | 0.10 Nm      | Minimum force before arm moves (noise filter)   |
| Max velocity  | 1.0 turns/s  | Safety clamp on admittance output               |
| Gear ratio    | 5.0          | Used in torque estimation (verify hardware)     |
| Motor Kt      | 0.087 N·m/A  | Used in torque estimation (verify hardware)     |

---

## Known Issues

| # | Description | Root cause |
|---|-------------|------------|
| 1 | Joint 0 returns to home during admittance | ODrive re-arms after `confirmHome()` without explicitly setting velocity mode; may re-enter position mode |
| 2 | Joints 1 and 2 do not move | Drives faulted due to issue #3; no current telemetry received; admittance loop skips joints with no data |
| 3 | ODrive 1 velocity limit error | `setAbsolutePosition(0.0f)` creates a position step, causing an apparent velocity spike that exceeds the limit |
| 4 | Redundant re-arm in return-to-home | `setControllerMode()` does not disarm the drive; the extra `clearErrors()` + `setState()` is unnecessary and can itself trigger faults |
| 5 | Admittance state not reset on READY re-entry | `static` local variables in the READY case persist across state exits; `resetAdmittanceController()` is skipped on re-entry |
| 6 | External encoder zeros never set during homing | `zeroOffset[]` in `comms.cpp` is never updated; joints 3–6 report absolute position from magnetic zero, not from the operator-defined home |
| 7 | Gear ratio and Kt defined in two places with conflicting values | `admittance.h` defines `GEAR_RATIO = 23.0` and `TORQUE_CONST = 100.0`; controller uses `5.0f` and `0.087f` |
