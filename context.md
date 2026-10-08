# STRIVE — Simulated Arm Mechanics Context File
>
> Use this document when tuning KinematicsFollower, newGrabScript, or any arm/gripper simulation parameter.

---

## 1. Project Overview

**STRIVE** is a robotic surgical-tool training simulator. A user physically operates a custom 6-DOF robotic arm handle; the Unity simulation mirrors that arm in real time so trainees learn instrument control before touching a real patient.

Key UX goals:

- The simulation arm should feel like a 1:1 shadow of the physical arm — no noticeable lag, no drift
- Gripper open/close should feel snappy and match the physical trigger feel
- Objects in the scene should respond to the gripper as if being physically grabbed
- The visual arm must stay visually attached to its base; parts must never fly off or jitter

---

## 2. Physical Hardware

| Component | Detail |
|---|---|
| Microcontroller | Teensy 4.1 |
| Motor controllers | ODrive (controls J0, J1, J2) |
| Wrist encoders | Quadrature encoders (J3, J4, J5) |
| Gripper trigger | Analog pin 40, `INPUT` mode, active-HIGH |
| USB baud rate | 115200 |

### Trigger specifics

- Read via `analogRead` on Teensy (12-bit ADC, max 4096)
- Unpressed resting value: ~2120 counts
- Fully pressed value: ~4040 counts
- Threshold for "pressed": ≥ 3000 counts
- Active-low in hardware → firmware inverts to active-high before sending

---

## 3. Joint Mapping (Firmware → Unity)

| Index | Name | Hardware | Axis of motion | Unity field |
|---|---|---|---|---|
| J0 | ROTATE | ODrive 0 | Base yaw — sweeps arm left/right | `pp.J0` |
| J1 | REACH | ODrive 1 | Forward/backward extension | `pp.J1` |
| J2 | LIFT | ODrive 2 | Elevation — raises/lowers arm | `pp.J2` |
| J3 | PITCH | Encoder 0 | Wrist pitch — first bend (up/down) | `pp.J3` |
| J4 | YAW | Encoder 1 | Wrist yaw — second bend (left/right) | `pp.J4` |
| J5 | ROLL | Encoder 2 | Shaft spin around its own axis | `pp.J5` |

### ⚠ Raw value units

`J0_AngleRaw` … `J5_AngleRaw` come straight out of firmware with **no unit conversion**:

- **J0, J1, J2 (ODrive)**: values are in **turns** (1.0 = one full revolution = 360°). A value of 0.25 = 90°.
- **J3, J4, J5 (Encoders)**: units depend on firmware processing — currently sent as raw encoder counts or normalised values. Validate by moving each joint manually and logging the delta.

**To convert ODrive turns to degrees in Unity:** multiply by 360.  
Until the firmware confirms units, treat all J values as "unknown scale" and dial in multipliers empirically.

---

## 4. USB Packet Protocol

```
[0x7F][0xFE] [type][len] [payload...] [CRC_lo][CRC_hi]
```

- Sync bytes: `0x7F 0xFE` (stripped before reaching PacketProcessor)
- CRC: CRC-16/ARC, polynomial `0xA001`, seeded 0x0000, computed over type + len + payload

### Relevant packet types

| Type | Hex | Direction | Description |
|---|---|---|---|
| CMD_PING | 0x01 | Host→Device | Heartbeat request |
| CMD_REQUEST_TELEM | 0x05 | Host→Device | Request one joint data snapshot |
| CMD_START_HOMING | 0x06 | Host→Device | Arm returns to home position |
| CMD_CONFIRM_HOME | 0x09 | Host→Device | Operator confirms arm is at home; firmware latches encoder zero |
| RESP_PONG | 0x81 | Device→Host | Response to ping |
| TELEM_JOINT_DATA | 0x84 | Device→Host | 52-byte joint snapshot (see below) |
| TELEM_STATUS | 0x85 | Device→Host | 10-byte arm status (streamed automatically) |

### TELEM_JOINT_DATA payload layout (52 bytes, Pack=1)

```
float J0_AngleRaw,  float J0_VelRaw    // 8 bytes
float J1_AngleRaw,  float J1_VelRaw    // 8 bytes
float J2_AngleRaw,  float J2_VelRaw    // 8 bytes
float J3_AngleRaw,  float J3_VelRaw    // 8 bytes
float J4_AngleRaw,  float J4_VelRaw    // 8 bytes
float J5_AngleRaw,  float J5_VelRaw    // 8 bytes
uint8_t TriggerPressed                 // 1 byte
uint8_t Reserved[3]                    // 3 bytes (C++ tail padding)
```

Total: 52 bytes. `Marshal.SizeOf<Payload_TelemJointData>()` must equal 52 — if it doesn't, `JointData.HasValue` will always be false.

### Telemetry rate

PacketProcessor sends `CMD_REQUEST_TELEM` every **20 ms (50 Hz)** from `Update()`.  
Adjust `TelemRequestInterval` in `PacketProcessor.cs` to change this.

---

## 5. Unity Scene Architecture

### Persistent singletons (DontDestroyOnLoad)

| GameObject | Script | Role |
|---|---|---|
| USBManager | `USB.USBManager` | Owns serial port + background read/write tasks |
| PacketProcessor | `PacketProcessor.PacketProcessor` | Parses packets, exposes J0–J5 + TriggerPressed |

Both survive scene transitions. Duplicates in new scenes are destroyed in `Awake()`.

### ToolController hierarchy

```
ToolController                       ← KinematicsFollower + (MouseFollower — DISABLE for live data)
  └─ HugoInspoToolBase
       └─ HugoInspoToolRotator
            └─ rollPivot             ← Roll Transform (KinematicsFollower)   — local X axis spin
                 └─ HugoInspoToolFirstGear
                      └─ pitchPivot  ← Pitch Transform (KinematicsFollower)  — local Z axis bend
                           └─ HugoInspoToolSecondGear
                                └─ YawPivot     ← Yaw Transform (KinematicsFollower) — local Y axis bend
                                     └─ yawGear
                                          ├─ Jaw1_Pivot
                                          └─ Jaw2_Pivot
                                └─ grabber      ← newGrabScript
```

> ⚠ **MouseFollower must be disabled** on ToolController when using live joint data — it fights KinematicsFollower for `transform.position` every frame.

---

## 6. KinematicsFollower — Parameters & Tuning

Script: `Assets/KinematicsFollower.cs`  
Lives on: `ToolController`

### Arm (root transform) parameters

| Parameter | Current value | What it does | Tuning note |
|---|---|---|---|
| `forwardMultiplier` | 0.005 | How far 1 unit of J1(REACH) moves the tool forward in world space | If REACH barely moves the arm, increase. If it overshoots, decrease. Since J1 is in ODrive turns, try `0.005 × 360 = 1.8` if you convert to degrees first |
| `baseOffset` | (0, 0, 0) | Static offset from the arm's resting point | Adjust if the tool floats above/below its mount when joints = 0 |
| `rotationGain` | 1 | Multiplier on the arm's yaw+pitch rotation delta | Increase if the visual arm rotates less than the physical arm |
| `positionGain` | 1 | Multiplier on forward/backward reach | Combine with forwardMultiplier for final reach scale |
| `positionLerpSpeed` | 20 | Smoothing speed for position (higher = snappier) | At 50 Hz telem, 10–20 feels natural. Go higher if there's visible lag |
| `rotationLerpSpeed` | 20 | Smoothing speed for rotation | Same as above |
| `positionScaleMultiplier` | 10 | Additional global position scale | Stacks with positionGain — may be redundant; consolidate if confusing |

### Wrist (local rotation) axes

These are baked into code — only change if a joint moves the wrong axis:

| Joint | Euler axis | Rationale |
|---|---|---|
| ROLL (J5) | `Euler(j6, 0, 0)` — local X | Shaft spins around its own forward axis in rollPivot's local space |
| PITCH (J3) | `Euler(0, 0, j4)` — local Z | PitchPivot's coordinate frame has the bend axis on Z |
| YAW (J4) | `Euler(0, j5, 0)` — local Y | YawPivot's coordinate frame has the bend axis on Y |

If a joint bends the **wrong direction**, negate the angle (e.g. `Euler(-j4, 0, 0)`).  
If a joint bends on the **wrong axis**, change X/Y/Z in the Euler call.

### Tuning checklist

1. Enable `useMockData` + `animateMockData` — watch each joint sweep to verify all axes look right
2. Set `animationAmplitude = 10` and observe the magnitude — this tells you expected visual range per degree/turn
3. Disable mock data, move physical arm, watch console for joint telemetry logs (every 100 packets)
4. Compare physical angle moved vs. visual movement — adjust `rotationGain` and `forwardMultiplier`

---

## 7. Gripper — newGrabScript Parameters & Tuning

Script: `Assets/newGrabScript.cs`  
Lives on: `grabber` (child of HugoInspoToolSecondGear)

### Jaw animation

| Parameter | Current value | What it does |
|---|---|---|
| `jaw1` / `jaw2` | Jaw1_Pivot / Jaw2_Pivot | The pivot transforms that rotate to open/close |
| `jaw1OpenRotation` | (0, 40, 0) | Jaw 1 local rotation when NOT triggered (resting open) |
| `jaw1ClosedRotation` | (0, 0, 0) | Jaw 1 local rotation when triggered (closed/grabbing) |
| `jaw2OpenRotation` | (0, −40, 0) | Jaw 2 local rotation when NOT triggered |
| `jaw2ClosedRotation` | (0, 0, 0) | Jaw 2 local rotation when triggered |
| `jawAnimationSpeed` | 15 | Lerp speed for jaw open/close — higher = snappier |
| `jawMaxOpenWidth` | 0.5 | Max world-space width used to prevent clipping through held objects |
| `jawPadding` | 0.2 | Extra gap to keep between jaw and held object surface |

### Grab physics

| Parameter | Current value | What it does |
|---|---|---|
| `positionStiffness` | 50 | How strongly grabbed object tracks gripper position |
| `rotationStiffness` | 50 | How strongly grabbed object tracks gripper rotation |
| `maxVelocity` | 20 | Caps velocity so grabbed objects don't explode |
| `maxAngularVelocity` | 20 | Caps angular velocity |
| `snapToCenter` | false | If true, snaps object to gripper center on grab |

### Trigger input sources (priority order)

1. `mockTrigger` inspector checkbox (editor testing)
2. `PacketProcessor.PacketProcessor.Instance.TriggerPressed` (physical hardware, volatile bool)
3. `Mouse.current.leftButton.isPressed` (mouse fallback for editor testing)

### Tuning checklist

1. Check `mockTrigger` in Play mode — jaws must animate. If they don't, jaw pivot transforms are wrong.
2. Uncheck mock, press physical trigger — same result expected.
3. If jaws clip through objects, increase `jawPadding`.
4. If grabbed objects jitter/shake, lower `positionStiffness` and `rotationStiffness`.
5. If grabbed objects lag behind the gripper, raise stiffness values.

---

## 8. Command Intent Map

| User action | Command sent | When |
|---|---|---|
| Press physical trigger | *(gripper open/close only — no USB command)* | Handled purely in Unity via TriggerPressed |
| Press "Confirm Home" UI button | `CMD_CONFIRM_HOME (0x09)` | Operator confirms arm is physically at home pose; firmware latches encoder zero |
| Exit simulation (exit button or lesson complete) | `CMD_START_HOMING (0x06)` | Arm autonomously returns to home position |
| Ping (auto on connect) | `CMD_PING (0x01)` | Startup handshake |
| Telem request (auto 50 Hz) | `CMD_REQUEST_TELEM (0x05)` | Continuous joint data polling |

---

## 9. Known Gotchas & Past Bugs

| Issue | Root cause | Fix |
|---|---|---|
| Tool head flies off on play | HingeJoint + Rigidbody on gear objects fights transform-based KinematicsFollower | Remove Rigidbody + Joint components from all wrist gear objects in Inspector |
| Arm doesn't move despite joint data | `Payload_TelemJointData` had J6 fields (60 bytes) vs firmware 52 bytes → size check failed → `JointData.HasValue` always false | Remove J6_AngleRaw / J6_VelRaw from struct |
| USB drops on trigger press / scene load | Old PacketProcessor loop not cancelled on scene transition; new scene's loop stole pong | Add `DontDestroyOnLoad` to both USBManager and PacketProcessor |
| TriggerPressed never true | Written from background thread, read on main thread without `volatile` keyword | Declare `private volatile bool _triggerPressed` |
| Pitch moves wrong direction | Euler X was used but PitchPivot's bend axis is local Z | `Quaternion.Euler(0, 0, j4)` |
| MouseFollower fights KinematicsFollower | Both set `transform.position` every frame on ToolController | Disable MouseFollower component when using live data |
| J6 compile errors | Firmware removed 7th joint but Unity code still referenced `J6` in multiple places | Removed J6 from PacketProcessor, ForwardKinematics, Payload_SetJointTargets, Payload_TelemJointData |

---

## 10. Files Quick Reference

| File | Purpose |
|---|---|
| `Assets/KinematicsFollower.cs` | Main arm + wrist simulation driver |
| `Assets/newGrabScript.cs` | Gripper jaw animation + physics grab |
| `Assets/MouseFollower.cs` | Mouse-based tool positioning (disable for live data) |
| `Assets/Scripts/PacketProcessor.cs` | USB packet parsing, joint data, telemetry polling |
| `Assets/Scripts/USB/USBManager.cs` | Serial port management, DontDestroyOnLoad singleton |
| `Assets/Scripts/USB/USBStructs.cs` | Packet structs — sizes must exactly match firmware C++ structs |
| `Assets/Scripts/Gripper/ForwardKinematics.cs` | Legacy kinematics (kept for reference/testbench) |
| `Assets/Scripts/UI/ExerciseControllers.cs` | StartHoming on exit, ConfirmHome on button press |
