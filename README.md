# Syndex - ODrive CAN Control System

## Library Architecture

This project uses three key header files to communicate with ODrive controllers via CAN bus:

### **FlexCAN_T4.h**

- **Purpose**: Low-level CAN bus hardware driver for Teensy 4.x microcontrollers
- **From**: External library ([tonton81/FlexCAN_T4](https://github.com/tonton81/FlexCAN_T4.git))
- **What it does**: Provides direct hardware interface to the Teensy's CAN controller (CAN1, CAN2, CAN3)
- **Key features**: Mailbox management, FIFO buffers, baudrate settings, interrupt handlers
- **In code**: `FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> can_intf;` creates the actual hardware interface

### **ODriveCAN.h**

- **Purpose**: ODrive's official CAN protocol library
- **From**: External library ([odriverobotics/ODriveArduino](https://github.com/odriverobotics/ODriveArduino.git))
- **What it does**: Implements ODrive-specific CAN messages and commands (heartbeat, encoder feedback, set state, clear errors, etc.)
- **Key features**: Protocol definitions, message parsing/encoding, ODrive state management
- **In your code**: `ODriveCAN odrv0` and `odrv1` are controller objects that speak the ODrive protocol

### **ODriveFlexCAN.hpp**

- **Purpose**: Adapter/glue layer between FlexCAN_T4 and ODriveCAN
- **From**: Part of ODriveArduino library or custom adapter
- **What it does**: Wraps the FlexCAN_T4 interface to work with ODriveCAN's expected interface (provides `wrap_can_intf()` function)
- **Why needed**: ODriveCAN is hardware-agnostic; this makes it compatible with Teensy's specific CAN driver

### Communication Flow

```
FlexCAN_T4 (hardware) → ODriveFlexCAN (adapter) → ODriveCAN (protocol)
```

## Dependencies

Defined in `platformio.ini`:

- `FlexCAN_T4` - Teensy CAN bus driver
- `ODriveArduino` - ODrive CAN protocol implementation
- `TCA9548` - I2C multiplexer support
