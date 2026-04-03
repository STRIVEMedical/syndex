# Button Failure Handling Analysis

## Overview
Button failure is detected during **BOOTUP** state and triggers an **ERROR_STATE** transition. The system uses a comprehensive error reporting and recovery mechanism.

---

## 1. Button Configuration

### Pins and IO Mode
[buttons.cpp](src/buttons.cpp) (lines 3-8)
```
powerButton   = Pin 0,  INPUT_PULLUP
autoHoming    = Pin 14, INPUT_PULLUP
triggerButton = Pin 15, INPUT_PULLUP
toolSelect    = Pin 16, INPUT_PULLUP
```

### Button Logic
- **Active-LOW**: LOW (0) = pressed, HIGH (1) = unpressed
- **INPUT_PULLUP**: Internal pull-up resistor keeps pin HIGH when not pressed
- **Initial State**: All buttons initialized to HIGH (unpressed)

### Button Structure [buttons.h](include/buttons.h) (lines 6-11)
```cpp
typedef struct {
    uint8_t pin;              // GPIO Pin #
    uint8_t io;               // INPUT or INPUT_PULLUP
    uint8_t buttonState;      // Current button state
    uint8_t lastButtonState;  // Last state from button, active LOW
} button_t;
```

---

## 2. Button Failure Detection

### Detection During BOOTUP
**[states.cpp:57-71](src/states.cpp#L57-L71) - verifyButton()**

```cpp
bool verifyButton(){
    // Ensure GPIO state used by boot checks is initialized.
    Buttons::setup();

    if (digitalRead(buttonPins::powerButton.pin) == LOW ||
        digitalRead(buttonPins::autoHoming.pin) == LOW ||
        digitalRead(buttonPins::triggerButton.pin) == LOW ||
        digitalRead(buttonPins::toolSelect.pin) == LOW) {
        setError(BUTTON_ERROR);
        return false;
     }
     else {
        return true;
     }
}
```

### Failure Triggers
Any button reading **LOW (pressed)** during boot indicates:
- **Accidental press** during power-on
- **Stuck button** physically jammed
- **Wiring short** on the GPIO pin
- **Pull-up failure** (faulty resistor/connection)

### Called From
**[states.cpp:530](src/states.cpp#L530)** - BOOTUP state checks all hardware:
```cpp
case BOOTUP:
    if (verifyODrive() && verifyI2C() && verifyButton() && verifyLED()) {
        ONToggleLED(&Led::powerLed);
        currState = IDLE;
    }
    else {
        currState = ERROR_STATE;  // Button failure triggers here
    }
    break;
```

---

## 3. Error Handling Flow

### Error State Diagram
```
BOOTUP 
  ↓
[verifyButton() called]
  ├─ ALL BUTTONS HIGH (unpressed)
  │   └─→ IDLE ✓
  └─ ANY BUTTON LOW (pressed)
      └─→ ERROR_STATE ✗
          [BUTTON_ERROR set]
```

### Error Code Definition
**[errors.h:6-13](include/errors.h)**
```cpp
typedef enum {
    NO_ERROR,
    ODRIVE_ERROR,
    I2C_ERROR,
    BUTTON_ERROR,      ← Button failure error code
    LED_ERROR,
    CONNECTION_ERROR,
} errorCode_e;
```

### Error Message Output
**[states.cpp:483-486](src/states.cpp#L483-L486)** - ERROR_STATE handles BUTTON_ERROR:
```cpp
case BUTTON_ERROR:
    Serial.println("ERROR: BUTTON FAILURE");
    break;
```

---

## 4. ERROR_STATE Behavior

### ERROR_STATE Actions
**[states.cpp:600-609](src/states.cpp#L600-L609)**

When button failure triggers ERROR_STATE:
1. **stopODrives()** - Emergency stop all ODrive motors
2. **endPower()** - Cut system power
3. **turnOnErrorLED()** - Turn on error LED (RED)
4. **sendErrMessage()** - Print "ERROR: BUTTON FAILURE" to Serial
5. **Wait for recovery** - Loop in ERROR_STATE until button press

```cpp
case ERROR_STATE:
    stopODrives();          // emergency stop all motors
    endPower();             // cut power
    turnOnErrorLED();       // alert operator visually
    sendErrMessage();       // report specific fault over Serial
    // Wait for operator acknowledgement before attempting recovery
    if (digitalRead(buttonPins::powerButton.pin) == LOW) {
        errorRecovery();    // clear error and restart from BOOTUP
    }
    break;
```

---

## 5. Error Recovery Mechanism

### Recovery Process
**[states.cpp:426-430](src/states.cpp#L426-L430)** - errorRecovery()

```cpp
void errorRecovery(){
    clearError();                  // reset currError to NO_ERROR
    OFFToggleLED(&Led::errorLed);  // turn off error LED
    currState = BOOTUP;            // restart verification from beginning
}
```

### Recovery Trigger
- **Requires**: Operator presses **powerButton** (Pin 0)
- **Action**: Clears error flag, turns off error LED, restarts BOOTUP verification
- **Loop**: System re-runs all hardware checks including `verifyButton()`

### Recovery Flow
```
ERROR_STATE (button failure detected)
  ↓
[Error LED on, error message printed]
  ↓
[Waiting for powerButton press]
  ↓
[Operator presses powerButton]
  ↓
errorRecovery()
  ↓
BOOTUP (restart all verifications)
  ├─ verifyButton() called again
  │   └─ If still LOW → ERROR_STATE again (button still stuck)
  │   └─ If now HIGH → Continue to next check
  └─ [Continue with verifyODrive, verifyI2C, verifyLED]
```

---

## 6. Related Systems

### Button Detection (Unused)
**[buttons.cpp:23-30](src/buttons.cpp#L23-L30)** - buttonDetect()
- Currently unused in main flow
- Prints button press events to Serial
- Contains blocking 100ms delay

### Button Update (Unused)
**[buttons.cpp:32-35](src/buttons.cpp#L32-L35)** - buttonUpdate()
- Updates button state tracking
- Not called in current BOOTUP verification
- Reserved for real-time button event handling

### Event System (Defined but Unused)
**[events.h:3-19](include/events.h)** - hardwareEvent_e enum includes:
- `ERR_GPIO` - Could be used for button failures
- `POWER_BUTTON_PRESS` - For operational button presses

---

## 7. State Machine Summary

### Full State Transitions
```
BOOTUP (Hardware Verification)
  ├─ verifyButton() returns FALSE → ERROR_STATE
  ├─ verifyODrive/I2C/LED fail → ERROR_STATE
  └─ All pass → IDLE

IDLE (Wait for Host)
  └─ CMD_PING received → CONNECTED

CONNECTED (Verify Communications)
  ├─ Connection check fails → ERROR_STATE
  ├─ System homed → READY
  └─ Not homed → HOMING

HOMING
  └─ Homing complete → READY

READY (Operational)
  └─ Telemetry streaming

ERROR_STATE (Safe State)
  └─ powerButton pressed → BOOTUP

POWERINGOFF (Shutdown)
  └─ Power cut
```

---

## 8. Diagnostics

### How to Debug Button Failure
1. **Check Serial output** for "ERROR: BUTTON FAILURE" message
2. **Measure GPIO voltage** for affected pins:
   - HIGH (~3.3V) when unpressed ✓
   - LOW (~0V) when pressed ✓
   - LOW when NOT pressed = hardware issue ✗
3. **Inspect wiring**:
   - Check for shorts between pin and GND
   - Verify pull-up resistor continuity
   - Look for bent/damaged pins
4. **Check physical buttons**:
   - Press each button individually
   - Feel for sticking or binding
   - Test without external pressure

### Serial Diagnostics
- **BOOTUP check**: Runs `digitalRead()` on all 4 buttons
- **No polling loop**: Single check during BOOTUP, then wait for recovery press
- **Power recovery dependent**: Must press button to exit ERROR_STATE

---

## 9. Key Files Summary

| File | Purpose | Button-Related Code |
|------|---------|-------------------|
| [buttons.h](include/buttons.h) | Button structures & API | Button definitions, buttonEvents_t |
| [buttons.cpp](src/buttons.cpp) | Button implementation | buttonInit(), buttonDetect(), buttonUpdate() |
| [states.h](include/states.h) | State machine declarations | verifyButton() declaration |
| [states.cpp](src/states.cpp) | State machine logic | verifyButton(), ERROR_STATE handling, recovery |
| [errors.h](include/errors.h) | Error code enums | BUTTON_ERROR enum |
| [main.cpp](src/main.cpp) | Entry point | Imports buttons.h, calls stateUpdate() |
| [events.h](include/events.h) | Event system (unused) | ERR_GPIO, POWER_BUTTON_PRESS events |

---

## 10. Current Limitations

1. **No debouncing**: Single `digitalRead()` check, no software debounce
2. **No runtime monitoring**: Only checked during BOOTUP, not during operation
3. **Binary state only**: Tracks HIGH/LOW, no analog readings
4. **Unused real-time API**: `buttonDetect()` and `buttonUpdate()` not integrated
5. **No event dispatch**: Event system defined but not implemented
6. **No history**: No button press logs or diagnostics
7. **Recovery requires button press**: Operator must physically interact to recover
