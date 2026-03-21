#include "main.h"

typedef enum {
    BOOTUP,
    HOMING_REQUEST,
    HOMING_COMPLETE,
    POWER_BUTTON_PRESS,
    TOOL_BUTTON_PRESS,
    TRIGGER_PRESS,
    TRIGGER_RELEASE,
    ERR_POWER,
    ERR_GPIO,
    ERR_CAN,
    ERR_ODRIVE,
    ERR_I2C,
    ERR_HOMING,
    ERR_USB,
    ERR_UNDEFINED
} hardwareEvent_e;

/* Current hardware event */
extern hardwareEvent_e mHardwareEvent;

namespace event {
    /* Send event - can handle state changes on events */
    dispatchEvent();
}