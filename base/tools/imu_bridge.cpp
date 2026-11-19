// Bench test for the full IMU path: BNO085 -> ESP32 handle (UART) -> Pi (UDP over SynDexAP)
// -> laptop (Bluetooth serial). Forwards every handle state packet as an "IMU ..." line on the
// Unity link (see unity_link.h) and prints it locally. While Bluetooth is connected but no handle
// is sending, it sends "ERR no handle data" every 2 s so the laptop can tell the two links apart.
//
// Handle: flash the handle-imu-link-test env. Pi: `sudo rfcomm watch hci0 1` must be running.
//
// For: Base (Raspberry Pi 5)

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>

#include "handle_link.h"
#include "unity_link.h"

using steady = std::chrono::steady_clock;

// Same quaternion -> degrees conversion as handle/src/imu.cpp and base/handle_link_test.cpp.
static void toEulerDeg(const float q[4], float &roll, float &pitch, float &yaw)
{
    const double w = q[0], x = q[1], y = q[2], z = q[3];
    const double toDeg = 180.0 / 3.14159265358979323846;
    roll = static_cast<float>(std::atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y)) * toDeg);
    pitch = static_cast<float>(std::asin(std::clamp(2.0 * (w * y - z * x), -1.0, 1.0)) * toDeg);
    yaw = static_cast<float>(std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)) * toDeg);
}

static void onDisconnect() { std::printf("Bluetooth: laptop disconnected\n"); }

int main()
{
    if (!handle_link_init())
        return 1;
    unityLinkOnDisconnect(onDisconnect);
    std::printf("Listening for handles on UDP %u, forwarding to /dev/rfcomm0\n", BASE_PORT);

    bool wasConnected = false;
    auto lastData = steady::now();
    auto lastWaitMsg = steady::now();

    for (;;)
    {
        handle_link_poll(20);

        // Pick up the Bluetooth link whenever the laptop connects (non-blocking).
        if (!unityLinkIsConnected())
            unityLinkBegin();
        if (unityLinkIsConnected() && !wasConnected)
            std::printf("Bluetooth: laptop connected\n");
        wasConnected = unityLinkIsConnected();

        for (uint8_t id = 0; id < NUM_HANDLES; ++id)
        {
            handleStatePayload st;
            if (!handle_link_get_state(id, st))
                continue;
            lastData = steady::now();

            // Copy out of the packed struct before taking a pointer (avoids unaligned access).
            const float quat[4] = {st.quat[0], st.quat[1], st.quat[2], st.quat[3]};
            float roll, pitch, yaw;
            toEulerDeg(quat, roll, pitch, yaw);
            unityLinkSendImu(st.meta.handleId, st.meta.seq, roll, pitch, yaw, quat);
            std::printf("handle %u seq %u roll=%.1f pitch=%.1f yaw=%.1f  bt=%s\n", st.meta.handleId,
                        st.meta.seq, roll, pitch, yaw, unityLinkIsConnected() ? "up" : "down");
        }

        auto now = steady::now();
        if (now - lastData > std::chrono::seconds(2) && now - lastWaitMsg > std::chrono::seconds(2))
        {
            lastWaitMsg = now;
            unityLinkSendError("no handle data");
            std::printf("no handle data  bt=%s\n", unityLinkIsConnected() ? "up" : "down");
        }

        unityLinkPoll(); // read any CMD lines and detect a dropped laptop
    }
}
