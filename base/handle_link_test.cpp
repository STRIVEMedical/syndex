#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include "handle_link.h"

// Prints the handle's orientation as degrees, using the same conversion as handle/src/imu.cpp
static void printState(const handleStatePayload &st)
{
    const double w = st.quat[0], x = st.quat[1], y = st.quat[2], z = st.quat[3];
    const double toDeg = 180.0 / 3.14159265358979323846;

    const double roll = std::atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y));
    const double pitch = std::asin(std::clamp(2.0 * (w * y - z * x), -1.0, 1.0));
    const double yaw = std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));

    std::printf("handle %u seq %u flags 0x%02X roll=%.1f pitch=%.1f yaw=%.1f\n",
                st.meta.handleId, st.meta.seq, st.flags,
                roll * toDeg, pitch * toDeg, yaw * toDeg);
}

int main()
{
    if (!handle_link_init())
        return 1;
    std::printf("Listening on UDP port %u...\n", BASE_PORT);

    auto lastVib = std::chrono::steady_clock::now();
    for (;;)
    {
        handle_link_poll(100);
        for (uint8_t id = 0; id < NUM_HANDLES; ++id)
        {
            handleStatePayload st;
            if (handle_link_get_state(id, st))
                printState(st);
        }

        // Every 3 s, buzz handle 0 for 200 ms (only works once it has sent us a packet)
        auto now = std::chrono::steady_clock::now();
        if (now - lastVib > std::chrono::seconds(3))
        {
            lastVib = now;
            bool ok = handle_link_send_vibrate(0, 200, 200);
            std::printf("sent vibrate to handle 0: %s\n", ok ? "yes" : "no (handle not heard yet)");
        }
        // test IR
        handle_link_send_ir(0, true, 255);
    }
}