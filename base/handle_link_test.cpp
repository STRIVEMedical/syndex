#include <chrono>
#include <cstdio>
#include "handle_link.h"

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
                std::printf("handle %u seq %u flags 0x%02X quat[0]=%.3f\n",
                            st.meta.handleId, st.meta.seq, st.flags, st.quat[0]);
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