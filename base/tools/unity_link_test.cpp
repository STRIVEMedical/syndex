// Standalone test for the Unity link: sends a fake moving position 20x/s and prints any commands
// Unity sends. Run `sudo rfcomm watch hci0 1`, connect from Unity, then run syndex_unity_link_test.
//
// For: Base (Raspberry Pi 5)

#include <chrono>
#include <cstdio>
#include <thread>

#include "unity_link.h"

static void onCommand(int id) { std::printf("Got: CMD %d\n", id); }
static void onDisconnect() { std::printf("Unity disconnected\n"); }

int main() {
    unityLinkOnCommand(onCommand);
    unityLinkOnDisconnect(onDisconnect);

    std::printf("Waiting for /dev/rfcomm0 ...\n");
    float t = 0.0f;
    while (true) {
        if (!unityLinkIsConnected() && !unityLinkWaitForConnection(2000)) continue;
        unityLinkSendPose(t, t * 0.5f, 0.0f);  // fake position; replace with real pose later
        t += 0.05f;
        unityLinkPoll();
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
}
