// Bluetooth Classic serial (SPP/RFCOMM) link to the Unity laptop. See unity_link.h for the
// protocol. Merged from the standalone pi_link_simple.cpp prototype.
//
// For: Base (Raspberry Pi 5)

#include "unity_link.h"

#include <errno.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <thread>

namespace {

const char* kPort = "/dev/rfcomm0";

int g_fd = -1;
std::string g_incoming;
void (*g_onCommand)(int) = nullptr;
void (*g_onDisconnect)() = nullptr;

void closePort() {
    if (g_fd >= 0) {
        close(g_fd);
        g_fd = -1;
    }
    g_incoming.clear();
}

void handleDisconnect() {
    closePort();
    if (g_onDisconnect) g_onDisconnect();
}

bool writeAll(const char* data, size_t len) {
    while (len > 0) {
        ssize_t w = write(g_fd, data, len);
        if (w < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) return true;  // drop; next cycle resends
            if (errno == EINTR) continue;
            handleDisconnect();
            return false;
        }
        data += w;
        len -= static_cast<size_t>(w);
    }
    return true;
}

void handleLine(const std::string& line) {
    if (line.rfind("CMD ", 0) == 0) {
        int id = std::atoi(line.c_str() + 4);
        if (g_onCommand) g_onCommand(id);
    } else if (!line.empty()) {
        std::printf("unity_link: unknown line: %s\n", line.c_str());
    }
}

}  // namespace

bool unityLinkBegin() {
    if (g_fd >= 0) return true;
    // Non-blocking so reading never stalls the main loop.
    int fd = open(kPort, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) return false;

    termios tty{};
    if (tcgetattr(fd, &tty) != 0) {
        close(fd);
        return false;
    }
    cfmakeraw(&tty);
    tcsetattr(fd, TCSANOW, &tty);

    g_fd = fd;
    g_incoming.clear();
    return true;
}

bool unityLinkWaitForConnection(int timeoutMs) {
    auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeoutMs);
    while (!unityLinkBegin()) {
        if (std::chrono::steady_clock::now() >= deadline) return false;
        std::this_thread::sleep_for(std::chrono::milliseconds(250));
    }
    return true;
}

bool unityLinkSendPose(float x, float y, float z) {
    if (g_fd < 0) return false;
    char out[96];
    int n = std::snprintf(out, sizeof out, "POS %.3f %.3f %.3f\n", x, y, z);
    if (n <= 0) return false;
    return writeAll(out, static_cast<size_t>(n));
}

bool unityLinkSendImu(unsigned handleId, unsigned seq, float roll, float pitch, float yaw,
                      const float quat[4]) {
    if (g_fd < 0) return false;
    char out[160];
    int n = std::snprintf(out, sizeof out, "IMU %u %u %.1f %.1f %.1f %.4f %.4f %.4f %.4f\n",
                          handleId, seq, roll, pitch, yaw, quat[0], quat[1], quat[2], quat[3]);
    if (n <= 0) return false;
    return writeAll(out, static_cast<size_t>(n));
}

bool unityLinkSendError(const char* message) {
    if (g_fd < 0) return false;
    std::string out = std::string("ERR ") + message + "\n";
    return writeAll(out.data(), out.size());
}

void unityLinkPoll() {
    if (g_fd < 0) return;

    char buf[128];
    ssize_t r;
    while ((r = read(g_fd, buf, sizeof buf)) > 0) g_incoming.append(buf, static_cast<size_t>(r));
    if (r == 0 || (r < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR)) {
        handleDisconnect();  // EOF / I/O error: the laptop dropped the link
        return;
    }

    size_t nl;
    while ((nl = g_incoming.find('\n')) != std::string::npos) {
        std::string line = g_incoming.substr(0, nl);
        g_incoming.erase(0, nl + 1);
        if (!line.empty() && line.back() == '\r') line.pop_back();
        handleLine(line);
    }
}

bool unityLinkIsConnected() {
    return g_fd >= 0;
}

void unityLinkOnCommand(void (*callback)(int id)) {
    g_onCommand = callback;
}

void unityLinkOnDisconnect(void (*callback)()) {
    g_onDisconnect = callback;
}
