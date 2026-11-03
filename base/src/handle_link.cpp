// UDP link to the two handles: receives orientation, gyro/accel, trigger and dock state,
// routes it by handle ID, and discards stale or out-of-order packets. Also forwards vibration
// commands back to the correct handle.
//
// For: Base (Raspberry Pi 5)

// confirm receiving and sending test packets using UDP
// server creaete?

/// Initialise the server object
// var type = network_socket_udp;
// var port = 8000;
// socket = network_create_socket_ext(type,port);
// //Placeholders
// remote_port = 0;
// remote_ip = “0.0.0.0”;
// //Make a buffer to hold packet data
// var size = 1024;
// var type = buffer_fixed;
// var alignment = 1;
// send_buffer = buffer_create(size,type,alignment);
// UDP link to the two handles: receives orientation, gyro/accel, trigger and dock state,
// routes it by handle ID, and discards stale or out-of-order packets. Also forwards vibration
// commands back to the correct handle.
//
// For: Base (Raspberry Pi 5). Also builds on Windows (Winsock) for desktop testing.

#include "handle_link.h"
#include "messages.h"

#include <chrono>
#include <cstdio>
#include <cstring>
#include <vector>

#ifdef _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
typedef SOCKET sock_t;
#define SOCK_INVALID INVALID_SOCKET
#define SOCK_POLL(pfd, n, ms) WSAPoll((pfd), (n), (ms))
typedef WSAPOLLFD pollfd_t;
#else
#include <unistd.h>
#include <poll.h>
#include <fcntl.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <netinet/in.h>
typedef int sock_t;
#define SOCK_INVALID (-1)
#define SOCK_POLL(pfd, n, ms) poll((pfd), (n), (ms))
typedef pollfd pollfd_t;
#endif

using steady = std::chrono::steady_clock;

namespace
{
    // After this much silence from a handle, accept any sequence number (covers handle reboots,
    // which restart seq at 0 and would otherwise look "stale").
    const std::chrono::milliseconds RESYNC_AFTER(1000);

    struct handleSlot
    {
        sockaddr_in addr{};
        bool known = false;
        bool haveSeq = false;
        uint16_t lastSeq = 0;
        uint16_t txSeq = 0;
        bool fresh = false;
        handleStatePayload latest{};
        steady::time_point lastRx{};
    };

    sock_t g_fd = SOCK_INVALID;
    handleSlot g_slots[NUM_HANDLES];

    void closeSocket(sock_t s)
    {
#ifdef _WIN32
        closesocket(s);
#else
        close(s);
#endif
    }

    bool setNonBlocking(sock_t s)
    {
#ifdef _WIN32
        u_long mode = 1;
        return ioctlsocket(s, FIONBIO, &mode) == 0;
#else
        int flags = fcntl(s, F_GETFL, 0);
        return flags >= 0 && fcntl(s, F_SETFL, flags | O_NONBLOCK) == 0;
#endif
    }

    // true if seq is newer than last (handles 16-bit wraparound)
    bool isNewer(uint16_t seq, uint16_t last)
    {
        return static_cast<int16_t>(seq - last) > 0;
    }

    bool sendPacket(handleSlot &s, packet &pkt)
    {
        std::vector<uint8_t> bytes = pkt.serialize();
        int n = (int)sendto(g_fd, (const char *)bytes.data(), (int)bytes.size(), 0,
                            (const sockaddr *)&s.addr, sizeof(s.addr));
        return n == (int)bytes.size();
    }

    void sendAck(handleSlot &s, uint8_t id, uint16_t ackedSeq)
    {
        ackPayload a;
        a.meta.handleId = id;
        a.meta.seq = s.txSeq++;
        a.ackedSeq = ackedSeq;
        packet pkt(RESP_ACK, a);
        sendPacket(s, pkt);
    }

    void handleIncoming(const uint8_t *buf, size_t len, const sockaddr_in &src)
    {
        packet pkt;
        if (!packetParse(buf, len, pkt))
            return; // bad sync/length/CRC

        switch (pkt.header.packetType)
        {
        case TELEM_HANDLE_STATE:
        {
            if (pkt.header.payloadSize != sizeof(handleStatePayload))
                return;
            handleStatePayload st;
            std::memcpy(&st, pkt.payload, sizeof(st));
            if (st.meta.handleId >= NUM_HANDLES)
                return;

            handleSlot &s = g_slots[st.meta.handleId];
            auto now = steady::now();
            bool resync = !s.known || (now - s.lastRx) > RESYNC_AFTER;
            if (s.haveSeq && !resync && !isNewer(st.meta.seq, s.lastSeq))
                return; // stale/duplicate

            s.haveSeq = true;
            s.lastSeq = st.meta.seq;
            s.lastRx = now;
            s.addr = src;
            s.addr.sin_port = htons(HANDLE_PORT); // reply to the handle's listen port
            s.known = true;
            s.latest = st;
            s.fresh = true;

            // Acking every state packet doubles as the handle's "base is alive" signal.
            // TODO: decide whether you want this, or a separate heartbeat (CMD_PING/RESP_PONG).
            sendAck(s, st.meta.handleId, st.meta.seq);
            break;
        }
        case RESP_ACK:
            // TODO: match ackedSeq against pending vibrate commands, cancel their retransmit timers
            break;
        default:
            break; // TODO: other message types
        }
    }
} // namespace

bool handle_link_init()
{
#ifdef _WIN32
    WSADATA wsa;
    if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0)
    {
        std::fprintf(stderr, "WSAStartup failed\n");
        return false;
    }
#endif

    g_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (g_fd == SOCK_INVALID)
    {
        std::perror("socket");
        return false;
    }

#ifdef _WIN32
    // Stop Windows from reporting ICMP "port unreachable" as a receive error on UDP sockets.
    BOOL newBehavior = FALSE;
    DWORD bytesReturned = 0;
    WSAIoctl(g_fd, _WSAIOW(IOC_VENDOR, 12), &newBehavior, sizeof(newBehavior),
             NULL, 0, &bytesReturned, NULL, NULL);
#endif

    sockaddr_in local{};
    local.sin_family = AF_INET;
    local.sin_addr.s_addr = htonl(INADDR_ANY);
    local.sin_port = htons(BASE_PORT);
    if (bind(g_fd, (sockaddr *)&local, sizeof(local)) < 0 || !setNonBlocking(g_fd))
    {
        std::perror("bind/nonblocking");
        closeSocket(g_fd);
        g_fd = SOCK_INVALID;
        return false;
    }
    return true;
}

void handle_link_poll(int timeout_ms)
{
    if (g_fd == SOCK_INVALID)
        return;

    pollfd_t pfd;
    std::memset(&pfd, 0, sizeof(pfd));
    pfd.fd = g_fd;
    pfd.events = POLLIN;
    if (SOCK_POLL(&pfd, 1, timeout_ms) <= 0 || !(pfd.revents & POLLIN))
        return;

    uint8_t buf[300]; // max packet = 4 + 255 + 2
    for (;;)          // drain everything queued
    {
        sockaddr_in src{};
        socklen_t srcLen = sizeof(src);
        int n = (int)recvfrom(g_fd, (char *)buf, (int)sizeof(buf), 0, (sockaddr *)&src, &srcLen);
        if (n < 0)
            break; // nothing left (non-blocking) or error
        handleIncoming(buf, (size_t)n, src);
    }
    // TODO: retransmit timers for unacked vibrate commands
}

bool handle_link_get_state(uint8_t handleId, handleStatePayload &out)
{
    if (handleId >= NUM_HANDLES || !g_slots[handleId].fresh)
        return false;
    out = g_slots[handleId].latest;
    g_slots[handleId].fresh = false;
    return true;
}

bool handle_link_send_vibrate(uint8_t handleId, uint8_t intensity, uint16_t durationMs)
{
    if (handleId >= NUM_HANDLES || !g_slots[handleId].known)
        return false; // haven't heard from it yet
    handleSlot &s = g_slots[handleId];

    vibrateCmdPayload v;
    v.meta.handleId = handleId;
    v.meta.seq = s.txSeq++;
    v.intensity = intensity;
    v.durationMs = durationMs;
    packet pkt(CMD_VIBRATE, v);
    return sendPacket(s, pkt);
}

bool handle_link_send_ir(uint8_t handleId, bool enabled, uint8_t brightness)
{
    if (handleId >= NUM_HANDLES || !g_slots[handleId].known)
        return false;
    handleSlot &s = g_slots[handleId];

    irLedCmdPayload c;
    c.meta.handleId = handleId;
    c.meta.seq = s.txSeq++;
    c.enabled = enabled ? 1 : 0;
    c.brightness = brightness;
    packet pkt(CMD_SET_IR_LED, c);
    return sendPacket(s, pkt);
}

bool handle_link_is_up(uint8_t handleId, int timeout_ms)
{
    if (handleId >= NUM_HANDLES || !g_slots[handleId].known)
        return false;
    return (steady::now() - g_slots[handleId].lastRx) < std::chrono::milliseconds(timeout_ms);
}

void handle_link_close()
{
    if (g_fd != SOCK_INVALID)
        closeSocket(g_fd);
    g_fd = SOCK_INVALID;
#ifdef _WIN32
    WSACleanup();
#endif
}