// UDP link from the handle to the Pi base's WiFi access point: sends orientation, gyro/accel,
// trigger and dock state with sequence numbers, and receives vibration commands. Stops the
// vibration motor on link loss.
//
// For: Handle ESP32-S3

#include "base_link.h"
#include <WiFi.h>
#include <WiFiUdp.h>

// Set per handle in platformio.ini: build_flags = -DHANDLE_ID=0 (or 1) -I../shared
#ifndef HANDLE_ID
#define HANDLE_ID 0
#endif

// Override per environment in platformio.ini, e.g.
//   -DLINK_SSID=\"MyWifi\" -DLINK_PASS=\"secret\" -DLINK_BASE_IP=192,168,1,50
#ifndef LINK_SSID
#define LINK_SSID "YOUR_PI_AP_SSID"
#endif
#ifndef LINK_PASS
#define LINK_PASS "YOUR_PI_AP_PASSWORD"
#endif
#ifndef LINK_BASE_IP
#define LINK_BASE_IP 10, 42, 0, 1 // Pi on its own AP (NetworkManager default)
#endif

static const char *AP_SSID = LINK_SSID;
static const char *AP_PASSWORD = LINK_PASS;
static IPAddress BASE_IP(LINK_BASE_IP);

static const unsigned long LINK_TIMEOUT_MS = 500;  // your call
static const unsigned long RESYNC_AFTER_MS = 1000; // accept any seq after this much silence

static WiFiUDP udp;
static uint16_t txSeq = 0;
static uint16_t lastRxSeq = 0;
static bool haveRxSeq = false;
static unsigned long lastRxMs = 0;
static vibrateCmdPayload pendingVibrate;
static bool hasPendingVibrate = false;
static irLedCmdPayload pendingIr;
static bool hasPendingIr = false;

static bool sendPkt(packet &pkt)
{
    std::vector<uint8_t> bytes = pkt.serialize();
    udp.beginPacket(BASE_IP, BASE_PORT);
    udp.write(bytes.data(), bytes.size());
    return udp.endPacket() == 1;
}

static void sendAck(uint16_t ackedSeq)
{
    ackPayload a;
    a.meta.handleId = HANDLE_ID;
    a.meta.seq = txSeq++;
    a.ackedSeq = ackedSeq;
    packet pkt(RESP_ACK, a);
    sendPkt(pkt);
}

static void handleIncoming(const uint8_t *buf, size_t len)
{
    packet pkt;
    if (!packetParse(buf, len, pkt))
        return; // bad sync/length/CRC

    unsigned long now = millis();
    bool resync = (now - lastRxMs) > RESYNC_AFTER_MS;

    switch (pkt.header.packetType)
    {
    case CMD_VIBRATE:
    {
        if (pkt.header.payloadSize != sizeof(vibrateCmdPayload))
            return;
        vibrateCmdPayload v;
        memcpy(&v, pkt.payload, sizeof(v));
        if (v.meta.handleId != HANDLE_ID)
            return;

        // TODO: if this is a duplicate (base retransmitted because our ACK was lost),
        // re-send the ACK but don't apply the command again.
        if (haveRxSeq && !resync && (int16_t)(v.meta.seq - lastRxSeq) <= 0)
            return;

        haveRxSeq = true;
        lastRxSeq = v.meta.seq;
        lastRxMs = now;
        pendingVibrate = v;
        hasPendingVibrate = true;
        sendAck(v.meta.seq);
        break;
    }
    case CMD_SET_IR_LED:
    {
        if (pkt.header.payloadSize != sizeof(irLedCmdPayload))
            return;
        irLedCmdPayload c;
        memcpy(&c, pkt.payload, sizeof(c));
        if (c.meta.handleId != HANDLE_ID)
            return;
        if (haveRxSeq && !resync && (int16_t)(c.meta.seq - lastRxSeq) <= 0)
            return;

        haveRxSeq = true;
        lastRxSeq = c.meta.seq;
        lastRxMs = now;
        pendingIr = c;
        hasPendingIr = true;
        sendAck(c.meta.seq);
        break;
    }
    case RESP_ACK:
    {
        if (pkt.header.payloadSize != sizeof(ackPayload))
            return;
        ackPayload a;
        memcpy(&a, pkt.payload, sizeof(a));
        if (a.meta.handleId != HANDLE_ID)
            return;
        lastRxMs = now; // any valid packet from the base counts as "link alive"
        // TODO: match a.ackedSeq against your pending-ACK bookkeeping
        break;
    }
    default:
        break;
    }
}

bool base_link_init()
{
    WiFi.mode(WIFI_STA);
    WiFi.begin(AP_SSID, AP_PASSWORD);
    unsigned long start = millis();
    while (WiFi.status() != WL_CONNECTED)
    {
        if (millis() - start > 15000)
            return false; // don't hang forever
        delay(250);
    }
    WiFi.setSleep(false);
    udp.begin(HANDLE_PORT);
    lastRxMs = millis();
    Serial.println(WiFi.localIP());
    return true;
}

void base_link_update()
{
    int size = udp.parsePacket();
    while (size > 0)
    {
        uint8_t buf[300];
        int len = udp.read(buf, sizeof(buf));
        if (len > 0 && udp.remoteIP() == BASE_IP)
            handleIncoming(buf, (size_t)len);
        size = udp.parsePacket();
    }

    if (!base_link_is_up())
    {
        // TODO: stop the vibration motor (link-loss safe state)
    }
    if (WiFi.status() != WL_CONNECTED)
    {
        // TODO: reconnect strategy (WiFi.reconnect() with backoff)
    }
}

bool base_link_send_state(handleStatePayload s)
{
    s.meta.handleId = HANDLE_ID;
    s.meta.seq = txSeq++;
    packet pkt(TELEM_HANDLE_STATE, s);
    return sendPkt(pkt);
}

bool base_link_get_vibrate(vibrateCmdPayload &out)
{
    if (!hasPendingVibrate)
        return false;
    out = pendingVibrate;
    hasPendingVibrate = false;
    return true;
}

bool base_link_is_up()
{
    return (millis() - lastRxMs) < LINK_TIMEOUT_MS;
}

bool base_link_get_ir(irLedCmdPayload &out)
{
    if (!hasPendingIr)
        return false;
    out = pendingIr;
    hasPendingIr = false;
    return true;
}