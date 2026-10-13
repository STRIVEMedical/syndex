#!/usr/bin/env python3
"""
bt_serial_test_client.py - Laptop-side stand-in for Unity (skeleton, not implemented yet).

Purpose: talk to the Raspberry Pi base over its Bluetooth serial (SPP) link, which Windows
exposes as a COM port, using pyserial and the V1-style framing + CRC from shared/messages.h,
so the base's Unity link can be tested without Unity.

Usage (planned):
    python tools/bt_serial_test_client.py --port COM5

Requires: pyserial
"""
import argparse


def parse_args():
    p = argparse.ArgumentParser(description="SynDex Bluetooth serial test client (skeleton)")
    p.add_argument("--port", required=True, help="COM port Windows assigned to the paired Pi (e.g. COM5)")
    return p.parse_args()


def main():
    args = parse_args()
    # TODO: open the COM port with pyserial.
    # TODO: send CMD_PING framed per shared/messages.h (0x7F 0xFE, type, length, payload, CRC-16/ARC).
    # TODO: print RESP_PONG, pose telemetry and status; report CRC errors and dropped frames.
    raise NotImplementedError(f"Bluetooth serial test client not implemented yet (port {args.port})")


if __name__ == "__main__":
    main()
