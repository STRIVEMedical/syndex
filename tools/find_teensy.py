#!/usr/bin/env python3
"""
find_teensy.py - Locate Teensy COM port by serial number

Usage:
    python find_teensy.py                    # Find any Teensy
    python find_teensy.py <serial_number>   # Find specific Teensy by SN
    
Output: Prints the COM port (e.g., COM3)
"""

import serial.tools.list_ports
import sys

def find_teensy_com_port(serial_num=None):
    """
    Find Teensy COM port by serial number.
    If serial_num is None, returns first available Teensy.
    """
    ports = serial.tools.list_ports.comports()
    
    for port, desc, hwid in ports:
        # Check if this is a Teensy device
        if "Teensy" in desc or "VID:PID=16C0:04" in hwid:
            if serial_num is None:
                # Return first Teensy found
                return port, hwid
            elif serial_num in hwid:
                # Return matching serial number
                return port, hwid
    
    return None, None

if __name__ == "__main__":
    serial_num = sys.argv[1] if len(sys.argv) > 1 else None
    
    com_port, hwid = find_teensy_com_port(serial_num)
    
    if com_port:
        print(f"Teensy found on: {com_port}")
        if hwid:
            print(f"Hardware ID: {hwid}")
        sys.exit(0)
    else:
        if serial_num:
            print(f"Teensy with serial {serial_num} not found")
        else:
            print("No Teensy devices found")
        sys.exit(1)
