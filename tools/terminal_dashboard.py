#!/usr/bin/env python3
"""
Terminal dashboard for reading button states and toggling LEDs.
Usage: python tools/terminal_dashboard.py --port COM3
Keys:
  1/2/3 - toggle LED 0/1/2 (power/data/error)
  s     - request status
  q     - quit

Requires: pyserial
"""
import argparse
import curses
import serial
import time


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument("--port", required=True, help="Serial port (e.g. COM3 or /dev/ttyACM0)")
    p.add_argument("--baud", default=115200, type=int)
    return p.parse_args()


class Dashboard:
    def __init__(self, ser):
        self.ser = ser
        self.buttons = [0, 0, 0, 0]
        self.leds = [0, 0, 0]
        self.last_status = 0

    def request_status(self):
        try:
            self.ser.write(b'STATUS\n')
        except Exception:
            pass

    def toggle_led(self, idx):
        cmd = f"TOGGLE {idx}\n".encode('ascii')
        try:
            self.ser.write(cmd)
        except Exception:
            pass

    def set_led(self, idx, val):
        cmd = f"LED {idx} {1 if val else 0}\n".encode('ascii')
        try:
            self.ser.write(cmd)
        except Exception:
            pass

    def read_lines(self):
        # read available lines
        try:
            while self.ser.in_waiting:
                line = self.ser.readline().decode('ascii', errors='ignore').strip()
                self._handle_line(line)
        except Exception:
            pass

    def _handle_line(self, line):
        if not line:
            return
        if line.startswith('BTN'):
            parts = line.split()
            # BTN pwr aut trg tool
            if len(parts) >= 5:
                self.buttons = [int(x) for x in parts[1:5]]
        elif line.startswith('LED'):
            parts = line.split()
            if len(parts) >= 4:
                self.leds = [int(x) for x in parts[1:4]]

    def draw(self, stdscr):
        stdscr.clear()
        stdscr.addstr(0, 0, 'Terminal Dashboard - press q to quit')
        stdscr.addstr(2, 0, 'Buttons:')
        stdscr.addstr(3, 2, f'Power:   {"PRESSED" if self.buttons[0] else "released"}')
        stdscr.addstr(4, 2, f'AutoHome: {"PRESSED" if self.buttons[1] else "released"}')
        stdscr.addstr(5, 2, f'Trigger:  {"PRESSED" if self.buttons[2] else "released"}')
        stdscr.addstr(6, 2, f'ToolSel:  {"PRESSED" if self.buttons[3] else "released"}')

        stdscr.addstr(8, 0, 'LEDs:')
        stdscr.addstr(9, 2, f'1: Power  - {"ON" if self.leds[0] else "OFF"}')
        stdscr.addstr(10, 2, f'2: Data   - {"ON" if self.leds[1] else "OFF"}')
        stdscr.addstr(11, 2, f'3: Error  - {"ON" if self.leds[2] else "OFF"}')

        stdscr.addstr(13, 0, 'Controls:')
        stdscr.addstr(14, 2, '1/2/3 - toggle LED 1/2/3')
        stdscr.addstr(15, 2, 's - request status')
        stdscr.refresh()


def curses_loop(stdscr, dashboard):
    stdscr.nodelay(True)
    last_req = 0
    while True:
        try:
            now = time.time()
            # auto-request status every 0.5s
            if now - last_req > 0.5:
                dashboard.request_status()
                last_req = now

            dashboard.read_lines()
            dashboard.draw(stdscr)

            c = stdscr.getch()
            if c == -1:
                time.sleep(0.05)
                continue
            if c in (ord('q'), ord('Q')):
                break
            if c in (ord('s'), ord('S')):
                dashboard.request_status()
            if c == ord('1'):
                dashboard.toggle_led(0)
            if c == ord('2'):
                dashboard.toggle_led(1)
            if c == ord('3'):
                dashboard.toggle_led(2)
        except KeyboardInterrupt:
            break


def main():
    args = parse_args()
    try:
        ser = serial.Serial(args.port, args.baud, timeout=0.1)
    except Exception as e:
        print(f"Failed to open {args.port}: {e}")
        return

    dashboard = Dashboard(ser)
    curses.wrapper(curses_loop, dashboard)
    ser.close()


if __name__ == '__main__':
    main()
