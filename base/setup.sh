#!/usr/bin/env bash
# SynDex base: Raspberry Pi 5 setup (skeleton). See SETUP.md for the full procedure.
# Run on the Pi from the repo root:  bash base/setup.sh
set -euo pipefail

# 1. Packages
sudo apt update
sudo apt install -y cmake build-essential pkg-config libcamera-dev libopencv-dev bluez libbluetooth-dev

# 2. Camera overlays in /boot/firmware/config.txt (dtoverlay=ov9281,cam0 / cam1)
# TODO

# 3. WiFi access point for the handles only (2.4 GHz, fixed address)
# TODO

# 4. Bluetooth serial link to the laptop (BlueZ, SPP service, pairing)
# TODO

# 5. Build the base program (cmake -S base -B base/build && cmake --build base/build)
# TODO

# 6. Install and enable the systemd service
# TODO

# 7. Verify (rpicam-hello --list-cameras must show 2 cameras; laptop sees a COM port)
# TODO

echo "Packages installed. Steps 2-7 are not implemented yet; follow base/SETUP.md."
