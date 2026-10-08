# SynDex base: Raspberry Pi 5 setup

> **Status: skeleton.** Anything not yet verified on the real hardware is marked **TODO**. The goal
> is that any team member can rebuild the base from a blank SD card by following this file (firmware
> plan item 1.12). `setup.sh` in this folder will automate it step by step.

The base is a Raspberry Pi 5 running the C++ program in `base/` (built with CMake, not PlatformIO),
with two OV9281 global-shutter IR cameras on the CAM0 and CAM1 ports. It hosts its own WiFi access
point for the two handles only; the Unity laptop keeps its own WiFi and connects to the Pi over
Bluetooth serial (section 5).

## 1. OS version to flash

- Raspberry Pi OS, 64-bit, flashed with Raspberry Pi Imager. The Pi 5 needs Bookworm or newer.
- **TODO:** pin the exact image/version the team uses (and whether Desktop or Lite).
- **TODO:** hostname, username, SSH enabled, and how the Pi is reached before its own AP exists.

## 2. Camera overlays (`/boot/firmware/config.txt`)

Add the OV9281 overlay for each CSI port, then reboot:

```
dtoverlay=ov9281,cam0
dtoverlay=ov9281,cam1
```

- **TODO:** verify on hardware that these two lines are enough with our exact OV9281 modules, and
  whether `camera_auto_detect=0` is also needed.
- **TODO:** confirm cable type with electrical (the Pi 5 CSI connectors are 22-pin).
- **TODO:** if the modules expose a hardware sync/trigger pin, document how it is wired.
- Mechanical: the cameras sit 4 cm apart (center to center), facing straight up.

## 3. Packages to install

```
sudo apt update
sudo apt install -y cmake build-essential pkg-config libcamera-dev libopencv-dev
```

- `pkg-config` is needed so CMake can find libcamera.
- `rpicam-apps` (provides `rpicam-hello`) ships with Raspberry Pi OS. **TODO:** confirm on our image.
- **TODO:** record the OpenCV version that `libopencv-dev` installs on our image.

Build the base program (from the repo root):

```
cmake -S base -B base/build
cmake --build base/build
```

This produces `syndex_base` and `syndex_calibrate` in `base/build/`.

## 4. WiFi access point (handles only)

The Pi hosts its own network for the two handles so they never depend on venue WiFi. The Unity
laptop does not join it. It must be 2.4 GHz (the ESP32-S3 handles are 2.4 GHz only). Starting
point with NetworkManager (Bookworm):

```
sudo nmcli device wifi hotspot ifname wlan0 con-name SynDexAP ssid <SSID> band bg channel <CHANNEL> password <PASSWORD>
sudo nmcli connection modify SynDexAP ipv4.addresses <PI_ADDRESS>/24
sudo nmcli connection modify SynDexAP connection.autoconnect yes
```

- **TODO:** choose SSID, password, channel and the Pi's fixed address (handles connect to that
  fixed address), then verify these commands on the Pi 5.
- **TODO:** DHCP/addressing for the handles.
- **TODO:** developers join the Pi's access point to push handle OTA updates; end users never need to.

## 5. Bluetooth serial link to the laptop

The Unity laptop talks to the Pi over Bluetooth Classic serial (SPP/RFCOMM). On Windows the Pi
shows up as a COM port, so Unity uses plain serial with V1-style framing + CRC.

- **TODO:** install BlueZ + `libbluetooth-dev` (`sudo apt install -y bluez libbluetooth-dev`, also
  in `setup.sh`).
- **TODO:** make the Pi discoverable and pairable.
- **TODO:** register the Serial Port (SPP) service.
- **TODO:** pair the laptop from Windows Bluetooth settings.
- **TODO:** find the COM port Windows assigns to the Pi (Device Manager / Bluetooth "More options").
- Fallback (note only, not built): if Bluetooth is too slow, the laptop joins the Pi's network
  with a USB WiFi adapter and the Unity link uses UDP instead.

## 6. Run the base program as a systemd service on boot

Skeleton unit (`/etc/systemd/system/syndex-base.service`):

```
[Unit]
Description=SynDex base program
After=network-online.target
Wants=network-online.target

[Service]
ExecStart=/opt/syndex/base/build/syndex_base
Restart=on-failure
RestartSec=2

[Install]
WantedBy=multi-user.target
```

Enable with `sudo systemctl daemon-reload && sudo systemctl enable --now syndex-base`.

- **TODO:** final install path, user, working directory.
- **TODO:** hardware watchdog (`WatchdogSec`), read-only/overlay root filesystem so pulling power
  does not corrupt the SD card (firmware plan items 17.3 and 17.7).

## 7. Verification

- `rpicam-hello --list-cameras` must list **2 cameras** (one per CSI port).
- `cmake --build base/build` finishes without errors.
- **TODO:** a handle can join the access point and reach the Pi's fixed address.
- **TODO:** the laptop pairs with the Pi and a COM port appears.
- **TODO:** measure Unity-link latency with the handle WiFi access point and Bluetooth both active
  (they share the Pi's radio).
- **TODO:** `syndex_base` starts on boot and is restarted after a crash.
