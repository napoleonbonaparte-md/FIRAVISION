# FIRAVISION

> Linux kernel camera driver for the FIRAVISION thermal camera on NVIDIA Jetson Orin Nano.  
> Developed and maintained by **Covenant**.

[![Branch](https://img.shields.io/badge/branch-reset-blue)](https://github.com/napoleonbonaparte-md/FIRAVISION/tree/reset)
[![Platform](https://img.shields.io/badge/platform-Jetson%20Orin%20Nano-green)](https://developer.nvidia.com/embedded/jetson-orin-nano)
[![Kernel](https://img.shields.io/badge/kernel-5.15%20jammy-orange)](https://kernel.org)
[![License](https://img.shields.io/badge/license-Proprietary-red)](#license)

---

## Overview

FIRAVISION is a V4L2/Tegra camera driver for the UR Series thermal imaging sensor, targeting the NVIDIA Jetson Orin Nano platform under JetPack 6 (kernel 5.15). The driver exposes the camera as a standard V4L2 subdevice, integrates with the Tegra camera pipeline (CSI/VI/NVCSI), and supports both single and dual camera configurations.

**Key capabilities:**
- YUV422 (YUYV) output at 640×512, up to 30 FPS
- I2C communication over 2-lane MIPI CSI-2 (DPHY)
- Hardware reset via GPIO with automatic stream recovery
- Sysfs interface for runtime camera reset without rebooting
- Dual camera support (CAM0 on `serial_b`, CAM1 on `serial_c`)
- Device Tree Overlay based configuration

---

## Hardware

| Property       | Value                        |
|----------------|------------------------------|
| Sensor Chip ID | `0x5352`                     |
| I2C Address    | `0x3C`                       |
| Resolution     | 640 × 512                    |
| Format         | YUV422 (YUYV8\_1X16)        |
| FPS            | 30                           |
| CSI Lanes      | 2 (DPHY)                     |
| MCLK           | 24 MHz (`extperiph1`)        |
| Platform       | Jetson Orin Nano (p3767-0005)|

### Reset GPIO — Per Camera

| Camera | GPIO Index | Pin Name   | CSI Interface | I2C Bus (mux) |
|--------|------------|------------|---------------|----------------|
| CAM0   | `0x3E` (62) | PH.06 — CAM0_PWDN | `serial_b` (CSI B) | `i2c@0` |
| CAM1   | `0xA0` (160) | PAC.00 — CAM1_PWDN | `serial_c` (CSI C) | `i2c@1` |

> Both cameras share I2C address `0x3C` but are isolated via a GPIO-controlled I2C mux (`i2c-mux-gpio`) on `gpio_aon` pin `0x13`.

---

## Repository Structure

```
FIRAVISION/
├── imx219.c              # Main camera driver (single camera)
├── imx219_reset.c        # Camera driver with sysfs reset support
├── imx219_reset.ko       # Prebuilt kernel module
├── ur_thermal.dts        # Device Tree Overlay — single camera (CAM0)
├── ur_dual_thermal.dts   # Device Tree Overlay — dual camera (CAM0 + CAM1)
├── imx_build.sh          # Build script for the kernel module
├── dts_build.sh          # Build and apply script for the DTS overlay
└── README.md
```

---

## Requirements

- NVIDIA Jetson Orin Nano (`p3768-0000+p3767-0005`)
- JetPack 6 / Kernel 5.15 (kernel-jammy)
- Kernel source at `~/Desktop/Linux_for_tegra/source/kernel/kernel-jammy-src/`
- Tools: `dtc`, `fdtoverlay`, `make`, `depmod`

---

## Build

### 1. Build the kernel module

```bash
chmod +x imx_build.sh
./imx_build.sh
```

This script compiles `imx219.ko` from the kernel source tree and copies it to the live modules directory:

```bash
cd ~/Desktop/Linux_for_tegra/source/kernel/kernel-jammy-src/
make M=drivers/media/i2c modules -j$(nproc)
sudo cp drivers/media/i2c/imx219.ko /lib/modules/$(uname -r)/kernel/drivers/media/i2c/
```

### 2. Build and apply the Device Tree Overlay

```bash
chmod +x dts_build.sh
./dts_build.sh
```

This compiles `ur_thermal.dts` into a `.dtbo` overlay and merges it into the active DTB:

```bash
dtc -O dtb -o imx219_fixed.dtbo -@ ur_thermal.dts
sudo fdtoverlay -i kernel_tegra234-p3768-0000+p3767-0005-nv-super_COPY.dtb \
    -o /tmp/merged_fix.dtb imx219_fixed.dtbo
sudo cp /tmp/merged_fix.dtb /boot/dtb/kernel_tegra234-p3768-0000+p3767-0005-nv-super.dtb
```

> **Note:** The base DTB copy (`kernel_tegra234-..._COPY.dtb`) must be present in the working directory before running the script. Reboot after applying.

---

## Load / Unload

```bash
# Load
sudo modprobe imx219
# or
sudo insmod /lib/modules/$(uname -r)/kernel/drivers/media/i2c/imx219.ko

# Unload
sudo rmmod imx219

# Reload
sudo rmmod imx219 && sudo modprobe imx219
```

---

## Runtime Camera Reset (Sysfs)

The `imx219_reset` variant of the driver exposes a sysfs interface to reset the camera and automatically restore the stream without rebooting.

**Trigger a reset — Single Camera:**
```bash
echo 1 > /sys/bus/i2c/devices/4-003c/reset_camera
```

**Trigger a reset — Dual Camera:**

In dual camera mode, each camera is on its own I2C mux channel and has a separate sysfs node:

```bash
# CAM0 — GPIO 0x3E (PH.06), CSI B
echo 1 > /sys/bus/i2c/devices/<mux-bus-cam0>-003c/reset_camera

# CAM1 — GPIO 0xA0 (PAC.00), CSI C
echo 1 > /sys/bus/i2c/devices/<mux-bus-cam1>-003c/reset_camera
```

To find the exact mux bus numbers on your system:
```bash
ls /sys/bus/i2c/devices/ | grep "003c"
# Example output:
# 4-003c     ← single camera
# 5-003c     ← CAM0 mux channel
# 6-003c     ← CAM1 mux channel
```

**Reset sequence performed by the driver:**
1. Sends stop stream command over I2C
2. Asserts the camera's dedicated reset GPIO low for 20 ms (CAM0: `0x3E`, CAM1: `0xA0`)
3. De-asserts reset GPIO, waits 50 ms for camera to boot
4. Sends start stream command to restore the pipeline

**Verify the reset worked:**
```bash
# Watch kernel log in real time
sudo dmesg -W

# Expected output:
# ur_thermal 5-003c: Camera reset: asserting reset GPIO
# ur_thermal 5-003c: Camera reset: GPIO released, camera booting
# ur_thermal 5-003c: Camera reset: restarting stream
# ur_thermal 5-003c: Camera reset: stream restored
```

**Check streaming state:**
```bash
# CAM0
cat /sys/bus/i2c/devices/<mux-bus-cam0>-003c/streaming

# CAM1
cat /sys/bus/i2c/devices/<mux-bus-cam1>-003c/streaming

# 0 = stopped  |  1 = streaming
```

**Automated watchdog example:**
```bash
#!/bin/bash
CAM0_SYSFS="/sys/bus/i2c/devices/5-003c"
CAM1_SYSFS="/sys/bus/i2c/devices/6-003c"

while true; do
    if ! your_health_check_cam0; then
        echo "CAM0 stream lost — resetting..."
        echo 1 > $CAM0_SYSFS/reset_camera
    fi
    if ! your_health_check_cam1; then
        echo "CAM1 stream lost — resetting..."
        echo 1 > $CAM1_SYSFS/reset_camera
    fi
    sleep 5
done
```

---

## Device Tree Configuration

The `compatible` string in the camera node must match one of the driver's registered IDs:

```dts
compatible = "covenant,imx219-yuv640";
```

The board root node must match the Jetson variant exactly:

```dts
compatible = "nvidia,p3768-0000+p3767-0005", "nvidia,p3767-0005", "nvidia,tegra234";
```

To verify your running board's compatible string:
```bash
cat /proc/device-tree/compatible | tr '\0' '\n'
```

---

## Verify Camera is Connected

```bash
# Check I2C detection — UU means driver bound, 3c means device present without driver
i2cdetect -y -r 4

# In dual camera mode, check both mux channels
i2cdetect -y -r 5   # CAM0
i2cdetect -y -r 6   # CAM1

# Manually read chip ID register (should return 0x53 0x52)
i2ctransfer -y 4 w2@0x3c 0x00 0x00 r2@0x3c       # single
i2ctransfer -y 5 w2@0x3c 0x00 0x00 r2@0x3c       # CAM0 dual
i2ctransfer -y 6 w2@0x3c 0x00 0x00 r2@0x3c       # CAM1 dual

# Check probe result in kernel log
dmesg | grep -i "ur_thermal\|imx219"
```

---

## Troubleshooting

| Symptom | Likely Cause | Fix |
|---|---|---|
| `UU` in i2cdetect but probe fails | Camera not physically connected or power issue | Run `i2ctransfer` to confirm electrical presence |
| `I2C Read Failed` in dmesg | Wrong bus, bad wiring, or camera not powered | Check bus number, verify with `i2ctransfer` |
| Driver not probing at all | `compatible` string mismatch in DTS | Verify DTS node uses `covenant,imx219-yuv640` |
| DTB overlay not applying | Wrong base DTB path | Confirm the `_COPY.dtb` file exists and matches your board |
| Stream not recovering after reset | GPIO not configured in DTS | Confirm `reset-gpios = <&gpio 0x3e 0x00>` is present |

---

## License

Copyright © 2026 Covenant. All Rights Reserved.

This software and all associated source code, documentation, device tree files, and build scripts (collectively, the "Software") are the exclusive proprietary property of Covenant.

No part of this Software may be reproduced, distributed, modified, reverse engineered, decompiled, or transmitted in any form or by any means — electronic, mechanical, or otherwise — without the prior written permission of Covenant.

Unauthorized use, copying, or disclosure of this Software, in whole or in part, is strictly prohibited and may result in severe civil and criminal penalties.

For licensing inquiries, contact Covenant directly.
