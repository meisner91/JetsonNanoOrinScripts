# CH341 – Minimal Kernel Module Rebuild on Jetson Orin Nano (Linux)

This guide explains how to **rebuild and install only the CH341 USB-Serial driver (`ch341.ko`)**
on a **Jetson Orin Nano / Jetson Nano**,  
**without reflashing JetPack or rebuilding the full kernel image**.

Suitable for:
- Jetson Orin Nano / Jetson Nano
- JetPack 5.x / 6.x (Ubuntu 20.04 / 22.04)
- CH341 / CH340 USB-Serial adapters
- ESP32, Arduino, micro-ROS, UART-based devices

---

## ⚠️ Important Notes

- **Kernel sources must exactly match the running kernel version**
- Mismatched sources will result in:
  ```
  invalid module format
  vermagic mismatch
  ```
- Secure Boot must be **disabled**, or the module must be signed

---

## 1️⃣ Check if a rebuild is actually required

```bash
zcat /proc/config.gz | grep CONFIG_USB_SERIAL_CH341
modinfo ch341
```

Expected:
- `CONFIG_USB_SERIAL_CH341=m` or `=y`
- `modinfo ch341` prints module information

👉 If this is true → **no rebuild is required**

---

## 2️⃣ Identify kernel & Jetson version (critical step)

```bash
uname -r
cat /etc/nv_tegra_release
```

This determines:
- which kernel source branch you need
- which tag (`jetson_XX.Y.Z`) must be checked out

---
# Jetson Linux 36.4.4 BSP + Kernel Sources (R36.4.7)

This guide shows how to obtain and extract the NVIDIA Jetson Linux
Driver Package (BSP) and sync the correct kernel sources for
**Jetson Linux R36.4.7** (matching your `/etc/nv_tegra_release`).

This applies to JetPack **6.2.1 / L4T R36.4.4** systems. :contentReference[oaicite:2]{index=2}

---

## 1) Download the BSP

From the NVIDIA Jetson Linux 36.4.4 release page:

👉 https://developer.nvidia.com/embedded/jetson-linux-r3644

Download:

- **Driver Package (BSP)**  
  `Jetson_Linux_R36.4.4_aarch64.tbz2`  
  (this contains the `Linux_for_Tegra` directory with tools/scripts)

📌 *Do not try to directly paste the URL into the browser — NVIDIA download servers require
click-through from the release page.* :contentReference[oaicite:3]{index=3}

---

## 2) Extract the BSP

Assuming you downloaded it to `~/Downloads`:

```bash
mkdir -p ~/l4t
cd ~/l4t
tar xf ~/Downloads/Jetson_Linux_R36.4.4_aarch64.tbz2

## 3️⃣ Install build dependencies

```bash
sudo apt update
sudo apt install -y \
  build-essential bc flex bison libssl-dev \
  libelf-dev libncurses-dev pkg-config
```

---

## 4️⃣ Obtain matching NVIDIA kernel sources

Recommended (NVIDIA-supported) method:

```bash
cd ~/l4t/Linux_for_Tegra/source
./source_sync.sh -t jetson_36.x.x
```

> ❗ Replace `jetson_36.x.x` with the version shown in `/etc/nv_tegra_release`

Typical kernel source root afterwards:

```text
Linux_for_Tegra/source/public/kernel/kernel-jammy-src
```

---

## 5️⃣ Import the running kernel configuration

```bash
cd <KERNEL_SOURCE_ROOT>
zcat /proc/config.gz > .config
make olddefconfig
```

---

## 6️⃣ Enable CH341 as a loadable module

### Fast method (CLI):
```bash
./scripts/config --module CONFIG_USB_SERIAL_CH341
make olddefconfig
```

### Alternative (interactive):
```bash
make menuconfig
# or (Qt GUI)
make xconfig
```

Menu path:
```
Device Drivers
 └─ USB support
    └─ USB Serial Converter support
       └─ USB CH341 Single Port Serial Driver (M)
```

---

## 7️⃣ **Build only the CH341 module**

```bash
make -j$(nproc) M=drivers/usb/serial modules
```

Result:
```bash
drivers/usb/serial/ch341.ko
```

---

## 8️⃣ Install the module (no kernel flash)

```bash
KVER=$(uname -r)
MODDIR=/lib/modules/$KVER/kernel/drivers/usb/serial

sudo mkdir -p $MODDIR

# Backup existing module (if any)
sudo cp $MODDIR/ch341.ko $MODDIR/ch341.ko.bak 2>/dev/null || true

# Copy new module
sudo cp drivers/usb/serial/ch341.ko $MODDIR/

# Update module dependency database
sudo depmod -a
```

---

## 9️⃣ Reload the module

```bash
sudo modprobe -r ch341
sudo modprobe ch341
```

---

## 🔟 Test

```bash
dmesg | tail -n 50
ls -l /dev/ttyUSB*
```

Expected:
```
/dev/ttyUSB0
```

---

## 🧠 Common Issues & Fixes

### ❌ `invalid module format`
➡️ Kernel sources do not match the running kernel

### ❌ No `/dev/ttyUSB0`
➡️ User not in `dialout` group:
```bash
sudo usermod -aG dialout $USER
sudo reboot
```

### ❌ Unstable USB detection
- use a different USB port
- use a powered USB hub
- use a proper data cable (not charge-only)

---

## ✅ Result

- No kernel flash
- No JetPack reinstallation
- Only `ch341.ko` replaced
- Safe for development and updates

---

## 🔗 Typical Use Cases

- ESP32 + micro-ROS
- Arduino
- UART debugging
- Motor controllers
- Sensor interfaces

---

**Author:**  
Jetson Linux / Embedded Linux Guide  
(optimized for Jetson Orin Nano)
