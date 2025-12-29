# CH341 – Minimal Kernel Module Build on Jetson Orin Nano (Linux)

This document describes **how the CH341 USB-Serial kernel driver** (commonly used by ESP32,
Arduino clones, and USB-UART adapters) is built and installed on a **Jetson Orin Nano**.

The process uses **official NVIDIA kernel sources** and builds **only the required kernel module**  
— no full kernel rebuild, no flashing, no root filesystem changes.

---

## 🧩 System Overview

- Device: **Jetson Orin Nano**
- Kernel: `5.15.148-tegra`
- Jetson Linux (L4T): **R36.x**
- Driver: **CH341 USB-Serial**

---

## 📥 Where the Kernel Sources Come From (NVIDIA)

NVIDIA publishes kernel sources separately from the root filesystem.

For Jetson Linux **R36.4.4**, the official source archive is:

```
https://developer.download.nvidia.com/embedded/L4T/r36_Release_v4.4/sources/public_sources.tbz2
```

### Important notes
- The host **must** be `developer.download.nvidia.com`
- The path is **case-sensitive**
- The file is several **gigabytes**
- This archive contains **exactly the kernel sources NVIDIA used**

Inside this archive, the script uses **only**:

```
Linux_for_Tegra/source/kernel_src.tbz2
```

---

## ▶️ How to Use the Script

### 1. Download the script
```bash
cd ~/Downloads

wget -O build_ch341.sh \
  https://raw.githubusercontent.com/meisner91/JetsonNanoOrinScripts/main/build_ch341.sh
```

### 2. Make it executable
```bash
chmod +x build_ch341.sh
```

### 3. Run it
```bash
sudo L4T_REV_OVERRIDE=4.4 ./build_ch341.sh
```

`L4T_REV_OVERRIDE=4.4` forces the correct NVIDIA source version  
(needed when the system reports a newer minor revision).

---

## 🧠 What the Script Does (Step by Step)

### 1️⃣ Detects the Running Kernel

```bash
uname -r
```

Example:
```
5.15.148-tegra
```

This ensures the module is built **exactly** for the running kernel.

---

### 2️⃣ Detects the Jetson Linux Version (L4T)

```bash
/etc/nv_tegra_release
```

Example:
```
R36 (release), REVISION: 4.4
```

This information determines **which NVIDIA kernel sources are downloaded**.

---

### 3️⃣ Installs Required Build Dependencies

The script installs only what is needed to build a kernel module:

- compiler toolchain (`gcc`, `make`)
- kernel build tools (`bc`, `flex`, `bison`)
- crypto & ELF support (`libssl-dev`, `libelf-dev`)
- `nvidia-l4t-kernel-headers` (for symbol compatibility)

No full BSP or SDK installation is required.

---

### 4️⃣ Downloads NVIDIA Kernel Sources

The script downloads:

```
public_sources.tbz2
```

from the official NVIDIA server and verifies that:
- the file is large enough
- the archive is valid (`bzip2 -t`)

---

### 5️⃣ Extracts Only the Kernel Source Tree

From the large archive, only this file is extracted:

```
kernel_src.tbz2
```

It is unpacked to:

```
/usr/src/kernel/kernel-jammy-src
```

No root filesystem or flashing tools are touched.

---

### 6️⃣ Synchronizes Kernel Configuration

The exact running kernel configuration is copied:

```bash
zcat /proc/config.gz > .config
```

The kernel version suffix is set:
```
LOCALVERSION = -tegra
```

---

### 7️⃣ Critical Configuration Fix (Very Important)

```text
CONFIG_USB_SERIAL=m
CONFIG_USB_SERIAL_CH341=m
```

Why this matters:

- A built-in driver (`=y`) **cannot** have a module child (`=m`)
- If `USB_SERIAL=y`, **`ch341.ko` will never be built**
- This script fixes that automatically

---

### 8️⃣ Prepares the Kernel Build Environment

```bash
make olddefconfig
make prepare
make modules_prepare
```

If available, `Module.symvers` is copied from the installed kernel headers
to avoid unresolved symbol warnings.

---

### 9️⃣ Locates the CH341 Source Automatically

The script searches for:
```
ch341.c
```

Example:
```
drivers/usb/serial/ch341.c
```

This makes the script robust against kernel layout changes.

---

### 🔟 Builds Only the CH341 Module

```bash
make M=drivers/usb/serial modules
```

- No full kernel build
- Only the CH341 driver is compiled
- Build time is short

Result:
```
ch341.ko
```

---

### 1️⃣1️⃣ Installs the Module

The module is installed to:

```
/lib/modules/$(uname -r)/kernel/drivers/usb/serial/ch341.ko
```

Then:
```bash
depmod -a
modprobe ch341
```

---

### 1️⃣2️⃣ Removes `brltty` (Important)

`brltty` often hijacks USB-Serial devices.

The script removes it to prevent conflicts:
```bash
apt purge brltty
```

---

## ✅ Final Result

After the script finishes:

```bash
ls /dev/ttyUSB*
```

Expected:
```
/dev/ttyUSB0
```

ESP32 / CH341 devices are now usable.

---

## 🧾 Summary

✔ Uses **official NVIDIA kernel sources**  
✔ Builds **only** the required kernel module  
✔ No kernel flashing  
✔ No reboot required  
✔ Safe and repeatable  

---

## 🚀 Optional Next Steps

- Auto-load `ch341` at boot
- Create a `.deb` package
- Add DKMS support
- Add udev rules for stable device names
