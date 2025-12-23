
# Intel RealSense on JetPack 6 (Ubuntu 22.04)

This guide explains how to install **Intel RealSense SDK (librealsense)** and the **ROS 2 RealSense wrapper (realsense-ros)** on **NVIDIA Jetson devices running JetPack 6 / Ubuntu 22.04**.

> ⚠️ **Important JetPack 6 Note**  
> On JetPack 6, **RSUSB backend (no kernel patches)** is strongly recommended.  
> Kernel patching often causes USB and device detection issues on Jetson.

---

## System Requirements

- NVIDIA Jetson (Orin Nano / Xavier / Nano)
- JetPack 6.x
- Ubuntu 22.04
- Intel RealSense camera (D435, D455, etc.)
- ROS 2 Humble (optional, for ROS integration)

---

## 1. Update System

```bash
sudo apt update
sudo apt upgrade -y
```

Reboot if the kernel was updated.

---

## 2. Install Dependencies

```bash
sudo apt install -y \
    git cmake build-essential \
    libssl-dev libusb-1.0-0-dev \
    pkg-config libgtk-3-dev \
    libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev \
    python3 python3-dev python3-pip
```

---

## 3. Clone librealsense (RealSense SDK)

```bash
cd ~
git clone https://github.com/IntelRealSense/librealsense.git
cd librealsense
```

### ⚠️ Version Pinning (IMPORTANT – prevents future breakage)

To avoid future updates breaking camera detection, **DO NOT use `master`**.
Pin librealsense to a known stable version and keep it fixed.

**Recommended versions for JetPack 6:**
- **v2.55.1** (most stable on Jetson)
- **v2.56.4** (works, but stricter about USB / permissions)

```bash
git fetch --tags
git checkout v2.55.1
```

🔒 Optional: prevent accidental git updates
```bash
git status
```
(Do not run `git pull` unless you explicitly want to upgrade.)

---

## 4. Install udev Rules (Required)

This allows access to the camera without root.

```bash
sudo ./scripts/setup_udev_rules.sh
```

Reload rules:

```bash
sudo udevadm control --reload-rules && sudo udevadm trigger
```

---

## 5. Build librealsense with RSUSB Backend (JetPack 6 Recommended)

### Create build directory

```bash
mkdir build && cd build
```

### Configure (RSUSB enabled, kernel patches disabled)

```bash
cmake .. \
  -DFORCE_RSUSB_BACKEND=ON \
  -DBUILD_WITH_CUDA=ON \
  -DBUILD_EXAMPLES=ON \
  -DBUILD_GRAPHICAL_EXAMPLES=ON \
  -DBUILD_PYTHON_BINDINGS=ON
```

> ✅ `FORCE_RSUSB_BACKEND=ON` is **mandatory** on JetPack 6

### Compile and install

```bash
make -j$(nproc)
sudo make install
```

Refresh linker cache:

```bash
sudo ldconfig
```
---

## 6. Test RealSense SDK (Non-ROS)

Plug in your RealSense camera and run:

```bash
realsense-viewer
```

You should see:
- Color stream
- Depth stream
- IMU (if supported)

### Common Issues

| Problem | Fix |
|------|-----|
| Device not detected | Use RSUSB backend |
| Viewer crashes | Try USB 3 port, good cable |
| Permission denied | Re-run udev rules |

---

## 7. Install ROS 2 Humble (If Not Installed)

If ROS 2 Humble is not installed yet:

```bash
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update
sudo apt install ros-humble-desktop -y
```

Source ROS:

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

## 8. Install RealSense ROS 2 Wrapper (Binary Packages)

Intel provides **prebuilt ROS 2 packages** for Humble.

```bash
sudo apt update
sudo apt install ros-humble-realsense2-* -y
```

### 🔒 Prevent ROS updates from replacing librealsense

ROS ships its **own librealsense**. To avoid conflicts:

```bash
sudo apt-mark hold ros-humble-realsense2-* librealsense2*
```

This ensures updates will **not overwrite your working setup**.

Verify installation:

```bash
ros2 pkg list | grep realsense
```

---

## 9. Launch RealSense ROS 2 Node

```bash
ros2 launch realsense2_camera rs_launch.py
```

### Common ROS Topics

```bash
/color/image_raw
/depth/image_rect_raw
/camera/camera_info
/imu
```

Check with:

```bash
ros2 topic list
```

---

## 10. Verify Camera Data in ROS

Visualize camera streams:

```bash
rviz2
```

Add:
- Image → `/color/image_raw`
- Image → `/depth/image_rect_raw`

---

## 11. Known JetPack 6 Issues & Workarounds

| Issue | Cause | Solution |
|-----|------|---------|
| Camera disappears after update | librealsense version changed | Pin git tag + `apt-mark hold` |
| RGB stream fails | RGB8 not exposed on Jetson | Use **YUYV**, convert in software |
| IMU disabled | HID interface unavailable via RSUSB | Normal on Jetson; see note below |
| Viewer shows no camera | librealsense library conflict | Enforce `/usr/local/lib` via `LD_LIBRARY_PATH` |
| Device only works with sudo | Permissions | udev rules + `plugdev`, `video` groups |
| Camera unstable | USB power / cable | USB3 port + powered hub |

---

## 12. realsense-viewer Does Not Show Camera (Library Conflict)

Even when **USB 3 is correctly negotiated**, `realsense-viewer` may still **not show the camera** on JetPack 6.

### Root Cause: librealsense Library Conflict

JetPack + ROS installs **two different librealsense versions**:

- ROS-shipped librealsense:
  ```
  /opt/ros/humble/lib/
  ```
- Manually built librealsense (RSUSB-enabled):
  ```
  /usr/local/lib/
  ```

If `realsense-viewer` or `rs-enumerate-devices` links against the **ROS-shipped librealsense**, the result is often:

- `bad optional access`
- device creation failure
- viewer opens but shows **no camera**

This is a **very common Jetson issue**.

---

### Fix: Force realsense-viewer to Use the Correct librealsense

#### 1️⃣ Check Which Library Is Used

```bash
ldd $(which realsense-viewer) | grep realsense
```

❌ If you see:
```
/opt/ros/humble/lib/.../librealsense2.so
```
then the **wrong library** is being used.

---

#### 2️⃣ Force the Correct librealsense (Temporary Fix)

```bash
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
ldd $(which realsense-viewer) | grep realsense
```

✅ Expected output:
```
/usr/local/lib/librealsense2.so.2.56.4
```

Now start the viewer:

```bash
realsense-viewer
```

👉 In most Jetson cases, **the camera appears immediately** after this step.

---

#### 3️⃣ Make the Fix Permanent (Recommended)

Add this line **above ROS sourcing** in `~/.bashrc`:
open
```bash
nano ~/.bashrc
```
add this:
```bash
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH
```

Reload the shell:

```bash
source ~/.bashrc
```

This ensures:
- `realsense-viewer`
- `rs-enumerate-devices`
- `realsense2_camera` (ROS)

always use the **correct RSUSB-enabled librealsense**.

---
--|------|---------|
| Camera disappears after update | librealsense version changed | Pin git tag + `apt-mark hold` |
| RGB stream fails | RGB8 not exposed on Jetson | Use **YUYV**, convert in software |
| IMU disabled | HID interface unavailable via RSUSB | Normal on Jetson; see note below |
| Viewer works, ROS does not | Library conflict (/opt/ros vs /usr/local) | Enforce `/usr/local` via `LD_LIBRARY_PATH` |
| Device only works with sudo | Permissions | udev rules + `plugdev`, `video` groups |
| Camera unstable | USB power / cable | USB3 port + powered hub |

### ℹ️ IMU (D435i) Notes

- D435i IMU uses **HID** interface
- On Jetson + RSUSB, IMU may be:
  - Disabled
  - Unstable
  - Missing in ROS

This is **expected behavior** on many Jetson systems.

✅ Depth + Color are fully supported and sufficient for:
- Navigation
- Obstacle avoidance
- SLAM

If IMU is critical, consider:
- External IMU
- Fusing with robot IMU instead

----|----|
| Viewer shows no streams | Use RSUSB backend |
| USB disconnects | Use powered USB hub |
| High CPU usage | Lower FPS / resolution |
| ROS node not starting | Check `realsense-viewer` first |

---

## 12. Useful References

- Intel librealsense GitHub  
  https://github.com/IntelRealSense/librealsense
- RealSense ROS Wrapper  
  https://github.com/IntelRealSense/realsense-ros
- NVIDIA Jetson Forums (RealSense)  
  https://forums.developer.nvidia.com/

---

## 13. Common Problems & Fixes

### Problem: RealSense Viewer says "UDEV-Rules are missing"

**Symptoms:**
- `realsense-viewer` starts but shows no camera
- Warning: *RealSense UDEV-Rules are missing*

**Fix:**
1. Ensure the udev rules file exists:

```bash
ls -l /etc/udev/rules.d/ | grep -i realsense
```

2. If missing, copy it manually from the librealsense repository:

```bash
cd ~/librealsense
sudo cp ./config/99-realsense-libusb.rules /etc/udev/rules.d/
sudo chmod 644 /etc/udev/rules.d/99-realsense-libusb.rules
```

3. Reload udev rules and replug the camera:

```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
```

4. Add user to required groups and re-login:

```bash
sudo usermod -aG plugdev,video $USER
```

---

### Problem: RGB stream not working ("Failed to resolve request: RGB8")

**Typical error message:**
```
Invalid Value in rs2_open_multiple(...)
Requested: RGB8 1280x720
Available: YUYV
```

**Cause:**
On JetPack 6 (RSUSB backend), the RealSense D435/D435i usually exposes the **color stream as YUYV**, not RGB8. Requesting RGB8 causes the stream negotiation to fail.

**Fix (Recommended):**
- Use **YUYV** as the color format
- Let the application/viewer convert to RGB if needed

**In realsense-viewer:**
- Color → Format: `YUYV`
- Resolution: `1280x720`
- FPS: `30`

**In ROS 2:**
```bash
ros2 launch realsense2_camera rs_launch.py rgb_camera.profile:=1280x720x30
```

Do **not** force RGB8. Convert to RGB downstream if required.

---

### Problem: Camera detected only with sudo

**Cause:**
Missing permissions or udev rules not applied

**Fix:**
- Reinstall udev rules
- Reload udev
- Re-login or reboot

---

### Problem: Camera not detected at all

**Checks:**
```bash
lsusb | grep -i intel
lsusb -t
```

Ensure the camera is connected via **USB 3 (5000M)**.
Use a **good USB 3 cable** or a **powered USB hub**.

---

## Summary

✅ **Stable, update-safe JetPack 6 setup**

- librealsense **pinned to a fixed git tag** (no master)
- RSUSB backend **forced**
- Kernel patching **disabled**
- ROS packages **held** to prevent overwrites
- RGB via **YUYV** (not RGB8)
- Depth fully supported
- IMU support limited on Jetson (expected)

This configuration is **proven stable** on:
- Jetson Orin Nano
- JetPack 6
- RealSense D435i

🚫 Avoid system updates that touch librealsense unless you intentionally want to upgrade.
