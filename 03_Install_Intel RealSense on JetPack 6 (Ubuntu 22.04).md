# Intel RealSense on JetPack 6 (Ubuntu 22.04)

This guide explains how to install **Intel RealSense SDK (librealsense)** and the **ROS 2 RealSense wrapper (realsense-ros)** on **NVIDIA Jetson devices running JetPack 6 / Ubuntu 22.04**.

> ⚠️ **Important JetPack 6 Note**
> On JetPack 6, **RSUSB backend (no kernel patches)** is strongly recommended.
> Kernel patching often causes USB and device detection issues on Jetson.

---

## System Requirements

* NVIDIA Jetson (Orin Nano / Xavier / Nano)
* JetPack 6.x
* Ubuntu 22.04
* Intel RealSense camera (D435, D455, etc.)
* ROS 2 Humble (optional, for ROS integration)

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

(Optional but recommended: checkout a stable release)

```bash
git tag
git checkout v2.55.1
```

---

## 4.0 Install udev Rules (Required)
### Install v4l-utils
```bash
sudo apt update
sudo apt install v4l-utils -y
```
Reload rules
```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
```
Now you should see the camera
```bash
v4l2-ctl --list-devices
```

## 4.1 Install udev Rules (Required)

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
  -DBUILD_GRAPHICAL_EXAMPLES=ON
```

> ✅ `FORCE_RSUSB_BACKEND=ON` is **critical** for JetPack 6 stability.

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

* Color stream
* Depth stream
* IMU (if supported)

### Common Issues

| Problem             | Fix                        |
| ------------------- | -------------------------- |
| Device not detected | Use RSUSB backend          |
| Viewer crashes      | Try USB 3 port, good cable |
| Permission denied   | Re-run udev rules          |

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

* Image → `/color/image_raw`
* Image → `/depth/image_rect_raw`

---

## 11. Known JetPack 6 Issues & Workarounds

| Issue                   | Solution                       |
| ----------------------- | ------------------------------ |
| Viewer shows no streams | Use RSUSB backend              |
| USB disconnects         | Use powered USB hub            |
| High CPU usage          | Lower FPS / resolution         |
| ROS node not starting   | Check `realsense-viewer` first |

---

## 12. Useful References

* Intel librealsense GitHub
  [https://github.com/IntelRealSense/librealsense](https://github.com/IntelRealSense/librealsense)
* RealSense ROS Wrapper
  [https://github.com/IntelRealSense/realsense-ros](https://github.com/IntelRealSense/realsense-ros)
* NVIDIA Jetson Forums (RealSense)
  [https://forums.developer.nvidia.com/](https://forums.developer.nvidia.com/)

---

## Summary

✅ **Recommended for JetPack 6**

* Build librealsense from source
* Use **RSUSB backend**
* Avoid kernel patching
* Use ROS 2 Humble binary packages

This setup is **stable and widely used on Jetson Orin Nano**.
