# CH341 – Minimal Kernel Module Build on Jetson Orin Nano (Linux) - USB- Driver ch341 - like ESP32

```bash
cd ~/Downloads
wget -O build_ch341.sh \
  https://raw.githubusercontent.com/meisner91/JetsonNanoOrinScripts/main/build_ch341.sh

chmod +x build_ch341.sh
sudo L4T_REV_OVERRIDE=4.4 ./build_ch341.sh

```
