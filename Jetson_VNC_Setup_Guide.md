# 🖥️ Jetson Orin Nano — Full VNC Setup Guide (Ubuntu 22.04 / JetPack 6.x)

This guide explains how to install and configure a **VNC server** on your NVIDIA Jetson Orin Nano that provides access to the **real desktop session** (not a separate XFCE virtual desktop).  
After following these steps, you can connect via any VNC viewer to **`<Jetson-IP>:5900`** and see exactly what’s on the HDMI screen.

---

## 🚀 1. Update and Install Packages

```bash
sudo apt update
sudo apt install -y     tightvncserver     tigervnc-standalone-server tigervnc-viewer     xfce4 xfce4-goodies xorg dbus-x11 x11-xserver-utils     x11vnc     nano wget xz-utils
```

---

## 🧭 2. (If using GDM3) Disable Wayland

Wayland must be disabled so x11vnc can access the X display.

```bash
sudo nano /etc/gdm3/custom.conf
```

Uncomment or add:
```
WaylandEnable=false
```

Save → `Ctrl + O`, Enter → `Ctrl + X`, then:
```bash
sudo reboot
```

---

## 🧩 3. Create a Working XFCE xstartup (for virtual VNC sessions)

(Optional but useful if you want an additional virtual desktop.)

```bash
mkdir -p ~/.vnc
cat > ~/.vnc/xstartup <<'EOF'
#!/bin/sh
unset SESSION_MANAGER
unset DBUS_SESSION_BUS_ADDRESS
[ -f "$HOME/.Xresources" ] && xrdb "$HOME/.Xresources"
exec startxfce4
EOF
chmod +x ~/.vnc/xstartup
```

Start a virtual VNC session if needed:
```bash
vncserver -localhost no -geometry 1920x1080 -depth 24 :2
```
→ connect to `<Jetson-IP>:5902`.

---

## 🧰 4. Find Your Real Desktop Display and Auth File

Check the running Xorg process:

```bash
ps -ef | grep Xorg
```

Example output:
```
root ... /usr/lib/xorg/Xorg vt2 -displayfd 3 -auth /run/user/1000/gdm/Xauthority ...
```

Record:
- **Display:** `:1` or `:0`  
- **Auth file:** `/run/user/1000/gdm/Xauthority`

---

## 🔧 5. Test x11vnc Manually

```bash
x11vnc -auth /run/user/1000/gdm/Xauthority -display :1 -forever -nopw -shared
```

If you see:
```
The VNC desktop is: ubuntu:0
PORT=5900
```
✅ You’re live! Connect from another computer:
```
<Jetson-IP>:5900
```

---

## ⚙️ 6. Create a Systemd Service for Auto-Start

```bash
sudo nano /etc/systemd/system/x11vnc.service
```

Paste:
```ini
[Unit]
Description=x11vnc server (mirror real desktop)
After=display-manager.service network.target
Requires=display-manager.service

[Service]
Type=simple
ExecStartPre=/usr/bin/sleep 20
ExecStart=/usr/bin/x11vnc -auth /run/user/1000/gdm/Xauthority -display :1 -forever -nopw -shared
Restart=on-failure
RestartSec=5
User=morris

[Install]
WantedBy=multi-user.target
```

Save and enable:
```bash
sudo systemctl daemon-reload
sudo systemctl enable x11vnc
sudo systemctl restart x11vnc
sudo systemctl status x11vnc
```

If status shows `active (running)`, you’re done.

---

## 🔒 7. (Recommended) Add a Password

```bash
x11vnc -storepasswd
```

Then edit the service’s `ExecStart` line:

```ini
ExecStart=/usr/bin/x11vnc -auth /run/user/1000/gdm/Xauthority -display :1 -forever -rfbauth /home/morris/.vnc/passwd -shared
```

Reload and restart:
```bash
sudo systemctl daemon-reload
sudo systemctl restart x11vnc
```

---

## 🧠 8. Usage Summary

| Type | Display | Port | Connect With | Description |
|------|----------|------|--------------|--------------|
| **Real Desktop** | `:1` | **5900** | `<Jetson-IP>:5900` | Mirrors HDMI desktop (x11vnc) |
| **Virtual XFCE** | `:2` | **5902** | `<Jetson-IP>:5902` | Optional separate session (TigerVNC) |

---

## 🧾 9. Logs and Maintenance

- Check service status  
  ```bash
  sudo systemctl status x11vnc
  ```
- View logs  
  ```bash
  sudo cat /var/log/x11vnc.log
  ```
- Stop or restart service  
  ```bash
  sudo systemctl stop x11vnc
  sudo systemctl restart x11vnc
  ```

---

## ✅ Result

After boot, your Jetson automatically launches **x11vnc**, and you can connect to  
```
<Jetson-IP>:5900
```
to view and control the **actual desktop session (`ubuntu:0`)** from any VNC viewer.
