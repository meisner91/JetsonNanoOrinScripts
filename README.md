# Nvidia Jetson Nano Orin Scripts
Helpful Scripts for Nvidia Jetson Nano Super

## 00 - Jetson Nano Orin - SNAPD Issue 
Apps like Firefox or Chromium not starting anymore use an older Snapd Version
[00_Jetson_Setup_SNAPD](https://github.com/meisner91/JetsonNanoOrinScripts/blob/main/00_Jetson_Setup_SNAPD.sh)

## 01_Nvidia_Jetson_Nano_Orin_Isaac_ROS_(Humble)

## 02_Install_Jetson_PS4Controller_Humble

## 03_Install_Intel_RealSense_on_JetPack_6

## 04_Run_Realsense_RVIZ2

## 05_CH341_Jetson_Minimal_Kernel_Build

# Ros2 Pixi Guide
# ROS 2 on Windows with pixi – `ros2pixi.ps1` helper

This guide explains how to place the `ros2pixi.ps1` helper script, make it easily accessible via `PATH`, and run ROS 2 commands (example: **talker**) in a stable pixi environment.

---

## Prerequisites

- Windows 11  
- ROS 2 installed via official Windows binaries  
  https://docs.ros.org/en/kilted/Installation/Windows-Install-Binary.html
- `pixi` installed and working
- pixi workspace (example):
  ```
  C:\pixi_ws
  ```
- ROS 2 setup script:
  ```
  C:\pixi_ws\ros2-windows\local_setup.bat
  ```

---

## 1. Create `ros2pixi.ps1`

Choose a permanent location for the script, for example:

```
C:\ros2_ws\ros2pixi.ps1
```

Paste the following content into the file:

```powershell
param(
    [Parameter(ValueFromRemainingArguments = $true)]
    [string[]] $Args
)

$ws = "C:\pixi_ws"
$rosSetup = "C:\pixi_ws\ros2-windows\local_setup.bat"

if (!(Test-Path $ws)) {
    Write-Error "Workspace not found: $ws"
    exit 1
}
if (!(Test-Path $rosSetup)) {
    Write-Error "ROS setup not found: $rosSetup"
    exit 1
}

Set-Location $ws

if ($Args.Count -eq 0) {
    Write-Host "Usage examples:"
    Write-Host "  ros2pixi ros2 --help"
    Write-Host "  ros2pixi ros2 run demo_nodes_cpp talker"
    exit 0
}

$joined = ($Args | ForEach-Object {
    $_.Replace('"', '\"')
}) -join ' '

$cmd = "call `"$rosSetup`" && $joined"
pixi run -- cmd /c $cmd
```

---

## 2. Add script directory to PATH

Add the directory containing `ros2pixi.ps1` (e.g. `C:\ros2_ws`) to your **user PATH**:

1. Open **System Properties**
2. Click **Environment Variables**
3. Under **User variables**, select **Path**
4. Click **Edit** → **New**
5. Add:
   ```
   C:\ros2_ws
   ```
6. Confirm and close all dialogs
7. Restart PowerShell

---

## 3. Unblock the script (if required)

If PowerShell blocks the script:

```powershell
Unblock-File C:\ros2_ws\ros2pixi.ps1
```

---

## 4. Usage examples

### Show ROS 2 help
```powershell
ros2pixi ros2 --help
```

### Run the demo talker
```powershell
ros2pixi ros2 run demo_nodes_cpp talker
```

### Run the demo listener (second terminal)
```powershell
ros2pixi ros2 run demo_nodes_cpp listener
```

Expected output:
```
[talker]: Publishing: 'Hello World: 1'
[talker]: Publishing: 'Hello World: 2'
...
```

---

## Notes

- `pixi run` is used to avoid subshell issues on Windows.
- RTI Connext DDS warnings are normal if RTI is not installed.
- All ROS 2 commands executed via `ros2pixi` run in a consistent environment.

---

## Summary

✔ No manual `pixi shell`  
✔ No DLL or PATH issues  
✔ Works from any directory  
✔ Reproducible ROS 2 setup on Windows
