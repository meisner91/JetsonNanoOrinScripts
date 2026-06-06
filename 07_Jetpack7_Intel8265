# WiFi Fix (Intel 8265 on Tegra kernel)

The `iwlwifi` driver is not compiled for the Tegra kernel (`6.8.12-1021-tegra`).
Install the DKMS backport to build it for your kernel:

```bash
sudo apt install backport-iwlwifi-dkms
```

After install, load the driver:

```bash
sudo modprobe iwlwifi
```

Verify the interface is up:

```bash
ip link show | grep wlan
```
