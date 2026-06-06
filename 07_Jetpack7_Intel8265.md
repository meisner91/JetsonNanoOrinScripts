# WiFi Fix (Intel 8265 on Tegra kernel)

The `iwlwifi` driver is not compiled for the Tegra kernel (`6.8.12-1021-tegra`).
The `backport-iwlwifi-dkms` package can build it, but requires two fixes before
the build will succeed.

## Steps

### 1. Install the DKMS backport

```bash
sudo apt install backport-iwlwifi-dkms
```

### 2. Remove blocking directives from dkms.conf

The package ships with two directives that prevent it from building on this kernel.
There is no DKMS flag to bypass these — the dkms.conf must be edited manually.

- `BUILD_EXCLUSIVE_CONFIG` — a config-presence check that incorrectly fails on this kernel despite all required configs being set
- `OBSOLETE_BY="6.7.0"` — marks the package obsolete for kernels ≥ 6.7, assuming those kernels ship `iwlwifi` built-in; the Tegra kernel 6.8.12 does not

```bash
sudo sed -i '/BUILD_EXCLUSIVE_CONFIG/d' /var/lib/dkms/backport-iwlwifi/11510/source/dkms.conf
sudo sed -i '/^OBSOLETE_BY/d' /var/lib/dkms/backport-iwlwifi/11510/source/dkms.conf
```

### 3. Build and install the module

```bash
sudo dkms build backport-iwlwifi/11510 -k $(uname -r)
sudo dkms install backport-iwlwifi/11510 -k $(uname -r)
```

### 4. Reboot

A reboot is required so the updated `cfg80211` and `mac80211` modules load in the
correct order before `iwlwifi`:

```bash
sudo reboot
```

### 5. Verify

After rebooting, the `wlan0` interface should appear:

```bash
ip link show | grep wlan
```
