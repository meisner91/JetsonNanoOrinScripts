#!/usr/bin/env bash
set -euo pipefail

# Fully automatic: detect L4T release -> download matching public_sources.tbz2
# -> extract kernel sources -> enable/build ch341 module -> install -> depmod
# -> purge brltty
#
# Tested logic for Jetson Linux R36.x (e.g., R36.4.4):
#   https://developer.download.nvidia.com/embedded/L4T/r36_Release_v4.4/sources/public_sources.tbz2
#
# Usage:
#   chmod +x build_ch341.sh
#   ./build_ch341.sh

log() { echo -e "\n==> $*\n"; }
die() { echo "ERROR: $*" >&2; exit 1; }

# Re-exec as root (so we can install deps, write /usr/src, /lib/modules, purge brltty)
if [[ "${EUID}" -ne 0 ]]; then
  exec sudo -E bash "$0" "$@"
fi

# ---- 1) Detect running kernel + L4T release ----
KREL="$(uname -r)"
LOCALVERSION="-${KREL#*-}"  # e.g. 5.15.148-tegra -> -tegra

log "Detected running kernel: ${KREL}"
log "Using LOCALVERSION: ${LOCALVERSION}"

NV_REL_FILE="/etc/nv_tegra_release"
[[ -f "${NV_REL_FILE}" ]] || die "Missing ${NV_REL_FILE}. Are you on Jetson Linux?"

# Example line:
# # R36 (release), REVISION: 4.4, GCID: ..., BOARD: ..., EABI: ..., DATE: ...
R_MAJOR="$(grep -oE 'R[0-9]+' "${NV_REL_FILE}" | head -n1 | tr -d 'R' || true)"
REVISION="$(grep -oE 'REVISION: *[0-9]+(\.[0-9]+)+' "${NV_REL_FILE}" | head -n1 | awk -F': *' '{print $2}' || true)"

[[ -n "${R_MAJOR}" ]] || die "Could not parse Rxx from ${NV_REL_FILE}"
[[ -n "${REVISION}" ]] || die "Could not parse REVISION from ${NV_REL_FILE}"

REV_TRIM2="$(echo "${REVISION}" | awk -F. '{print $1"."$2}')"

log "Detected Jetson Linux release: R${R_MAJOR}, REVISION ${REVISION} (using v${REV_TRIM2} for download)"

# ---- 2) Install build dependencies ----
log "Installing build dependencies..."
apt-get update -y
apt-get install -y \
  ca-certificates curl \
  build-essential bc flex bison \
  libssl-dev libelf-dev dwarves \
  bzip2 xz-utils tar

# ---- 3) Download public_sources.tbz2 (matching your L4T) ----
WORKDIR="/tmp/ch341_build_$$"
mkdir -p "${WORKDIR}"
cd "${WORKDIR}"

PSRC="public_sources.tbz2"

# NVIDIA R36+ uses developer.download.nvidia.com; capitalization in path matters
URL="https://developer.download.nvidia.com/embedded/L4T/r${R_MAJOR}_Release_v${REV_TRIM2}/sources/public_sources.tbz2"

log "Downloading: ${URL}"
curl -fL --retry 3 --retry-delay 2 -o "${PSRC}" "${URL}" \
  || die "Download failed. Check URL or network."

# Quick sanity check: ensure archive is readable
log "Sanity-checking archive (bzip2 test)..."
bzip2 -t "${PSRC}" || die "Downloaded file appears corrupted (bzip2 test failed). Re-try your network/download."

# ---- 4) Extract kernel_src.tbz2 from public_sources.tbz2 ----
log "Extracting kernel_src.tbz2 from public_sources.tbz2..."
tar -xvf "${PSRC}" "Linux_for_Tegra/source/kernel_src.tbz2" --strip-components=2

[[ -f "kernel_src.tbz2" ]] || die "kernel_src.tbz2 was not extracted. Archive layout may have changed."

# ---- 5) Extract kernel sources into /usr/src/kernel ----
log "Installing kernel sources to /usr/src/kernel ..."
rm -rf /usr/src/kernel
mkdir -p /usr/src

tar -xvf "kernel_src.tbz2" -C /usr/src/

# Find the kernel source directory (typically /usr/src/kernel/kernel-jammy-src on R36)
KSRC_DIR="$(find /usr/src/kernel -maxdepth 2 -type d -name 'kernel-*-src' | head -n1 || true)"
[[ -n "${KSRC_DIR}" ]] || die "Could not find kernel source dir under /usr/src/kernel (expected kernel-*-src)."

log "Kernel source directory: ${KSRC_DIR}"

# ---- 6) Prepare config to match the running kernel ----
log "Copying running kernel config into source tree..."
if [[ -f /proc/config.gz ]]; then
  zcat /proc/config.gz > "${KSRC_DIR}/.config"
else
  die "/proc/config.gz not found. Enable IKCONFIG in your current kernel or provide a config."
fi

log "Setting LOCALVERSION to match uname -r (${LOCALVERSION})..."
"${KSRC_DIR}/scripts/config" --file "${KSRC_DIR}/.config" --set-str LOCALVERSION "${LOCALVERSION}"

log "Preparing kernel tree (olddefconfig, prepare, modules_prepare)..."
make -C "${KSRC_DIR}" olddefconfig
make -C "${KSRC_DIR}" prepare
make -C "${KSRC_DIR}" modules_prepare

# ---- 7) Enable CH341 as a module and build only USB serial directory ----
log "Enabling CONFIG_USB_SERIAL=y and CONFIG_USB_SERIAL_CH341=m ..."
"${KSRC_DIR}/scripts/config" --file "${KSRC_DIR}/.config" --enable USB_SERIAL
"${KSRC_DIR}/scripts/config" --file "${KSRC_DIR}/.config" --module USB_SERIAL_CH341
make -C "${KSRC_DIR}" olddefconfig

log "Building USB serial modules (includes ch341)..."
make -C "${KSRC_DIR}" -j"$(nproc)" M=drivers/usb/serial modules

CHKO="${KSRC_DIR}/drivers/usb/serial/ch341.ko"
[[ -f "${CHKO}" ]] || die "Build finished but ${CHKO} not found."

# ---- 8) Install ch341.ko into the running kernel's module tree ----
log "Installing ch341.ko into /lib/modules/${KREL}/ ..."
install -D -m 644 "${CHKO}" "/lib/modules/${KREL}/kernel/drivers/usb/serial/ch341.ko"

log "Running depmod..."
depmod -a

# Optional: load it now (won't fail the whole script if device isn't present)
log "Trying to load module (modprobe ch341)..."
modprobe ch341 || true

# ---- 9) Purge brltty to avoid grabbing the USB serial device ----
log "Purging brltty (common conflict with CH341 USB serial adapters)..."
apt-get purge -y brltty || true
apt-get autoremove -y || true

log "Done."
echo "Next:"
echo "  - Plug your CH341 device"
echo "  - Check: dmesg | tail -100"
echo "  - Check: ls -l /dev/ttyUSB*"
echo "If you want to reboot now: sudo reboot"
