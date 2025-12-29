#!/usr/bin/env bash
set -euo pipefail

log() { echo -e "\n==> $*\n"; }
die() { echo "ERROR: $*" >&2; exit 1; }

# Re-run as root
if [[ "${EUID}" -ne 0 ]]; then
  exec sudo -E bash "$0" "$@"
fi

# ------------------------------------------------------------
# 1) Detect kernel + Jetson Linux version
# ------------------------------------------------------------
KREL="$(uname -r)"
LOCALVERSION="-${KREL#*-}"

log "Kernel: ${KREL}"
log "LOCALVERSION: ${LOCALVERSION}"

NV_REL="/etc/nv_tegra_release"
[[ -f "${NV_REL}" ]] || die "Not a Jetson system"

R_MAJOR="$(grep -oE 'R[0-9]+' "${NV_REL}" | tr -d 'R')"
REVISION="$(grep -oE 'REVISION: *[0-9]+(\.[0-9]+)+' "${NV_REL}" | awk -F': *' '{print $2}')"
REV_TRIM2="$(echo "${REVISION}" | awk -F. '{print $1"."$2}')"

# Optional override
if [[ -n "${L4T_REV_OVERRIDE:-}" ]]; then
  log "Forcing L4T revision to ${L4T_REV_OVERRIDE}"
  REV_TRIM2="${L4T_REV_OVERRIDE}"
fi

log "Jetson Linux: R${R_MAJOR} v${REV_TRIM2}"

# ------------------------------------------------------------
# 2) Dependencies
# ------------------------------------------------------------
log "Installing dependencies..."
apt-get update -y
apt-get install -y \
  curl ca-certificates \
  build-essential bc flex bison \
  libssl-dev libelf-dev dwarves \
  bzip2 xz-utils tar \
  nvidia-l4t-kernel-headers || true

# ------------------------------------------------------------
# 3) Download public_sources.tbz2
# ------------------------------------------------------------
WORKDIR="/tmp/ch341_build"
rm -rf "${WORKDIR}"
mkdir -p "${WORKDIR}"
cd "${WORKDIR}"

URL="https://developer.download.nvidia.com/embedded/L4T/r${R_MAJOR}_Release_v${REV_TRIM2}/sources/public_sources.tbz2"

log "Downloading kernel sources:"
log "${URL}"

curl -fL --retry 3 -o public_sources.tbz2 "${URL}" \
  || die "Download failed"

[[ "$(stat -c%s public_sources.tbz2)" -gt 1000000 ]] \
  || die "Downloaded file too small"

bzip2 -t public_sources.tbz2

# ------------------------------------------------------------
# 4) Extract kernel sources
# ------------------------------------------------------------
log "Extracting kernel_src.tbz2..."
tar -xvf public_sources.tbz2 Linux_for_Tegra/source/kernel_src.tbz2 --strip-components=2

rm -rf /usr/src/kernel
mkdir -p /usr/src
tar -xvf kernel_src.tbz2 -C /usr/src/

KSRC="$(find /usr/src/kernel -maxdepth 2 -type d -name 'kernel-*-src' | head -n1)"
[[ -n "${KSRC}" ]] || die "Kernel source dir not found"

log "Kernel source dir: ${KSRC}"

# ------------------------------------------------------------
# 5) Prepare kernel config
# ------------------------------------------------------------
log "Preparing kernel config..."
zcat /proc/config.gz > "${KSRC}/.config"

"${KSRC}/scripts/config" --file "${KSRC}/.config" --set-str LOCALVERSION "${LOCALVERSION}"

# IMPORTANT FIX
"${KSRC}/scripts/config" --file "${KSRC}/.config" --module USB_SERIAL
"${KSRC}/scripts/config" --file "${KSRC}/.config" --module USB_SERIAL_CH341

make -C "${KSRC}" olddefconfig
make -C "${KSRC}" prepare
make -C "${KSRC}" modules_prepare

# Copy Module.symvers if available
if [[ -f "/lib/modules/${KREL}/build/Module.symvers" ]]; then
  cp /lib/modules/${KREL}/build/Module.symvers "${KSRC}/Module.symvers"
fi

# ------------------------------------------------------------
# 6) Locate + build CH341
# ------------------------------------------------------------
log "Locating ch341.c..."
CH341_C="$(find "${KSRC}" -name ch341.c | head -n1)"
[[ -n "${CH341_C}" ]] || die "ch341.c not found"

CH341_DIR="$(dirname "${CH341_C}" | sed "s|^${KSRC}/||")"
log "Building CH341 in: ${CH341_DIR}"

make -C "${KSRC}" -j"$(nproc)" M="${CH341_DIR}" modules

CH341_KO="$(find "${KSRC}/${CH341_DIR}" -name ch341.ko | head -n1)"
[[ -n "${CH341_KO}" ]] || die "ch341.ko not produced"

# ------------------------------------------------------------
# 7) Install module
# ------------------------------------------------------------
DEST="/lib/modules/${KREL}/kernel/${CH341_DIR}/ch341.ko"
log "Installing to ${DEST}"

install -D -m 644 "${CH341_KO}" "${DEST}"
depmod -a
modprobe ch341 || true

# ------------------------------------------------------------
# 8) Remove brltty
# ------------------------------------------------------------
log "Removing brltty..."
apt-get purge -y brltty || true
apt-get autoremove -y || true

log "DONE"
echo "Plug CH341 device and check:"
echo "  dmesg | tail -100"
echo "  ls /dev/ttyUSB*"
