#!/usr/bin/env bash
set -euo pipefail

# Fully automatic:
#   detect L4T release -> download matching public_sources.tbz2
#   -> extract kernel sources -> enable/build ch341 -> install -> depmod
#   -> purge brltty
#
# Optional override:
#   L4T_REV_OVERRIDE=4.4 ./build_ch341.sh
#
# Usage:
#   chmod +x build_ch341.sh
#   ./build_ch341.sh

log() { echo -e "\n==> $*\n"; }
die() { echo "ERROR: $*" >&2; exit 1; }

# Re-exec as root
if [[ "${EUID}" -ne 0 ]]; then
  exec sudo -E bash "$0" "$@"
fi

# ---- 1) Detect running kernel + L4T release ----
KREL="$(uname -r)"
LOCALVERSION="-${KREL#*-}"  # 5.15.148-tegra -> -tegra

log "Detected running kernel: ${KREL}"
log "Using LOCALVERSION: ${LOCALVERSION}"

NV_REL_FILE="/etc/nv_tegra_release"
[[ -f "${NV_REL_FILE}" ]] || die "Missing ${NV_REL_FILE}. Are you on Jetson Linux?"

R_MAJOR="$(grep -oE 'R[0-9]+' "${NV_REL_FILE}" | head -n1 | tr -d 'R' || true)"
REVISION="$(grep -oE 'REVISION: *[0-9]+(\.[0-9]+)+' "${NV_REL_FILE}" | head -n1 | awk -F': *' '{print $2}' || true)"

[[ -n "${R_MAJOR}" ]] || die "Could not parse Rxx from ${NV_REL_FILE}"
[[ -n "${REVISION}" ]] || die "Could not parse REVISION from ${NV_REL_FILE}"

# Trim revision to major.minor (e.g., 4.7.1 -> 4.7)
REV_TRIM2="$(echo "${REVISION}" | awk -F. '{print $1"."$2}')"

# Allow forcing the revision (e.g. user wants 4.4)
if [[ -n "${L4T_REV_OVERRIDE:-}" ]]; then
  log "L4T_REV_OVERRIDE is set -> forcing revision to: ${L4T_REV_OVERRIDE}"
  REV_TRIM2="${L4T_REV_OVERRIDE}"
fi

log "Detected Jetson Linux: R${R_MAJOR}, REVISION ${REVISION} (download v${REV_TRIM2})"

# ---- 2) Install build dependencies ----
log "Installing build dependencies..."
apt-get update -y
apt-get install -y \
  ca-certificates curl \
  build-essential bc flex bison \
  libssl-dev libelf-dev dwarves \
  bzip2 xz-utils tar

# (Recommended) headers provide Module.symvers and correct build metadata
apt-get install -y nvidia-l4t-kernel-headers || true

# ---- 3) Download public_sources.tbz2 ----
WORKDIR="/tmp/ch341_build_$$"
mkdir -p "${WORKDIR}"
cd "${WORKDIR}"

PSRC="public_sources.tbz2"

URL_CANDIDATES=(
  # Newer host/path style (capitalization matters)
  "https://developer.download.nvidia.com/embedded/L4T/r${R_MAJOR}_Release_v${REV_TRIM2}/sources/public_sources.tbz2"

  # Older host/path style
  "https://developer.nvidia.com/downloads/embedded/l4t/r${R_MAJOR}_release_v${REV_TRIM2}/sources/public_sources.tbz2"
  "https://developer.nvidia.com/embedded/l4t/r${R_MAJOR}_release_v${REV_TRIM2}/sources/public_sources.tbz2"
)

download_ok="false"
for url in "${URL_CANDIDATES[@]}"; do
  log "Trying download: ${url}"
  rm -f "${PSRC}"
  if curl -fL --retry 3 --retry-delay 2 -o "${PSRC}" "${url}"; then
    # reject tiny "not found" downloads
    sz="$(stat -c%s "${PSRC}" 2>/dev/null || echo 0)"
    if [[ "${sz}" -lt 1000000 ]]; then
      log "Downloaded file is too small (${sz} bytes). Likely an error page. Trying next URL..."
      continue
    fi
    download_ok="true"
    log "Downloaded: ${PSRC} (${sz} bytes)"
    break
  fi
done

[[ "${download_ok}" == "true" ]] || die "Failed to download public_sources.tbz2 for R${R_MAJOR} v${REV_TRIM2}.
Tip: run 'cat /etc/nv_tegra_release' and consider L4T_REV_OVERRIDE=4.4"

log "Sanity-checking archive (bzip2 test)..."
bzip2 -t "${PSRC}" || die "Downloaded file appears corrupted (bzip2 test failed)."

# ---- 4) Extract kernel_src.tbz2 ----
log "Extracting kernel_src.tbz2 from public_sources.tbz2..."
tar -xvf "${PSRC}" "Linux_for_Tegra/source/kernel_src.tbz2" --strip-components=2
[[ -f "kernel_src.tbz2" ]] || die "kernel_src.tbz2 was not extracted (archive layout changed?)."

# ---- 5) Extract kernel sources into /usr/src/kernel ----
log "Installing kernel sources to /usr/src/kernel ..."
rm -rf /usr/src/kernel
mkdir -p /usr/src
tar -xvf "kernel_src.tbz2" -C /usr/src/

KSRC_DIR="$(find /usr/src/kernel -maxdepth 2 -type d -name 'kernel-*-src' | head -n1 || true)"
[[ -n "${KSRC_DIR}" ]] || die "Could not find kernel source dir under /usr/src/kernel (expected kernel-*-src)."
log "Kernel source directory: ${KSRC_DIR}"

# ---- 6) Prepare config ----
log "Copying running kernel config into source tree..."
[[ -f /proc/config.gz ]] || die "/proc/config.gz not found."
zcat /proc/config.gz > "${KSRC_DIR}/.config"

log "Setting LOCALVERSION to match uname -r (${LOCALVERSION})..."
"${KSRC_DIR}/scripts/config" --file "${KSRC_DIR}/.config" --set-str LOCALVERSION "${LOCALVERSION}"

# Try to provide Module.symvers to avoid modpost unresolved symbol spam
if [[ -f "/lib/modules/${KREL}/build/Module.symvers" ]]; then
  log "Copying Module.symvers from installed headers..."
  cp -f "/lib/modules/${KREL}/build/Module.symvers" "${KSRC_DIR}/Module.symvers" || true
fi

log "Preparing kernel tree..."
make -C "${KSRC_DIR}" olddefconfig
make -C "${KSRC_DIR}" prepare
make -C "${KSRC_DIR}" modules_prepare

# ---- 7) Enable CH341 as module ----
log "Enabling CONFIG_USB_SERIAL=y and CONFIG_USB_SERIAL_CH341=m ..."
"${KSRC_DIR}/scripts/config" --file "${KSRC_DIR}/.config" --enable USB_SERIAL
"${KSRC_DIR}/scripts/config" --file "${KSRC_DIR}/.config" --module USB_SERIAL_CH341
make -C "${KSRC_DIR}" olddefconfig

# ---- 8) Locate CH341 source + build correct directory ----
log "Locating ch341 source in kernel tree..."
CH341_C_PATH="$(find "${KSRC_DIR}" -type f -name 'ch341.c' | head -n1 || true)"
[[ -n "${CH341_C_PATH}" ]] || die "Could not find ch341.c in kernel sources. (Unexpected for Linux 5.15.)"

CH341_DIR_REL="$(dirname "${CH341_C_PATH}" | sed "s|^${KSRC_DIR}/||")"
log "Found ch341.c at: ${CH341_DIR_REL}/ch341.c"
log "Building modules in: M=${CH341_DIR_REL}"

make -C "${KSRC_DIR}" -j"$(nproc)" M="${CH341_DIR_REL}" modules

# Find produced module
CHKO="$(find "${KSRC_DIR}/${CH341_DIR_REL}" -maxdepth 1 -type f -name 'ch341.ko' | head -n1 || true)"
[[ -n "${CHKO}" ]] || die "Build finished but ch341.ko not found in ${CH341_DIR_REL}."

# ---- 9) Install ch341.ko to matching path ----
INSTALL_REL="drivers/${CH341_DIR_REL#drivers/}/ch341.ko"
DEST="/lib/modules/${KREL}/kernel/${INSTALL_REL}"

log "Installing ch341.ko to: ${DEST}"
install -D -m 644
::contentReference[oaicite:1]{index=1}
