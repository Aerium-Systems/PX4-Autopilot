#!/usr/bin/env bash
#
# bundle_build.sh - Clean-build a PX4 target, then collect the important
# artifacts and logs into a single folder with a build-info file.
#
# Runs `make clean` followed by `make <BUILD_TARGET>` before bundling, so the
# bundle always reflects a fresh build rather than stale incremental artifacts.
#
# Usage:
#   Tools/bundle_build.sh BUILD_TARGET [-o OUTPUT_DIR] [-z]
#
#   BUILD_TARGET   PX4 board target to build (required),
#                  e.g. px4_fmu-v6x_default or aerium_radian_h7_rev_b_default.
#   -o OUTPUT_DIR  Where to place the bundle folder (default: build/bundles).
#   -z             Also create a .zip archive of the bundle.
#
# Examples:
#   Tools/bundle_build.sh px4_fmu-v6x_default -z
#   Tools/bundle_build.sh aerium_radian_h7_rev_b_default -o /tmp/out -z

set -euo pipefail

# --- locate repo root (this script lives in Tools/) ---------------------------
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
BUILD_ROOT="${REPO_ROOT}/build"

# --- parse args ---------------------------------------------------------------
TARGET=""
OUTPUT_DIR="${BUILD_ROOT}/bundles"
MAKE_ARCHIVE=0

while [[ $# -gt 0 ]]; do
    case "$1" in
        -o|--output) OUTPUT_DIR="$2"; shift 2 ;;
        -z|--archive) MAKE_ARCHIVE=1; shift ;;
        -h|--help)
            sed -n '2,19p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'
            exit 0 ;;
        -*) echo "Unknown option: $1" >&2; exit 1 ;;
        *) TARGET="$1"; shift ;;
    esac
done

# --- require explicit target --------------------------------------------------
# A clean build is performed below, so the target cannot be auto-detected from
# existing build dirs (there may be none after 'make clean').
if [[ -z "${TARGET}" ]]; then
    echo "ERROR: a build target is required (this script performs a clean build)." >&2
    echo "Usage: Tools/bundle_build.sh BUILD_TARGET [-o OUTPUT_DIR] [-z]" >&2
    exit 1
fi

# --- clean build --------------------------------------------------------------
# The committed bootloader binary (extras/<vendor>_<model>_bootloader.bin) is
# also a bootloader-target build output, so 'make clean' deletes it. Restore it
# from git after the clean so the working tree is clean for the git describe used
# in the bundle name (no spurious -dirty) and the bin is available to bundle for
# JTAG/SWD flashing. (The app firmware embeds it in ROMFS only when
# CONFIG_SYSTEMCMDS_BL_UPDATE is set; otherwise PX4 strips it.)
BL_BIN="${TARGET%_*}_bootloader.bin"
BL_PATH="$(git -C "${REPO_ROOT}" ls-files "*/extras/${BL_BIN}" 2>/dev/null | head -1)"

echo ">> Clean build: make clean && make ${TARGET}"
make -C "${REPO_ROOT}" clean
[[ -n "${BL_PATH}" ]] && git -C "${REPO_ROOT}" checkout -q -- "${BL_PATH}" 2>/dev/null || true

# Capture git state in the clean window: 'make clean' has just cleaned submodule
# build artifacts and the bootloader bin is restored, so the tree is pristine
# here. Submodule state IS significant (e.g. a patched NuttX), so it is counted
# in the dirty check; capturing pre-build keeps the build's own submodule
# artifacts out of the bundle name / BUILD_INFO.
GIT_DESCRIBE="$(git -C "${REPO_ROOT}" describe --tags --always --dirty 2>/dev/null || echo nogit)"
if [[ -n "$(git -C "${REPO_ROOT}" status --porcelain 2>/dev/null)" ]]; then
    GIT_TREE_STATE="DIRTY (uncommitted changes present)"
else
    GIT_TREE_STATE="clean"
fi

make -C "${REPO_ROOT}" "${TARGET}"

BUILD_DIR="${BUILD_ROOT}/${TARGET}"
if [[ ! -d "${BUILD_DIR}" ]]; then
    echo "ERROR: build directory not found after build: ${BUILD_DIR}" >&2
    exit 1
fi

# --- prepare bundle folder ----------------------------------------------------
STAMP="$(date +%Y%m%d_%H%M%S)"
# Name the bundle after `git describe` (captured pre-build in the clean window
# above) so the identity matches the release tag without the build's submodule
# churn marking it -dirty.
BUNDLE_NAME="${TARGET}_${GIT_DESCRIBE}_${STAMP}"
BUNDLE_DIR="${OUTPUT_DIR}/${BUNDLE_NAME}"
mkdir -p "${BUNDLE_DIR}/logs" "${BUNDLE_DIR}/config"

echo ">> Bundling '${TARGET}' -> ${BUNDLE_DIR}"

# helper: copy a file if it exists
copy_if() {  # copy_if <src> <dest-dir>
    [[ -f "$1" ]] && cp -p "$1" "$2/" && echo "   + $(basename "$1")"
    return 0
}

# --- firmware products --------------------------------------------------------
for ext in px4 elf bin hex map; do
    copy_if "${BUILD_DIR}/${TARGET}.${ext}" "${BUNDLE_DIR}"
done

# --- bootloader (for JTAG / SWD / bare-metal flashing) ------------------------
# BL_PATH was resolved and the file restored from git before the build (above),
# so it is on disk here; bundle it for a bare-metal JTAG/SWD flash.
BL_BUNDLED=""
if [[ -n "${BL_PATH}" && -f "${REPO_ROOT}/${BL_PATH}" ]]; then
    cp -p "${REPO_ROOT}/${BL_PATH}" "${BUNDLE_DIR}/"
    BL_BUNDLED="${BUNDLE_DIR}/${BL_BIN}"
    echo "   + ${BL_BIN} (from git: ${BL_PATH})"
else
    echo "   (no committed bootloader for this board: ${BL_BIN})"
fi

# --- metadata / definition artifacts ------------------------------------------
for f in parameters.xml parameters.json airframes.xml actuators.json \
         component_general.json; do
    copy_if "${BUILD_DIR}/${f}" "${BUNDLE_DIR}/config"
done

# --- build configuration ------------------------------------------------------
for f in CMakeCache.txt px4_boardconfig.h boardconfig .config; do
    copy_if "${BUILD_DIR}/${f}" "${BUNDLE_DIR}/config"
done
# NuttX defconfig (kept under nuttx_config/ in the source tree, but the resolved
# .config is the most useful one if present)
copy_if "${BUILD_DIR}/NuttX/nuttx/.config" "${BUNDLE_DIR}/config"

# --- logs ---------------------------------------------------------------------
for log in "${BUILD_DIR}"/*.log; do
    copy_if "${log}" "${BUNDLE_DIR}/logs"
done
copy_if "${BUILD_DIR}/.ninja_log" "${BUNDLE_DIR}/logs"

# --- build-info file ----------------------------------------------------------
INFO="${BUNDLE_DIR}/BUILD_INFO.txt"
ELF="${BUILD_DIR}/${TARGET}.elf"

{
    echo "=============================================================="
    echo " PX4 BUILD INFO"
    echo "=============================================================="
    echo "Bundle name      : ${BUNDLE_NAME}"
    echo "Build target     : ${TARGET}"
    echo "Bundled at       : $(date '+%Y-%m-%d %H:%M:%S %z')"
    echo "Bundled on host  : $(hostname) ($(uname -srm))"
    echo "Bundled by       : $(whoami)"
    echo

    echo "----- Git --------------------------------------------------"
    if git -C "${REPO_ROOT}" rev-parse --git-dir >/dev/null 2>&1; then
        echo "Commit (full)    : $(git -C "${REPO_ROOT}" rev-parse HEAD)"
        echo "Commit (short)   : $(git -C "${REPO_ROOT}" rev-parse --short HEAD)"
        echo "Branch           : $(git -C "${REPO_ROOT}" rev-parse --abbrev-ref HEAD)"
        echo "Describe         : ${GIT_DESCRIBE}"
        echo "Commit date      : $(git -C "${REPO_ROOT}" log -1 --format=%ci)"
        echo "Commit subject   : $(git -C "${REPO_ROOT}" log -1 --format=%s)"
        echo "Working tree     : ${GIT_TREE_STATE}"
        echo
        echo "Submodule status :"
        git -C "${REPO_ROOT}" submodule status --recursive 2>/dev/null | sed 's/^/  /' || echo "  n/a"
    else
        echo "Not a git repository."
    fi
    echo

    echo "----- Firmware artifacts -----------------------------------"
    for ext in px4 elf bin hex map; do
        f="${BUILD_DIR}/${TARGET}.${ext}"
        if [[ -f "${f}" ]]; then
            size=$(stat -c '%s' "${f}")
            mtime=$(date -d "@$(stat -c '%Y' "${f}")" '+%Y-%m-%d %H:%M:%S')
            md5=$(md5sum "${f}" | awk '{print $1}')
            printf "%-6s : %10d bytes  built %s  md5 %s\n" \
                "${ext}" "${size}" "${mtime}" "${md5}"
        fi
    done
    echo

    echo "----- Bootloader -------------------------------------------"
    if [[ -n "${BL_BUNDLED}" && -f "${BL_BUNDLED}" ]]; then
        bsize=$(stat -c '%s' "${BL_BUNDLED}")
        bmd5=$(md5sum "${BL_BUNDLED}" | awk '{print $1}')
        echo "File             : ${BL_BIN}"
        echo "Source (git)     : ${BL_PATH}"
        echo "Size             : ${bsize} bytes"
        echo "MD5              : ${bmd5}"
        echo "Flash (SWD/JTAG) : write to flash origin 0x08000000 (bootloader region)"
    else
        echo "(no bootloader bundled for this board)"
    fi
    echo

    if [[ -f "${ELF}" ]]; then
        echo "----- ELF size (arm-none-eabi-size) ------------------------"
        if command -v arm-none-eabi-size >/dev/null 2>&1; then
            arm-none-eabi-size "${ELF}" 2>/dev/null || true
        elif command -v size >/dev/null 2>&1; then
            size "${ELF}" 2>/dev/null || true
        else
            echo "(size tool not available)"
        fi
        echo
    fi

    echo "----- Toolchain --------------------------------------------"
    if command -v arm-none-eabi-gcc >/dev/null 2>&1; then
        echo "arm-none-eabi-gcc: $(arm-none-eabi-gcc --version | head -1)"
    fi
    command -v cmake >/dev/null 2>&1 && echo "cmake            : $(cmake --version | head -1)"
    command -v ninja >/dev/null 2>&1 && echo "ninja            : $(ninja --version 2>/dev/null)"
    command -v python3 >/dev/null 2>&1 && echo "python3          : $(python3 --version 2>&1)"
} > "${INFO}"

echo "   + BUILD_INFO.txt"

# --- optional archive ---------------------------------------------------------
if [[ "${MAKE_ARCHIVE}" -eq 1 ]]; then
    ARCHIVE="${OUTPUT_DIR}/${BUNDLE_NAME}.zip"
    ( cd "${OUTPUT_DIR}" && zip -qr "${BUNDLE_NAME}.zip" "${BUNDLE_NAME}" )
    echo ">> Archive: ${ARCHIVE}"
fi

echo ">> Done. Bundle at: ${BUNDLE_DIR}"
