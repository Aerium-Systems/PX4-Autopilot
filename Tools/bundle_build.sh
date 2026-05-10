#!/usr/bin/env bash
#
# bundle_build.sh - Collect the important artifacts and logs from a PX4 build
# into a single folder, together with a build-info file describing it.
#
# Usage:
#   Tools/bundle_build.sh [BUILD_TARGET] [-o OUTPUT_DIR] [-z]
#
#   BUILD_TARGET   Name of the build under build/ (e.g. px4_fmu-v6x_default).
#                  If omitted, auto-detects the most recently built target.
#   -o OUTPUT_DIR  Where to place the bundle folder (default: build/bundles).
#   -z             Also create a .tar.gz archive of the bundle.
#
# Examples:
#   Tools/bundle_build.sh
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
            sed -n '2,21p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'
            exit 0 ;;
        -*) echo "Unknown option: $1" >&2; exit 1 ;;
        *) TARGET="$1"; shift ;;
    esac
done

# --- resolve target -----------------------------------------------------------
if [[ -z "${TARGET}" ]]; then
    # pick the most recently modified build dir that has a firmware product
    TARGET="$(find "${BUILD_ROOT}" -maxdepth 2 -name "*.px4" -printf '%T@ %h\n' 2>/dev/null \
        | sort -rn | head -1 | awk '{print $2}' | xargs -r basename)"
    [[ -n "${TARGET}" ]] && echo ">> Auto-detected target: ${TARGET}"
fi

BUILD_DIR="${BUILD_ROOT}/${TARGET}"
if [[ -z "${TARGET}" || ! -d "${BUILD_DIR}" ]]; then
    echo "ERROR: build directory not found: ${BUILD_DIR:-<none>}" >&2
    echo "Available builds:" >&2
    find "${BUILD_ROOT}" -maxdepth 1 -mindepth 1 -type d -printf '  %f\n' 2>/dev/null >&2
    exit 1
fi

# --- prepare bundle folder ----------------------------------------------------
STAMP="$(date +%Y%m%d_%H%M%S)"
GIT_SHORT="$(git -C "${REPO_ROOT}" rev-parse --short HEAD 2>/dev/null || echo nogit)"
BUNDLE_NAME="${TARGET}_${GIT_SHORT}_${STAMP}"
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
        echo "Describe         : $(git -C "${REPO_ROOT}" describe --always --tags --dirty 2>/dev/null || echo n/a)"
        echo "Commit date      : $(git -C "${REPO_ROOT}" log -1 --format=%ci)"
        echo "Commit subject   : $(git -C "${REPO_ROOT}" log -1 --format=%s)"
        if [[ -n "$(git -C "${REPO_ROOT}" status --porcelain 2>/dev/null)" ]]; then
            echo "Working tree     : DIRTY (uncommitted changes present)"
        else
            echo "Working tree     : clean"
        fi
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
    ARCHIVE="${OUTPUT_DIR}/${BUNDLE_NAME}.tar.gz"
    tar -czf "${ARCHIVE}" -C "${OUTPUT_DIR}" "${BUNDLE_NAME}"
    echo ">> Archive: ${ARCHIVE}"
fi

echo ">> Done. Bundle at: ${BUNDLE_DIR}"
