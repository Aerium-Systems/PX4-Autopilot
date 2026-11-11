#!/usr/bin/env bash
#
# Build the canbootloader + application firmware for aerium_neo-m9n and
# merge them into a single Intel HEX file, for one-shot production flashing
# of blank boards (e.g. via STM32CubeProgrammer) instead of flashing the
# bootloader and app as two separate steps.
#
# Usage: boards/aerium/neo-m9n/scripts/make_merged_hex.sh
# Output: build/aerium_neo-m9n_default/aerium_neo-m9n_merged.hex

set -euo pipefail

BOARD=aerium_neo-m9n
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SRC_DIR="$(cd "$SCRIPT_DIR/../../../.." && pwd)"
cd "$SRC_DIR"

command -v arm-none-eabi-objcopy >/dev/null || { echo "arm-none-eabi-objcopy not found in PATH" >&2; exit 1; }
command -v srec_cat >/dev/null || { echo "srec_cat not found (install package 'srecord')" >&2; exit 1; }

make "${BOARD}_canbootloader"
make "${BOARD}_default"

BL_ELF="build/${BOARD}_canbootloader/${BOARD}_canbootloader.elf"
APP_BIN="build/${BOARD}_default/deploy/1.bin"
OUT_DIR="build/${BOARD}_default"
MERGED="$OUT_DIR/${BOARD}_merged.hex"

[ -f "$BL_ELF" ] || { echo "missing bootloader elf: $BL_ELF" >&2; exit 1; }
[ -f "$APP_BIN" ] || { echo "missing patched app binary: $APP_BIN" >&2; exit 1; }

# Application load address, mirrors OPT_APPLICATION_IMAGE_OFFSET in
# boards/aerium/neo-m9n/src/boot_config.h (OPT_BOOTLOADER_SIZE_IN_K = 64K).
APP_ADDR=0x08010000

TMP_BL_HEX="$(mktemp)"
TMP_APP_HEX="$(mktemp)"
trap 'rm -f "$TMP_BL_HEX" "$TMP_APP_HEX"' EXIT

arm-none-eabi-objcopy -O ihex "$BL_ELF" "$TMP_BL_HEX"
arm-none-eabi-objcopy -I binary -O ihex --change-addresses "$APP_ADDR" "$APP_BIN" "$TMP_APP_HEX"

srec_cat "$TMP_BL_HEX" -intel "$TMP_APP_HEX" -intel -o "$MERGED" -intel

echo "merged hex: $MERGED"
srec_info "$MERGED" -intel
