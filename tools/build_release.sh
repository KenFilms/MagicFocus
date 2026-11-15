#!/bin/sh
# MagicFocus firmware: lidar autofocus for an external lens focus motor
#
# Builds the binary release that the web installer at kenfilms.github.io/MagicFocus/#install offers:
#  1. the Gateway H2's bridge (h2_firmware/TiltaBridgeH2), merged into TiltaBridgeH2.bin, which the
#     CoreS3 build carries and installs on its stacked H2 (src/BridgeUpdate.h);
#  2. MagicFocus for the CoreS3 and the Core2;
#  3. docs/firmware/: each of the three as one image to write at offset 0, and the manifest that ESP Web
#     Tools reads. It picks the image for the chip it finds: ESP32-S3 = CoreS3, ESP32 = Core2,
#     ESP32-H2 = Gateway H2.
# The version comes from MAGICFOCUS_VERSION in src/Config.h. Commit docs/firmware/ and
# h2_firmware/TiltaBridgeH2/TiltaBridgeH2.bin afterwards; GitHub Pages then serves the release.
#
# Usage: tools/build_release.sh
set -eu
cd "$(dirname "$0")/.."

PIO=${PIO:-$HOME/.platformio/penv/bin/pio}
PYTHON=${PYTHON:-$HOME/.platformio/penv/bin/python}
ESPTOOL=${ESPTOOL:-$HOME/.platformio/packages/tool-esptoolpy/esptool.py}
esptool() { "$PYTHON" "$ESPTOOL" "$@"; }

VERSION=$(sed -n 's/^#define MAGICFOCUS_VERSION "\(.*\)"$/\1/p' src/Config.h)
[ -n "$VERSION" ] || { echo "MAGICFOCUS_VERSION not found in src/Config.h" >&2; exit 1; }
echo "MagicFocus $VERSION"

# 1. The bridge. Written as the README's esptool command does: bootloader, partition table and app,
# DIO at 48 MHz on 2 MB. PlatformIO's own firmware.factory.bin also writes boot_app0 at 0xe000,
# which this partition table uses for phy_init.
"$PIO" run -d h2_firmware/TiltaBridgeH2
H2=h2_firmware/TiltaBridgeH2/.pio/build/gateway-h2
esptool --chip esp32h2 merge-bin --flash-mode dio --flash-freq 48m --flash-size 2MB \
  -o h2_firmware/TiltaBridgeH2/TiltaBridgeH2.bin \
  0x0 "$H2/bootloader.bin" 0x8000 "$H2/partitions.bin" 0x10000 "$H2/firmware.bin"

# 2. MagicFocus; the CoreS3 build embeds the bridge from step 1.
"$PIO" run -e m5stack-cores3 -e m5stack-core2

# 3. The installer's files. PlatformIO's firmware.factory.bin is the whole image from offset 0:
# bootloader (at 0x0 on the S3, 0x1000 on the ESP32), partition table, boot_app0 and the app.
mkdir -p docs/firmware
cp .pio/build/m5stack-cores3/firmware.factory.bin docs/firmware/magicfocus-cores3.bin
cp .pio/build/m5stack-core2/firmware.factory.bin docs/firmware/magicfocus-core2.bin
cp h2_firmware/TiltaBridgeH2/TiltaBridgeH2.bin docs/firmware/tiltabridge-h2.bin

cat > docs/firmware/manifest.json <<EOF
{
  "name": "MagicFocus",
  "version": "$VERSION",
  "new_install_prompt_erase": true,
  "builds": [
    { "chipFamily": "ESP32-S3", "parts": [{ "path": "magicfocus-cores3.bin", "offset": 0 }] },
    { "chipFamily": "ESP32", "parts": [{ "path": "magicfocus-core2.bin", "offset": 0 }] },
    { "chipFamily": "ESP32-H2", "parts": [{ "path": "tiltabridge-h2.bin", "offset": 0 }] }
  ]
}
EOF

for f in docs/firmware/*.bin; do
  echo "$(shasum -a 256 "$f" | cut -c1-16)  $(wc -c < "$f" | tr -d ' ') bytes  $f"
done
echo "Release $VERSION is in docs/firmware/."
