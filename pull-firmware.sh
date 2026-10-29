#!/bin/bash
# This phone's firmware, from your own phone while it runs Android (adb): the GPU's, into fw-gpu/ at the top of the tree; the
# touch screen's comes from Xiaomi's kernel source (devices/miatoll/tools/mk-fw-touch-miatoll.sh), not from the phone.
# The firmware is the vendor's and stays out of git.
set -e
cd "$(dirname "$(readlink -f "$0")")/../.."
mkdir -p fw-gpu
for f in a618_gmu.bin a630_sqe.fw a615_zap.mdt a615_zap.b00 a615_zap.b01 a615_zap.b02; do
	adb pull /vendor/firmware/$f fw-gpu/
done
devices/miatoll/tools/mk-fw-touch-miatoll.sh
