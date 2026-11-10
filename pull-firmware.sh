#!/bin/bash
# This phone's firmware, from your own phone while it runs Android (adb): the GPU's, into fw-gpu/ at the top of the tree, and
# the Tianma panel's touch firmware (its Novatek NT36675 keeps none; every miatoll's vendor image carries it, whichever panel
# the phone has) into fw-touch/nt36675.bin. The Huaxing panel's comes from Xiaomi's kernel source
# (devices/miatoll/tools/mk-fw-touch-miatoll.sh), not from the phone.
# The firmware is the vendor's and stays out of git.
set -e
cd "$(dirname "$(readlink -f "$0")")/../.."
mkdir -p fw-gpu
for f in a618_gmu.bin a630_sqe.fw a615_zap.mdt a615_zap.b00 a615_zap.b01 a615_zap.b02; do
	adb pull /vendor/firmware/$f fw-gpu/
done
mkdir -p fw-touch
if adb shell ls /vendor/firmware/novatek_ts_tianma_fw.bin >/dev/null 2>&1; then
	adb pull /vendor/firmware/novatek_ts_tianma_fw.bin fw-touch/nt36675.bin
else
	echo "pull-firmware: no /vendor/firmware/novatek_ts_tianma_fw.bin on this phone: the Tianma panel's touch screen (nt36675=1) will not start" >&2
fi
devices/miatoll/tools/mk-fw-touch-miatoll.sh
