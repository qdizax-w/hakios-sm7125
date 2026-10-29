#!/bin/bash
# Make Mu-Silicium-3.6 at the top of the HakiOS tree: Mu-Silicium at release 3.6 with the miatoll port in it.
# Releases after 3.6 do not boot on these phones (Mi logo, a reset, again), so this device is built from 3.6, not from the
# commit hakios-uefi's BASE names for the POCO. Build it with clang 18 -- clang 23 makes images that do not boot either:
#   cd Mu-Silicium-3.6 && CLANG_BIN=<llvm-18>/bin/ python3 build_uefi.py -d miatoll -m N -r RELEASE
#
# What goes in: mu-silicium-3.6.patch (the boot shim copying the payload, the boot manager starting the XNU loader, the
# "XNU Payload" region of miatollPkg's memory map) and loader-hook/XnuLoader.[ch], hakios-uefi's loader hook with the payload's
# address taken from that memory-map region (0xB0000000 here, not the POCO's 0xF0000000); in 3.6 the boot manager library is
# PlatformBootManagerLib. (Not in files/: hakios-uefi/apply.sh copies every devices/*/uefi/files/ into its own, newer base.)
set -e
cd "$(dirname "$(readlink -f "$0")")"
MU=../../../Mu-Silicium-3.6
[ -e $MU ] && { echo "devices/miatoll/uefi/apply.sh: $MU already exists"; exit 1; }
git clone https://github.com/Project-Silicium/Mu-Silicium.git $MU
git -C $MU checkout -q "$(cat BASE)"
git -C $MU submodule update --init --recursive
git -C $MU apply --whitespace=nowarn "$PWD/mu-silicium-3.6.patch"
cp loader-hook/XnuLoader.[ch] $MU/Silicon/Silicium/SiliciumPkg/Library/PlatformBootManagerLib/
echo "devices/miatoll/uefi/apply.sh: done; next: build it with clang 18 (above)"
