# hakios-sm7125: the Redmi Note 9S (miatoll, curtana) and its Snapdragon 720G (SM7125)

One HakiOS device repository: the device (`device.conf`, `configs/`, `boot.conf`, its own drivers in `drivers/`, its UEFI port in
`uefi/`) and its SoC family's layer (`soc/`: SM7125). It covers the phones that share the SoC: Redmi Note 9S / 9 Pro / 9 Pro Max,
Redmi Note 10 Lite, POCO M2 Pro. It was made and tested on a 64 GB Redmi Note 9S with the Huaxing panel; the partition numbers
below are that phone's. `tools/hakios-device.py sync` in the common repository clones it into `devices/miatoll/`.

iOS 16.5 runs on it from the phone's own flash: the home screen, every stock Apple app, touch, brightness, the flashlight, the
battery level, Sileo, SSH and internet over the USB cable. What does not (sound, vibration, Wi-Fi, camera, cellular) is in
`device.conf`, area by area.

## What is here

| What | Where |
|---|---|
| the kernel's configuration | `configs/miatoll_defconfig` = common + `soc/configs/sm7125.config` + `configs/miatoll.config` |
| the drivers of this phone alone | `drivers/input/hakios.miatoll_touch.c` (FT8756), `drivers/power/hakios.miatoll_battery.c` (PM6150 fuel gauge), `drivers/power/hakios.miatoll_backlight.c` (PM6150L WLED) |
| what `xl` and the boot image need | `boot.conf`: the UEFI target, its addresses, the arguments only the drivers above read |
| the UEFI | `uefi/`: Mu-Silicium **3.6** with the boot shim, the loader hook and the memory map (`apply.sh`); later releases do not boot these phones, and clang 18 builds it |
| the touch screen's firmware | `tools/mk-fw-touch-miatoll.sh` (Xiaomi's, from their kernel source) |
| the phone's own firmware | `pull-firmware.sh`: the GPU's, off your phone over `adb`. Not in the repository |

The UEFI image itself is **not** in this repository: it holds Qualcomm's and Xiaomi's drivers. Build it on your computer
(`uefi/apply.sh`, then Mu-Silicium's `build_uefi.py -d miatoll -m 0 -r RELEASE` with clang 18; `-m` is the model, 0 the Note 9S).
`prebuilt/SHA256SUMS` has the checksum of the build that boots, because the same source built on another machine once did not.

## Setting up the phone (once)

iOS's root volume lives in a partition of its own, `darwin`, carved from the end of `userdata`. **This wipes Android's data.** The
loader and the kernel write nowhere else. The numbers are in the loader (`darwin_miatoll` in `xnuloader/loader.c`), the UFS driver and
`tools/darwin-write.py`: put your phone's own (from `parted ... print`) in all three before you do it on another phone.

## Booting

The phone in fastboot (Power + Volume Down), then `DEVICE=miatoll ./xl ios` from the top of the tree. About a minute to the home
screen. Hold Power + Volume Down to leave; settings are saved on the way.
