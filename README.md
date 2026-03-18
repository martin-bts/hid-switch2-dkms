# hid-switch2-dkms

A DKMS module that adds Nintendo Switch 2 controller support to kernels
6.18 and 6.19. [I want this!](#about-this-project)

## Beware

This repo exists because I bought a Nintendo Switch 2 Pro Controller a long
time ago and really want to use it with my computer now. The proper
procedure to add support for this controller to Linux has been underway
and is **NOT** based on this repo at all.

## History

### Switch 2 controller HID driver

* Kernel HID support for the Switch 2 devices is currently developed by Vicki Pfau.
* [HID v1](https://lore.kernel.org/all/20251120022457.469342-1-vi@endrift.com/) was submitted in November 2025.
* [HID v2](https://lore.kernel.org/all/20260109034034.565630-2-vi@endrift.com/) in January 2026.
* [Out-of-tree version](https://github.com/Senko-p/hid-switch2-dkms) (now archived) created by Senko-p.
* [In March 2026](https://lore.kernel.org/all/941e5399-be1c-4e19-b1d4-04ff9ec2a32b@endrift.com/) it seems the kernel driver is still a long way away.

### Bluetooth support in BlueZ

* [BlueZ v1](https://lore.kernel.org/all/20260301152930.221472-1-martinbts@gmx.net/) was submitted in March 2026.
* [BlueZ v2](https://lore.kernel.org/all/20260308124745.19248-1-martinbts@gmx.net/) was submitted in March 2026.
* [BlueZ v3](https://lore.kernel.org/all/20260317202637.158997-1-martinbts@gmx.net/) was submitted in March 2026.


## About this project

This driver is based on the HID v2 driver with Bluetooth support added
on top of it. To make the controller work over Bluetooth, you need a
patched BlueZ bluetoothd that can connect and wire the controller via
BLE. [BlueZ v3 all-in-one patch](v3_all_in_one.patch)

The extension to the HID driver that allows connecting the controller
through BLE is yet to be submitted upstream.

## Install

Download the source, install manually using DKMS, and apply udev rules.

### Arch Linux (as written by Senko-p)

Use an AUR helper to install hid-switch2-dkms:

```
yay -S hid-switch2-dkms
```

## Credits

Based on a mailing list submission from Vicki Pfau <vi@endrift.com> with
minor modifications by @Senko_p.

## Known issues

* IMU does not work.
* Face buttons may be bound incorrectly — rebind them through Steam.
