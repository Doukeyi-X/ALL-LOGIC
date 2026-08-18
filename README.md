# ALL LOGIC

Unofficial multi-vendor logic analyzer host.  
This is a **secondary modification** of [DSView](https://github.com/DreamSourceLab/DSView)
(sigrok / PulseView). It is **not** an official DreamSourceLab product
and is not affiliated with any hardware vendor.

## Download

Windows installer and portable zip: see
[Releases](https://github.com/Doukeyi-X/ALL-LOGIC/releases).

This git repository is **source only**. Firmware trees, installers, and
local calibration tools are not published here.

## What this tree contains

- Host UI (`DSView/`)
- Device layer (`libsigrok4DSL/`) including community drivers
- Protocol decoders (`libsigrokdecode4DSL/`)
- Language files (`lang/`)
- Windows packager (`tools/package_windows.ps1`)

## License

GNU GPLv3 or later. See `COPYING` and `NOTICE.txt`.

## Upstream

- DSView: https://github.com/DreamSourceLab/DSView
- sigrok / PulseView: https://sigrok.org
