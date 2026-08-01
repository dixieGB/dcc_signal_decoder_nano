# DCC Signal Decoder

Self-learning DCC accessory decoder for UK colour-light signals (2–4 aspect + route indicator). Learns its own address from the command station — no PC needed after setup. Arduino/ATmega328P firmware with EEPROM config, PWM brightness control, and an inter-board "Aspect Link" for automatic block signalling.

## What's in this repo

This repo hosts the **firmware** and the **Uploader** — a Windows desktop app used to flash that firmware onto a decoder and configure it, with no Arduino IDE or typed commands required.

### Firmware

Runs on an Arduino Nano (ATmega328P) wired into the signal decoder hardware. Key features:

- **Self-learning address** — hold the board's button to enter Learn Mode, then send the DCC address to teach it from your command station. No PC or source code needed after the initial flash.
- **2–4 aspect signalling** with optional Route Indicator(s) and Position Light.
- **PWM brightness control**, independently adjustable per LED colour and per route.
- **Aspect Link** — decoders can be chained so an upstream signal automatically reflects the state of the one ahead, for basic automatic block signalling.
- **EEPROM-backed configuration** — every setting (address, signal type, brightness, links) survives power loss.
- A serial bench-test command set (`status`, `dcc`, `occ`, `cascade`, `learn`, `reset`, `help`) for configuring and testing a board directly from a PC during development.

### DCC Signal Decoder Uploader

A branded Windows `.exe` (Python/Tkinter, packaged with PyInstaller and Inno Setup) that gives end users a simple GUI for:

- **Flashing firmware** — pick a bundled firmware version and a COM port, click Upload. No Arduino IDE, no command line, and the firmware source is never exposed.
- **Board Settings** — a graphical front end for every bench-test setting above: DCC address, signal type, routes, position light, aspect memory/link, end-of-line, and per-colour brightness — all read from and written to the board live over serial.
- **Serial Monitor** — a live view of everything the board prints, useful for troubleshooting.
- **User Guides & Change Logs** per firmware version, opened straight from the app.
- **Auto-update** — checks GitHub Releases on startup and offers to download/install a newer version automatically.

## Getting the software

Download the latest installer from the [Releases](../../releases/latest) page and run it — it installs the app, an optional CH340 USB-serial driver (needed by most Nano clones), a Start Menu shortcut, and an uninstaller.
