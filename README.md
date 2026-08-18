# DCC Signal Decoder (Nano)

Self-learning DCC accessory decoder for UK colour-light signals — 2–4 aspect signalling with a flexible set of general-purpose ports for Junction Indicators, Position Lights, and a second independent signal head. Learns its own DCC address from the command station — no PC needed after setup. Configured entirely over USB through a built-in browser-based settings page, no Arduino IDE, no typed commands.

This is the Arduino Nano sibling of the [ESP32-based DCC Signal Decoder](https://github.com/dixieGB/dcc_signal_decoder_esp32) — a separate, independent project with its own firmware and its own Uploader tool. **Firmware and installers between the two are never interchangeable** — this repo's Uploader only ever flashes and updates a Nano, and only ever checks for updates here. That said, a Nano-based decoder and an ESP32-based decoder work together perfectly well side by side on the same layout — chain any mix of the two via Aspect Link (cascade) for automatic block signalling, board type doesn't matter for that.

## What's in this repo

This repo hosts the **firmware** and the **Uploader** — a Windows desktop app used to flash that firmware onto a decoder and configure it, with no Arduino IDE or typed commands required.

### Firmware

Runs on an Arduino Nano (ATmega328P) wired into the signal decoder hardware. Key features:

- **Self-learning address** — hold the board's button to enter Learn Mode, then send the DCC address to teach it from your command station. No PC or source code needed after the initial flash.
- **2–4 aspect signalling** with a flexible set of general-purpose EX ports, each independently configurable as a Junction Indicator, Position Light, or part of a second, fully independent signal head (2/3/4 Aspect) sharing the same board.
- **PWM brightness control**, independently adjustable per LED and per EX port — the primary signal and a second signal head each get their own global dial plus per-LED sliders.
- **Aspect Link** — decoders can be chained so an upstream signal automatically reflects the state of the one ahead, for basic automatic block signalling. Manual overrides are allowed even while a block is occupied (for shunting/manual moves), without ever telling the block behind the line is clear until it actually is.
- **EEPROM-backed configuration** — every setting (address, signal type, routes, brightness, links) survives power loss and firmware updates.
- **Config export/import** — download a full settings backup as a JSON file, and restore it later or onto another board.
- A JSON-based API over USB Serial, the same one the built-in settings page uses.

### DCC Signal Decoder Uploader

A branded Windows `.exe` (Python/Tkinter, packaged with PyInstaller and Inno Setup) that gives end users a simple GUI for:

- **Flashing firmware** — pick a bundled firmware version and a COM port, click Upload. No Arduino IDE, no command line, and the firmware source is never exposed.
- **Warns before re-flashing the same version** already running on a connected board.
- **Board Settings** — opens the board's own configuration page locally over USB: DCC address, signal type, Junction Indicator/Position Light/second signal mapping, Aspect Link, brightness, and more, all read from and written to the board live.
- **Serial Monitor** — a raw view of the board's USB serial traffic, useful for diagnostics.
- **User Guides & Change Logs** per firmware version, opened straight from the app.
- **Auto-update** — checks GitHub Releases on startup and offers to download/install a newer version automatically.

## Getting the software

Download the latest installer from the [Releases](../../releases/latest) page and run it — it installs the app, optional USB-serial drivers (covering most genuine and clone Nano boards), a Start Menu shortcut, and an uninstaller.
