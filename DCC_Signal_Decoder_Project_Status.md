# DCC Signal Decoder — Project Status & Handoff

Single-signal DCC accessory decoder (2/3/4 aspect + optional route indicator) that learns its own DCC addresses directly from the command station — no PC required after installation. Developed and bench-tested on an Arduino Nano. **A PCB is now on order/in production** — it hosts the Nano directly (as a module), rather than a bare ATmega328P chip, so the original bare-chip migration plan (external crystal, fuse bits, ISP-vs-bootloader programming) no longer applies. Current firmware baseline: **v1.4** (`DCC_Signal_Decoder_v1_4.ino`).

## Hardware

- DCC input via the previously-confirmed 6N137 opto-isolator sniffer circuit (1k5 + 1N4148 protection, 6N137, NAND gate with pull-ups) → Nano D2.
- Pin map (current, v1.4):
  - D2 – DCC input
  - D3 – Red
  - D4 – Status LED
  - D5 – Double Yellow
  - D6 – Green
  - D7 – DIP1 (Signal Type: 2 Aspect / 3-4 Aspect)
  - D8 – Learn button
  - D9 – Yellow
  - D10 – Route Indicator
  - D11 – DIP2 (Route Indicator fitted / not fitted)
  - D12 – reserved, unused
  - D13 – reserved, unused
  - A0 – Track occupancy detector input, this signal's own block. Detector bridges this pin to GND when occupied (relay/opto-style contact closure) — reads LOW when occupied against the Nano's internal pull-up, matching `OCCUPANCY_ACTIVE_STATE = LOW`.
  - A1 – Link IN (SoftwareSerial RX), receives the aspect broadcast from the block ahead
  - A2 – Link OUT (SoftwareSerial TX), transmits this board's own aspect to the block behind
- DIP switches and Learn button: one leg to GND, other leg to the pin, relying on the Nano's internal pull-ups (`INPUT_PULLUP`). ON / Pressed = LOW. (If hardware is wired the opposite way, flip `DIP_ACTIVE_STATE` / `BUTTON_PRESSED_STATE` in the sketch.)
- Full pin rationale (PWM constraints, why D12/D13 are unused, why A0–A2 were chosen) lives in the Firmware Configuration & Maintenance Guide — the `.ino` header no longer duplicates it, just points there.

## PCB status

- A PCB is on order. **It does not include the occupancy detector circuit** — that stays as separate, already-owned/existing detector boards, external to the new PCB.
- Incorporating occupancy detection needs no PCB provision beyond the existing A0/GND terminal pair: the detector board is simply wired in with 2 cables (GND and A0), the same relay/opto-style contact-closure behaviour the firmware already expects (see A0 in the pin map above) — this is exactly how it was bench-tested, just with a jumper wire standing in for the detector board's contact.
- Next step once the PCB arrives: attach a real detector board via those 2 cables and test live on track with a real train, rather than the jumper-wire/`occ` bench simulation used so far.

## Firmware design decisions

- **3-aspect and 4-aspect are treated identically** by the firmware (2 DCC addresses each). DIP1 only toggles "2 Aspect" vs "3/4 Aspect" — the real difference between a 3- and 4-aspect signal is just which LEDs are physically fitted (Double Yellow populated or not).
- **Address allocation on Learn** (base address = N, the first address operated from the command station), rejected with a Serial message if the highest address needed would exceed the NMRA 2044 accessory decoder limit:
  - 2 Aspect, no route: Aspect = N
  - 2 Aspect, + route: Aspect = N, Route = N+1
  - 3/4 Aspect, no route: Aspect1 = N, Aspect2 = N+1
  - 3/4 Aspect, + route: Aspect1 = N, Aspect2 = N+1, Route = N+2
- **DIP switches are sampled only when Learn Mode is entered**, not continuously.
- **EEPROM config** stored at address 600 (avoids collision with NmraDcc's internal CV storage), protected by a validity marker + checksum, validated only at boot (`loadConfig()`), not on every `isConfigured()` call.
- **Duplicate-packet filter** uses a 250ms window (`DUPLICATE_PACKET_WINDOW_MS`) so genuine rapid-repeat packets are filtered without dropping the first real repeat command after a Learn.
- **PWM-based brightness control** across all 5 LEDs, per-LED fixed baselines (`BRIGHTNESS_BASELINE_*`, compiled in, not runtime-adjustable) plus one user-adjustable global percentage (1–100%, step 1) via Learn Mode + address 999 (`BRIGHTNESS_CONFIG_ADDRESS`).
- **Status LED reference:** OFF = normal/learned/DCC present, Double Blip = learned but no DCC seen ~5s, Slow flash = Learn Mode waiting, Fast flash = saving to EEPROM, Triple flash = Learn success, Rapid flash = Factory Reset confirmed, Solid ON = not yet learned.
- **Factory Reset** via 10-second Learn button hold, wipes learned config back to "not yet learned."
- Optional `USE_DEFAULT_DCC_ADDRESS` / `FORCE_SIGNAL_CONFIG` build-time options for bench testing and buttonless/switchless Nano builds.
- Board-address → flat-address translation kept identical to the original proven multi-signal sketch.
- Bench-test-only Serial commands (115200 baud), no effect on final standalone behaviour: `status`, `build`, `learn`, `reset`, `dcc <addr> <closed|thrown>`, `occ <on|off|auto>`, `cascade <aspect>`, `help`.
- **Boot is quiet** — only the version banner, build options, and `printConfigSummary()` print at startup. `setAspect()`'s per-change Serial line is suppressed during all startup state-setting (including the Aspect Link boot resolution), since the resulting aspect is already shown in the "Current Aspect" line.
- **Aspect Memory's per-change EEPROM save is skipped entirely whenever Aspect Link is enabled** — its saved value is already ignored at boot in that case (see below), so saving it on every cascade-derived change would just be needless EEPROM wear (rated ~100,000 write cycles per cell) for a value that could never be read back. With Aspect Memory left disabled (the default, and the planned deployment state for this project), the only EEPROM writes that ever happen are genuine one-off configuration actions — teaching an address, toggling 888/889/998, adjusting brightness — never anything triggered by ongoing operation (DCC commands, occupancy, or cascade activity).
- **Aspect Memory is being kept in the firmware, not removed**, despite being functionally redundant on any board running Aspect Link (its value is ignored at boot whenever Aspect Link is on). Decided to keep it as an option for any future signal built without an occupancy detector fitted, even though every board built so far — and likely all future ones — will run with Aspect Link enabled and Aspect Memory left disabled.
- **Factory Reset now also clears the Aspect Link runtime state** (`dccAspectOverrideActive`, the clear-debounce countdown, cascade RX/TX history, occupancy edge tracking) — a Factory Reset happens live, not via a power cycle, so these RAM variables don't reset themselves the way they would on a genuine reboot. Without this fix, resetting a board while Aspect Link was active and then quickly re-enabling it again (without power-cycling) could briefly act on stale leftover state until the next genuine occupancy edge sorted it out.
- **Hardware watchdog timer** (`WATCHDOG_TIMEOUT_SETTING`, `WDTO_4S` / 4 seconds) added — if `loop()` ever stops running (electrical noise, a brown-out, an unforeseen software fault), the chip force-reboots itself rather than sitting locked up indefinitely under the layout. `wdt_disable()` is called as the very first line of `setup()` (clears any watchdog state left armed by the bootloader from a prior watchdog reset, avoiding a known reset-loop risk on some AVR bootloaders), `wdt_enable()` only at the very end of `setup()` once everything is genuinely initialised, and `wdt_reset()` on every pass through `loop()`. 4 seconds was chosen deliberately: the longest legitimate blocking sequence in this firmware is a completed Learn (`indicateEepromSaving()` + `indicateLearnSuccess()` back-to-back, ~1.7s of intentional confirmation blinking) — a shorter timeout would have falsely rebooted the board mid-confirmation-flash on every successful Learn. A watchdog reboot is functionally identical to a power cycle from the firmware's perspective; nothing extra needed to handle it given the existing boot-time state initialisation and EEPROM checksum protection.

## Naming: "Aspect Link" (renamed from "Occupancy Override")

The master enable for occupancy detection + cascade link exchange — DCC address **888**, EEPROM field, related constants — is called **Aspect Link** throughout the firmware, both guides, and this document. This does **not** apply to the physical occupancy sensing itself (`PIN_OCCUPANCY_INPUT`, `isOccupied()`, `OCCUPANCY_ACTIVE_STATE`, the `occ` bench command, "Block Occupied" status line) — those keep "occupancy" wording, since they refer to the physical track sensor, not the address-888 toggle.

Renamed identifiers: `DEFAULT_OCCUPANCY_OVERRIDE_ENABLED` → `DEFAULT_ASPECT_LINK_ENABLED`, `OCCUPANCY_OVERRIDE_TOGGLE_ADDRESS` → `ASPECT_LINK_TOGGLE_ADDRESS`, `config.occupancyOverrideEnabled` → `config.aspectLinkEnabled`, `toggleOccupancyOverride()` → `toggleAspectLink()`.

In the User Guide specifically, **every** "Occupancy Detection" reference (Sections 3, 7, 9, 10, 12) was updated to "Aspect Link" for full consistency — there is no longer a naming split between the two guides.

## Occupancy & Cascade ("Aspect Link") Addition (implemented in v1.4)

A track occupancy detector (existing hardware — a relay/opto-style contact that bridges the A0 pin to GND when occupied) is integrated per signal board, alongside a wired Link between adjacent signal boards, so a chain of boards can automatically derive Yellow/Double Yellow/Green aspects from the state of the block ahead — full 4-aspect distributed block signalling, without needing external layout software to compute every signal aspect itself.

### EEPROM config fields

- `bool aspectLinkEnabled` — master on/off for all occupancy/cascade behaviour. Toggled by sending DCC address **888** during Learn Mode (same interception mechanism as the existing brightness address 999 — no aspect address is consumed, no full learn cycle needed). Default `false` on new builds / factory reset, via `DEFAULT_ASPECT_LINK_ENABLED`. When `false`, decoder behaves exactly as a standalone signal — no occupancy pin read, no cascade activity.
- `bool endOfLineEnabled` — marks this board as the last in a physical chain (nothing wired into the Link input). Toggled via address **889**. Default `false`, via `DEFAULT_END_OF_LINE_ENABLED`. Resets to `false` on factory reset like every other field (no exemption) — the one board that needs this set either has 889 re-sent after a reset, or is flashed with the default constant set `true` for that specific board.
- `bool aspectMemoryEnabled` — toggled via address **998**, same pattern as above. Default `false`, via `DEFAULT_ASPECT_MEMORY_ENABLED`. Decoupled from the Learn cycle — relearning a signal's address does not touch this flag.

Special-address table:

| Address | Toggles |
|---|---|
| 888 | `aspectLinkEnabled` |
| 889 | `endOfLineEnabled` |
| 998 | `aspectMemoryEnabled` |
| 999 | Brightness (existing) |

### Priority order for displayed aspect (highest first)

1. **DCC command** — always sets the displayed aspect immediately, whether or not this block is currently occupied. Deliberate: allows the command station or layout software to move a train through/past a signal during a breakdown or shunting move, with no separate mode or address needed.
2. **Cascade-derived aspect** — applies whenever no more recent DCC command is in force, derived from the aspect byte received on the Link input (see derivation table below). Skipped entirely if `endOfLineEnabled` is true.

Own-block occupancy does **not** block a DCC command from setting the displayed aspect — its role is in what gets transmitted to the block behind, and in triggering a re-read once it clears (both below).

### Cascade aspect derivation table

Applied only when `aspectLinkEnabled` is true, `endOfLineEnabled` is false, and no DCC command is currently in force:

| Aspect received on Link input | 3/4 Aspect shows | 2 Aspect shows |
|---|---|---|
| Red | Yellow | Green |
| Yellow | Double Yellow | Green |
| Double Yellow | Green | Green |
| Green | Green | Green |

### End-of-line behaviour (`endOfLineEnabled` = true)

No Link-input read, no derivation table — the simplified two-state rule, regardless of signal type:
- Occupied → Red (unless DCC-overridden)
- Clear → Green

TX is unaffected by this flag and continues to operate normally — a board can have nothing wired ahead of it (RX ignored) while still having a real board wired behind it (TX still needed).

### What gets transmitted on Link OUT (key safety rule)

Displayed aspect and transmitted aspect are not always the same value:
- **If this block's own occupancy is currently active** → TX always sends **Red**, regardless of what's actually displayed locally (even if a DCC command has manually set the display to Green, e.g. for shunting past a broken-down train).
- **Else** → TX sends whatever is currently displayed.

This prevents a manually-overridden "clear" aspect from ever being seen by the block behind while a train is still physically present — the override is visible to a human standing at the signal, but invisible to the automatic cascade chain.

### Cascade heartbeat (`CASCADE_HEARTBEAT_MS`, default 1000ms)

The current aspect is re-sent on Link OUT on a fixed interval, **not just when it changes**. Found necessary during first real 2-board bench testing: without this, a board that reboots (or a Link that's only just been wired up) has no way to learn the *current settled state* of the board ahead — it only ever hears about *future* changes to it, and can be left believing its default "assume clear" starting guess indefinitely if the upstream board simply isn't changing at that moment. Set to `0` to disable (on-change sending only).

### Resume-on-occupancy-clear

The moment this board's own occupancy transitions from active → clear, it discards whatever was previously displayed (including any prior DCC override) and performs a fresh cascade read (or the end-of-line rule) against the current state of the block ahead. No timer, no "resume auto" command — occupancy clearing is the only trigger for discarding a DCC override.

A genuinely new byte arriving on the Link input is now **also** its own trigger for an immediate re-derive (not just remembered for the next time this board's own occupancy happens to clear) — this was a real bug found during 2-board testing (see "Key debugging lessons" below), fixed via a new `dccAspectOverrideActive` flag that stops a fresh Link byte from ever silently clobbering an active DCC override; only an occupancy-clear edge on *this* board is still allowed to discard one.

### Interaction with Aspect Memory

If `aspectMemoryEnabled` **and** `aspectLinkEnabled` are both true, saved aspect/route are ignored entirely on boot — occupancy/cascade determine true current state fresh, since a saved value is essentially guesswork compared to what occupancy can tell you directly. Aspect Memory only governs boot behaviour when Aspect Link is disabled.

### Occupancy clear-debounce (`OCCUPANCY_CLEAR_DEBOUNCE_MS`, default 2000ms)

Replaces the hardware debounce capacitor with a firmware equivalent, so the timeout is configurable per board instead of fixed by component value:
- **Entering occupied is always instant** — no debounce, by design (a safety property, not something to soften).
- **Clearing is debounced** — the pin must read clear continuously for the whole window before it's believed. Any flicker back to occupied during that window (dirty track, momentary lost wheel contact) restarts the countdown from zero — a proper debounce, not a fixed delay.
- Set to `0` for raw/undebounced reading (old capacitor-only behaviour, if the capacitor is left in place).
- The `occ on/off/auto` bench-test simulation deliberately **bypasses** this debounce entirely, so bench testing stays instantly responsive regardless of the configured window.
- `status` shows a live countdown (`Clear Debounce: pending, Nms remaining`) while a clear is being debounced.

### Considered and deliberately NOT implemented: cosmetic delay on aspect changes

Discussed at length: whether to add a deliberate delay to the visual transition itself (e.g. Board 2 waiting a moment before switching to Yellow after Board 1 goes Red), purely so the reaction looks less abrupt. Decided against, for two reasons: (1) it would reopen exactly the "false clearance" gap the TX-always-Red-while-occupied rule was built to close, if applied to the Red-broadcast path; (2) research into how real block signals behave confirmed genuine track circuits react in tens of milliseconds with no deliberate softening — the instant reaction is realistic, not something that needs disguising. The occupancy clear-debounce above solves a *real* problem (dirty track / detector chatter); this would have solved a cosmetic preference only, and was set aside.

### New hardware

- **A0** – Occupancy input for this signal's own block. Bridges to GND when occupied.
- **A1** – Link IN (SoftwareSerial RX), receives the aspect byte from the block ahead. Ignored if `endOfLineEnabled`.
- **A2** – Link OUT (SoftwareSerial TX), transmits this board's broadcast aspect to the block behind.
- SoftwareSerial used deliberately so hardware Serial (D0/D1) stays free for the existing bench-test CLI.
- Ground shared via common DC bus — no dedicated ground wire needed between boards on the final layout (bench-testing two separate Nanos on separate USB supplies DOES need an explicit GND-to-GND jumper between them — see "Key debugging lessons" below).

### Bench-test CLI (current)

- `occ <on|off|auto>` — simulate local occupancy state without a physical detector wired up; `auto` returns to reading the real pin. Bypasses the clear-debounce entirely.
- `cascade <aspect>` — simulate a received Link byte from the block ahead. Now correctly triggers an immediate re-derive (same guards as the real Link: skipped if this board is occupied, or if a DCC override is active) — this was fixed after being found not to work during bench testing; it originally only stored the simulated value without ever acting on it. Also correctly no-ops with an explanatory message if `aspectLinkEnabled` is off, matching how a real board would never even read the byte in that state.
- `status` output includes: Aspect Link enabled/disabled, end-of-line enabled/disabled, current occupancy state, clear-debounce countdown (if pending), DCC override active/not active, last Link byte received/transmitted.

## Key debugging lessons from first real 2-board bench test

- **Wiring direction**: the Link is directional — Board A's OUT (A2) must go to Board B's IN (A1), not the same-letter pin on both boards. An easy mistake to make when wiring two identical boards side by side.
- **Common ground required** between two separately-USB-powered Nanos for SoftwareSerial to work at all — not needed on the final layout (shared DC bus), but essential on the bench.
- **Stale bench-test state carries across a link test.** Running `occ on` or a manual `dcc <addr> <closed|thrown>` command on a board during earlier, unrelated bench testing leaves that board's occupancy-simulation or DCC-override state latched — neither expires on its own. If that board is then wired into a fresh Link test, it can silently ignore the other board indefinitely, with no obvious cause. Power-cycling the board (or explicitly sending `occ auto`) clears this. Worth resetting a board before starting a fresh Link test if it's been individually poked with `dcc`/`occ` commands beforehand.
- **A board that joins the Link late (e.g. after a reboot) needs the heartbeat** to learn the current state — without it, it's stuck on its default "assume clear" guess until the upstream board's state happens to change again. This is what `CASCADE_HEARTBEAT_MS` fixes.
- **Factory Reset (and a real Learn) could leave a board stuck on Red instead of resolving from the block ahead.** Found via Board 2 going to Red on Factory Reset but correctly resolving to match Board 1 on a subsequent reboot. Root cause: `applyDefaultAddress()` (which runs automatically after Factory Reset when `USE_DEFAULT_DCC_ADDRESS` is on) and `completeLearnMode()` (the real Learn path) both predated Aspect Link and unconditionally forced Red with no check for it — unlike `applyStartupState()`, which already had the correct logic, which is exactly why a reboot always "fixed" it. Fixed by consolidating the "establish a live aspect from current occupancy/cascade state" logic into one shared function (`establishLiveAspectFromOccupancy()`), now called from all three places that need it (`toggleAspectLink()`, `completeLearnMode()`, `applyDefaultAddress()`), so a future change can't drift the same way again. `applyStartupState()` keeps its own separate variant, since it needs to stay quiet on Serial during boot.

## Testing completed

- DIP switches and Learn button bench-tested via Serial Monitor — confirmed working.
- Full Learn Mode cycle simulated for all 4 signal-type/route combinations — confirmed working.
- Brightness control, EEPROM checksum protection, and Factory Reset confirmed working.
- Two real Arduino Nanos linked via A1/A2, common ground, confirmed working end-to-end over Serial Monitor: occupancy on Board 1 correctly cascades to Yellow on Board 2, DCC override on Board 2 correctly holds against further Link updates until Board 2's own occupancy clears, and the heartbeat correctly allows Board 2 to catch up to Board 1's real state after simulated testing left it stale.
- Command station + sniffer wiring: connected, real-world Learn/operate testing ongoing.

## Testing needed

- **Full live-track test, blocked on the PCB arriving.** Everything below the Nano-to-Nano/jumper-wire bench level (real detector board wired via A0/GND, real train, real track) is still pending — this is the immediate next step once the PCB is in hand.
- **Occupancy clear-debounce** (`OCCUPANCY_CLEAR_DEBOUNCE_MS`) not yet tested against a real occupancy detector board on real track — bench-confirmed logic only so far (via a jumper wire to A0/GND standing in for the detector, and via `occ`, which deliberately bypasses the debounce). Worth deliberately testing a genuinely dirty/intermittent track moment to confirm the countdown resets rather than just delays.
- End Of Line behaviour not yet tested with a real third board in a genuine 3-board chain (only 2 boards tested so far).
- DCC-command-overrides-occupancy behaviour not yet exercised with a real occupancy detector (only via `occ`/`dcc` bench commands so far).
- Aspect Memory power-cycle behaviour not yet re-verified since moving to the address-998 toggle mechanism.
- **Watchdog** not yet flashed/tested on real hardware — worth confirming a full Learn cycle (the longest legitimate blocking sequence, ~1.7s) does NOT trigger a spurious reboot, before relying on it in the field. Deliberately forcing a genuine hang to confirm real recovery is harder to test cleanly and lower priority than confirming it doesn't misfire during normal use.
- **Re-verify the Factory-Reset-stuck-on-Red fix on real hardware** (not yet re-tested since the fix), and additionally confirm the same fix works via a genuine Learn Mode re-teach on a live Aspect-Link-enabled board — that half of the bug was found and fixed by code inspection alongside the reported symptom, not separately observed/reproduced.

## Files referenced in this project

- `DCC_Signal_Decoder_v1_4.ino` — current working Nano sketch. Note: upload this to Project files as `DCC_Signal_Decoder_v1_4.txt` (identical content) — `.ino` is not an accepted Project file type in Claude, only the extension differs, rename back to `.ino` for the Arduino IDE.
- `DCC_Signal_Decoder_Project_Plan_V1.docx` — original project plan/spec
- `DCC_Sniffer_Circuit.png` — confirmed 6N137 DCC input circuit
- `DCC_Signal_Decoder_Firmware_Configuration_Guide_v1_4.docx` — updated for v1.4, including Aspect Link naming, cascade heartbeat, occupancy clear-debounce, and PCB migration notes (Section 11) removed. Blue/navy accent style.
- `DCC_Signal_Decoder_User_Guide_v1_4.docx` — updated for v1.4, fully consistent "Aspect Link" terminology throughout (no more "Occupancy Detection" wording anywhere). Green accent style.

Both guides follow the same visual template (see Claude's saved memory for this project for the exact style spec: Arial, solid accent-colour table headers with white bold text, pale alternating rows, bold accent-coloured headings, left-bar-only callout boxes, small running header + simple page-number footer) — only the accent colour differs (green for the User Guide, blue/navy for the Firmware Guide).

**To keep this available in new chats:** the files above need to be added to this Project's files by you — Claude cannot write into a Project's file store directly, only produce files as conversation outputs. Replacing the older versions already in the Project with these v1.4 ones is what makes a fresh chat able to pick up exactly where this one left off.
