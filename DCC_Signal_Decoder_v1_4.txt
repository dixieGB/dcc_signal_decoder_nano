/*
DCC_Signal_Decoder_v1_4.ino

Full design rationale, pin assignments, build-time configuration options,
and bench-test command reference are documented in the Firmware
Configuration & Maintenance Guide (v1.4) - refer to that document rather
than duplicating it here. This header is intentionally kept short.
*/


#include <EEPROM.h>
#include <NmraDcc.h>
#include <SoftwareSerial.h>
#include <avr/wdt.h>  // hardware watchdog timer - see WATCHDOG_TIMEOUT_SETTING
#include <stddef.h>   // for offsetof(), used by calculateChecksum()

const byte PIN_DCC_INPUT     = 2;
const byte PIN_RED           = 3;
const byte PIN_STATUS_LED    = 4;
const byte PIN_DOUBLE_YELLOW = 5;
const byte PIN_GREEN         = 6;
const byte PIN_DIP_1         = 7;
const byte PIN_LEARN_BUTTON  = 8;
const byte PIN_YELLOW        = 9;
const byte PIN_ROUTE         = 10;
const byte PIN_DIP_2         = 11;
// D12 reserved - intentionally left unused
// D13 reserved - intentionally left unused
const byte PIN_OCCUPANCY_INPUT = A0; // track occupancy detector, this signal's own block
const byte PIN_CASCADE_RX      = A1; // Link IN  - receives the aspect broadcast FROM the block AHEAD (the signal in front of this one)
const byte PIN_CASCADE_TX      = A2; // Link OUT - transmits this board's own aspect TO the block BEHIND (the signal following this one)

// ======================================================
// SWITCH POLARITY
// ------------------------------------------------------
// Assumes the Learn button and DIP switches are wired to GND
// and use the Nano's internal pull-ups (INPUT_PULLUP).
// ON / Pressed = LOW. If your hardware is wired the other way
// round, just flip these two constants.
// ======================================================
const int DIP_ACTIVE_STATE     = LOW;
const int BUTTON_PRESSED_STATE = LOW;

// The track occupancy detector's output is read the same way - one
// state means "occupied". Uses INPUT_PULLUP like the switches above;
// harmless even if the detector actively drives the pin rather than
// just pulling it to GND. Flip this if a given detector board's output
// is the opposite polarity.
const int OCCUPANCY_ACTIVE_STATE = LOW;

// Going occupied is always believed instantly - that's a safety
// property, not something to soften. Going clear is debounced: the pin
// must read clear continuously for this whole window before it's
// believed, exactly like the capacitor on the detector circuit already
// does in hardware - any interruption (dirty track, a momentary lost
// wheel contact) restarts the countdown from zero. Set to 0 to disable
// (raw, undebounced pin reading). Only applies to the real pin - the
// `occ on/off` bench-test simulation bypasses this entirely, so bench
// testing stays instantly responsive.
const unsigned long OCCUPANCY_CLEAR_DEBOUNCE_MS = 2000;

// ======================================================
// DEFAULT DCC ADDRESS (BENCH TESTING CONVENIENCE)
// ------------------------------------------------------
// When true, a board with no learned address yet (fresh EEPROM, OR
// just Factory Reset) will automatically configure itself using
// DEFAULT_DCC_ADDRESS and whatever the DIP switches are currently
// set to - no physical button press or DCC command needed. Learn
// Mode still works as normal afterwards if you want to change it.
//
// Set to false before deploying boards permanently. With it left
// on, every freshly-flashed board defaults to the SAME address,
// so two boards installed without an explicit Learn would both
// respond to the same commands - exactly the situation Learn Mode
// exists to avoid.
// ======================================================
const bool     USE_DEFAULT_DCC_ADDRESS = true;
const uint16_t DEFAULT_DCC_ADDRESS     = 1;

// ======================================================
// DCC / ADDRESS CONSTANTS
// ======================================================
const uint16_t NO_ADDR = 0;
const byte DCC_CLOSED  = 0;
const byte DCC_THROWN  = 1;

// Highest valid NMRA basic accessory decoder address. A base address
// learned near this limit could push Aspect2/Route past it once the
// +1/+2 offsets are added - checked in completeLearnMode().
const uint16_t MAX_DCC_ACCESSORY_ADDRESS = 2044;

// ======================================================
// SIGNAL TYPE CONSTANTS
// ======================================================
const byte SIGNAL_TYPE_2_ASPECT   = 2;
const byte SIGNAL_TYPE_3_4_ASPECT = 4;

// ======================================================
// FORCED SIGNAL CONFIGURATION (NO DIP SWITCHES NEEDED)
// ------------------------------------------------------
// For running on a bare Nano with no DIP switches (or no Learn
// button) physically wired at all - e.g. quick logic-only testing,
// or a fixed single-purpose build. When FORCE_SIGNAL_CONFIG is
// true, the 2 values below are used INSTEAD of reading the DIP
// switches, for both a Learn Mode entry and the Default DCC Address
// feature - the DIP pins are never read. Aspect Memory is not part
// of this - it is a persistent Learn Mode toggle, not a switch, so
// it has nothing to force here - see DEFAULT_ASPECT_MEMORY_ENABLED
// further down instead.
//
// Leave FORCE_SIGNAL_CONFIG false for a normal board with DIP
// switches fitted (including the final PCB) - the DIP switches are
// then read as normal.
// ======================================================
const bool FORCE_SIGNAL_CONFIG         = true;
const byte FORCE_SIGNAL_TYPE           = SIGNAL_TYPE_3_4_ASPECT; // SIGNAL_TYPE_2_ASPECT or SIGNAL_TYPE_3_4_ASPECT
const bool FORCE_ROUTE_ENABLED         = false;

// ======================================================
// ASPECT CONSTANTS
// ======================================================
const byte ASPECT_RED           = 1;
const byte ASPECT_YELLOW        = 2;
const byte ASPECT_DOUBLE_YELLOW = 3;
const byte ASPECT_GREEN         = 4;

// ======================================================
// DOUBLE YELLOW WIRING COMPATIBILITY
// ------------------------------------------------------
// Some 4-aspect signal heads internally wire their Double Yellow
// lamp so the Yellow lamp lights automatically whenever Double
// Yellow does (a "double yellow" aspect IS a yellow light, just with
// a second lamp alongside it). Others don't, and need both pins
// driven by the firmware for the aspect to look correct.
//
// Set to true if your signal head does NOT do this internally -
// the firmware will then drive PIN_YELLOW and PIN_DOUBLE_YELLOW
// together whenever showing the Double Yellow aspect.
// ======================================================
const bool DOUBLE_YELLOW_ALSO_LIGHTS_YELLOW = true;

// ======================================================
// BRIGHTNESS CONTROL
// ------------------------------------------------------
// All 5 LEDs (the 4 signal colours plus Route) share one 330R series
// resistor value - but different LED colours, and especially
// different LED types (e.g. a white Route LED vs coloured signal
// LEDs), draw differently at the same resistor value, so some end up
// far brighter than others. Rather than fitting a different resistor
// per LED, each one gets its own fixed PWM baseline below -
// calibrate these by eye once, then reflash; they never change at
// runtime. There's no assumption about which LED(s) will need
// pulling down - it depends entirely on the actual LEDs fitted. On
// one build it might be Green; on another, Red might be the bright
// one, or more than one LED at once. We found Route (a white LED)
// came out the brightest of the five on this build. Start every
// baseline at 100% and lower whichever ones look too bright relative
// to the others, until all 5 are matched.
//
// On top of these fixed baselines sits ONE user-adjustable global
// percentage (MIN_BRIGHTNESS_PERCENT to MAX_BRIGHTNESS_PERCENT, in
// steps of BRIGHTNESS_STEP_PERCENT) - the only thing the end user
// can ever change. It multiplies all 5 baselines together, so the
// LEDs dim as a matched set rather than drifting out of balance with
// each other. Adjusted via Learn Mode: send BRIGHTNESS_CONFIG_ADDRESS
// (either direction) to arm it - every LED lights up together at the
// current brightness so they can be compared directly - then send a
// value in the valid range to apply it live. It keeps waiting for
// another value - it does NOT auto-exit like a normal Learn - so you
// can try several before holding the Learn button for 2s to finish,
// which restores normal single-aspect display.
// ======================================================
const byte BRIGHTNESS_BASELINE_RED           = 100; // % of the 330R resistor's natural brightness
const byte BRIGHTNESS_BASELINE_YELLOW        = 100;
const byte BRIGHTNESS_BASELINE_DOUBLE_YELLOW = 100;
const byte BRIGHTNESS_BASELINE_GREEN         = 100;
const byte BRIGHTNESS_BASELINE_ROUTE         = 100;

const byte DEFAULT_GLOBAL_BRIGHTNESS_PERCENT = 100; // restored by Factory Reset / a blank EEPROM
const byte MIN_BRIGHTNESS_PERCENT            = 1;
const byte MAX_BRIGHTNESS_PERCENT            = 100;
const byte BRIGHTNESS_STEP_PERCENT           = 1;
const uint16_t BRIGHTNESS_CONFIG_ADDRESS     = 999; // sent during Learn Mode to arm brightness adjustment

// ======================================================
// ASPECT MEMORY, ASPECT LINK & END OF LINE
// ------------------------------------------------------
// Three persistent settings, each toggled via Learn Mode using its own
// dedicated address below (either direction) - exactly the same
// mechanism BRIGHTNESS_CONFIG_ADDRESS already uses, except toggling
// these exits Learn Mode immediately rather than arming a sub-mode,
// since there is nothing further to wait for. None of these three
// addresses consume a learned aspect/route address.
//
// ASPECT MEMORY (address 998) - whether the signal restores its last
// aspect/route on power up, or always starts at Red. Persists
// independently of relearning the signal's address - it only ever
// changes when explicitly toggled again.
//
// ASPECT LINK (address 888) - master on/off for reading the track
// occupancy detector on PIN_OCCUPANCY_INPUT, and exchanging aspect
// data with the adjacent board over the cascade link, at all. When
// off, none of this runs. When on: the moment this signal's own block
// becomes occupied, the aspect defaults to Red - but a DCC command
// received afterwards can still change the display at any time,
// occupied or not (e.g. moving a train past a failure, or a shunting
// move). The moment the block clears, the aspect is re-derived fresh
// from the block ahead (via the cascade link below), discarding any
// DCC override that was in effect.
//
// END OF LINE (address 889) - only meaningful when Aspect Link
// is on. Marks this board as having nothing wired into its cascade
// input - there is no further block ahead to derive a caution aspect
// from. When on, the cascade lookup is skipped entirely: Red when this
// board's own block is occupied, Green when clear, regardless of
// signal type. Typically only one board in a chain needs this set.
// ======================================================
const bool DEFAULT_ASPECT_MEMORY_ENABLED = false;
const bool DEFAULT_ASPECT_LINK_ENABLED   = true;
const bool DEFAULT_END_OF_LINE_ENABLED   = true;

const uint16_t ASPECT_MEMORY_TOGGLE_ADDRESS = 998;
const uint16_t ASPECT_LINK_TOGGLE_ADDRESS   = 888;
const uint16_t END_OF_LINE_TOGGLE_ADDRESS   = 889;

// ======================================================
// CASCADE LINK (BOARD-TO-BOARD)
// ------------------------------------------------------
// One byte is sent whenever the aspect being broadcast to the block
// behind changes - the byte value is simply the relevant ASPECT_*
// constant. SoftwareSerial is used deliberately so hardware Serial
// (D0/D1) stays free for the existing bench-test CLI over USB.
//
// The current aspect is ALSO re-sent on a fixed heartbeat, regardless
// of whether it has changed, every CASCADE_HEARTBEAT_MS. Without this,
// a board that reboots (or a link that is only just connected) would
// have no way to learn the block ahead's CURRENT settled state - it
// would only ever hear about the NEXT change, and could be left
// believing the default "assume clear" starting guess indefinitely if
// the board ahead simply isn't changing at that moment.
// ======================================================
const long CASCADE_SERIAL_BAUD = 9600;
const unsigned long CASCADE_HEARTBEAT_MS = 1000;

// ======================================================
// ROUTE CONSTANTS
// ======================================================
const byte ROUTE_OFF = 0;
const byte ROUTE_ON  = 1;

// ======================================================
// TIMING CONSTANTS
// ======================================================
const unsigned long DEBOUNCE_DELAY_MS     = 50;
const unsigned long LEARN_HOLD_MS         = 2000;
const unsigned long LEARN_MODE_TIMEOUT_MS = 60000; // auto-cancel Learn Mode if no address is received in this long
const unsigned long BRIGHTNESS_MODE_TIMEOUT_MS = 300000; // 5 minutes - adjusting by eye can take a while, longer than a normal Learn
const unsigned long FACTORY_RESET_HOLD_MS = 10000;
const unsigned long SLOW_FLASH_PERIOD_MS  = 1000;
const unsigned long FAST_FLASH_MS         = 100;
const unsigned long SUCCESS_FLASH_MS      = 150;
const unsigned long RESET_FLASH_MS        = 60;
const byte EEPROM_SAVE_BLINK_COUNT        = 4;
const byte LEARN_SUCCESS_BLINK_COUNT      = 3;
const byte FACTORY_RESET_BLINK_COUNT      = 12;

// ======================================================
// WATCHDOG
// ------------------------------------------------------
// Hardware watchdog timer - if loop() ever stops running (electrical
// noise, a brown-out, an unforeseen software fault), the chip
// force-reboots itself after this long, rather than sitting locked up
// indefinitely under the layout until someone notices and manually
// power-cycles it. A watchdog-triggered reboot is functionally
// identical to a power cycle from this firmware's point of view -
// RAM-based state (occupancy/cascade tracking, DCC override) already
// re-initialises correctly via applyStartupState(), and the existing
// EEPROM checksum/marker already treats a write interrupted mid-way as
// "not configured" rather than running on corrupted data.
//
// The AVR watchdog only accepts fixed steps (WDTO_15MS ... WDTO_8S),
// not an arbitrary millisecond value. WDTO_4S is chosen deliberately
// with real margin over the longest legitimate blocking sequence in
// this firmware - a completed Learn calls indicateEepromSaving() (4 x
// 100ms x 2 = 800ms) immediately followed by indicateLearnSuccess() (3
// x 150ms x 2 = 900ms), a genuine ~1.7s of intentional blinking, not a
// hang. A timeout anywhere near that would falsely reboot the board
// mid-confirmation-flash on every single successful Learn.
// ======================================================
const uint8_t WATCHDOG_TIMEOUT_SETTING = WDTO_4S;

// "No DCC seen" warning (Double Blip), driven by the raw notifyDccMsg()
// callback rather than just accessory commands, so it reflects whether
// DCC is present at all, not just whether THIS signal's addresses are
// being sent.
const unsigned long NO_DCC_TIMEOUT_MS    = 5000;
const unsigned long DOUBLE_BLIP_ON_MS    = 100;
const unsigned long DOUBLE_BLIP_GAP_MS   = 150;
const unsigned long DOUBLE_BLIP_PAUSE_MS = 600;
const unsigned long DOUBLE_BLIP_CYCLE_MS = (DOUBLE_BLIP_ON_MS * 2) + DOUBLE_BLIP_GAP_MS + DOUBLE_BLIP_PAUSE_MS;

// ======================================================
// EEPROM CONFIGURATION
// ======================================================
const uint16_t CONFIG_VALID_MARKER = 0xDC01;
// NmraDcc keeps its OWN internal CV storage directly in raw EEPROM,
// using the CV number as the byte address (CV1, CV29, and some
// internal factory-default bookkeeping CVs around 515-519) - broadly
// across the first ~520 bytes. Our own config struct must live well
// clear of that range, or the library can silently overwrite parts
// of it from underneath us.
const int EEPROM_CONFIG_ADDRESS = 600;

// ======================================================
// FORWARD DECLARATIONS
// ======================================================
void handleAccessoryAddress(uint16_t address, byte direction);
void processDccCommand(uint16_t address, byte direction);
void setAspect(byte aspect);
void setRoute(bool routeOn);
void enterLearnMode();
void cancelLearnMode();
void checkLearnModeTimeout();
void completeLearnMode(uint16_t baseAddress);
void allocateAddresses(uint16_t baseAddress);
void applyDefaultAddress();
void factoryReset();
byte effectiveSignalType();
bool effectiveRouteEnabled();
byte calculatePwmDuty(byte baselinePercent);
void allOn();
void enterBrightnessAdjustMode();
void exitBrightnessAdjustMode();
void applyBrightnessValue(uint16_t value);
void checkBrightnessModeTimeout();
bool readRawOccupancyPin();
void updateOccupancyDebounce();
bool isOccupied();
void resolveAspectFromCascade();
void establishLiveAspectFromOccupancy();
void updateOccupancyCascade();
void toggleAspectLink();
void toggleEndOfLine();
void toggleAspectMemory();

// ======================================================
// DECODER STATE
// ======================================================
enum DecoderState
{
  STATE_NORMAL,
  STATE_LEARN_WAITING_FOR_ADDRESS,
  STATE_LEARN_WAITING_FOR_BRIGHTNESS
};

DecoderState decoderState = STATE_NORMAL;

// ======================================================
// EEPROM-BACKED CONFIGURATION
// ======================================================
struct DecoderConfig
{
  uint16_t validityMarker;     // Must equal CONFIG_VALID_MARKER once learned
  byte     signalType;         // SIGNAL_TYPE_2_ASPECT or SIGNAL_TYPE_3_4_ASPECT
  bool     routeEnabled;
  bool     aspectMemoryEnabled; // Toggled via ASPECT_MEMORY_TOGGLE_ADDRESS (998)
  bool     aspectLinkEnabled;   // Toggled via ASPECT_LINK_TOGGLE_ADDRESS (888)
  bool     endOfLineEnabled;    // Toggled via END_OF_LINE_TOGGLE_ADDRESS (889)
  uint16_t aspectAddress1;
  uint16_t aspectAddress2;     // NO_ADDR when signalType is 2 Aspect
  uint16_t routeAddress;       // NO_ADDR when routeEnabled is false
  byte     globalBrightness;   // MIN_BRIGHTNESS_PERCENT-MAX_BRIGHTNESS_PERCENT, multiplies all 5 LED baselines together
  byte     checksum;           // Must equal calculateChecksum() of the fields
                                // ABOVE this point only. Deliberately placed
                                // before savedAspect/savedRoute, which change
                                // on every DCC command - if the checksum
                                // covered those too, it would go stale the
                                // moment the signal changed aspect with
                                // Aspect Memory disabled, locking the
                                // decoder up as "not configured".
  byte     savedAspect;        // Restored on power up when aspect memory enabled
  byte     savedRoute;         // ROUTE_OFF or ROUTE_ON
};

DecoderConfig config;

NmraDcc Dcc;

// Values captured from the DIP switches when Learn Mode is entered,
// committed to config only once a learn is successfully completed.
// Aspect Memory is not captured here - it persists independently,
// toggled directly via ASPECT_MEMORY_TOGGLE_ADDRESS.
byte pendingSignalType;
bool pendingRouteEnabled;

// Cascade link to adjacent boards - RX from the block ahead, TX to the
// block behind. Only actually used when config.aspectLinkEnabled
// is true, but initialised unconditionally; a few bytes idle on a
// SoftwareSerial link cost nothing when the feature is disabled.
SoftwareSerial cascadeSerial(PIN_CASCADE_RX, PIN_CASCADE_TX);

// Tracks whether this board's own block was occupied on the previous
// pass through the loop, so a genuine transition (just occupied / just
// cleared) can be detected, rather than re-triggering every loop.
bool previousOccupancyState = false;

// Last aspect byte actually transmitted on the cascade link, and when
// it was last sent - re-sent whenever it changes, AND on a fixed
// heartbeat regardless of change (see CASCADE_HEARTBEAT_MS above), so a
// board that (re)joins the link part-way through can still learn the
// current settled state, not just future changes to it.
byte lastCascadeTxAspect = 0; // 0 never matches a real ASPECT_* value, forces the first send
unsigned long lastCascadeTxSentTime = 0;

// Last aspect byte received from the block ahead. Defaults to Green -
// a reasonable "assume clear ahead" starting point before any real
// byte has ever been received (e.g. just after power-up).
byte lastCascadeRxAspect = ASPECT_GREEN;

// True once a DCC command has set this board's aspect, and stays true
// right through any occupancy cycle that follows (occupied or not) -
// exactly matching the existing rule that only an occupancy-clear edge
// ever discards a DCC override, never a mere incoming cascade update.
// Cleared only in updateOccupancyCascade()'s occupied->clear branch.
bool dccAspectOverrideActive = false;

// Bench-test only: lets `occ on` / `occ off` simulate the occupancy
// input over Serial without real detector hardware wired up. `occ auto`
// releases the simulation and goes back to reading the real pin.
bool serialOccupancySimActive = false;
bool serialOccupancySimState  = false;

// Duplicate DCC packet filter (kept from the proven previous decoder,
// now with a time window so a genuinely new command is never swallowed)
const unsigned long DUPLICATE_PACKET_WINDOW_MS = 250;
uint16_t lastDccAddress = 0;
int lastDccDirection = -1;
unsigned long lastDccPacketTime = 0;

bool restoringFromEEPROM = false;

// Learn button debounce / long-press state
int lastRawButtonReading = HIGH;
unsigned long lastDebounceTime = 0;
bool debouncedButtonPressed = false;
unsigned long pressStartTime = 0;
bool shortHoldActionFired = false;   // Learn Mode enter/cancel, fires at LEARN_HOLD_MS
bool longHoldActionFired = false;    // Factory reset, fires at FACTORY_RESET_HOLD_MS

// Last time ANY valid DCC packet was seen (via the raw notifyDccMsg()
// callback, not just accessory commands aimed at this decoder). Used
// to drive the "no DCC signal" status LED warning once learned.
unsigned long lastDccActivityTime = 0;

// Set when Learn Mode is entered, checked by checkLearnModeTimeout()
unsigned long learnModeEnteredTime = 0;

// Set when Brightness Adjust mode is entered, checked by checkBrightnessModeTimeout()
unsigned long brightnessModeEnteredTime = 0;


// ======================================================
// EEPROM HELPERS
// ======================================================

byte calculateChecksum()
{
  // Simple additive checksum over every config byte EXCEPT the checksum
  // field itself (which must be the last field in the struct for this
  // offsetof()-based range to be correct).
  const byte *configBytes = (const byte*)&config;
  size_t checksumOffset = offsetof(DecoderConfig, checksum);

  byte sum = 0;
  for (size_t i = 0; i < checksumOffset; i++)
  {
    sum += configBytes[i];
  }

  return sum;
}

bool isConfigured()
{
  return config.validityMarker == CONFIG_VALID_MARKER;
}

void setDefaultConfig()
{
  config.validityMarker = 0;
  config.signalType = SIGNAL_TYPE_2_ASPECT;
  config.routeEnabled = false;
  config.aspectMemoryEnabled = DEFAULT_ASPECT_MEMORY_ENABLED;
  config.aspectLinkEnabled = DEFAULT_ASPECT_LINK_ENABLED;
  config.endOfLineEnabled = DEFAULT_END_OF_LINE_ENABLED;
  config.aspectAddress1 = NO_ADDR;
  config.aspectAddress2 = NO_ADDR;
  config.routeAddress = NO_ADDR;
  config.globalBrightness = DEFAULT_GLOBAL_BRIGHTNESS_PERCENT;
  config.savedAspect = ASPECT_RED;
  config.savedRoute = ROUTE_OFF;
}

void loadConfig()
{
  EEPROM.get(EEPROM_CONFIG_ADDRESS, config);

  // Checksum is validated here ONLY - once, right after reading EEPROM.
  // It must NOT be re-checked against live RAM during normal running,
  // because savedAspect/savedRoute change in RAM every time the signal
  // is operated, with no matching EEPROM write when Aspect Memory is
  // disabled - that would make the live checksum "go stale" the very
  // first time the aspect changed, even though nothing is actually wrong.
  bool markerOk = (config.validityMarker == CONFIG_VALID_MARKER);
  bool checksumOk = (config.checksum == calculateChecksum());

  if (!markerOk || !checksumOk)
  {
    // Either this EEPROM has never been written by this firmware, or a
    // power cut during a previous save left it half-written. Either
    // way, treat as "not learned" rather than risk running on
    // corrupted addresses.
    setDefaultConfig();

    if (USE_DEFAULT_DCC_ADDRESS)
    {
      applyDefaultAddress();
    }
  }
}

void saveConfig()
{
  config.validityMarker = CONFIG_VALID_MARKER;
  config.checksum = calculateChecksum();
  EEPROM.put(EEPROM_CONFIG_ADDRESS, config);
}


// ======================================================
// SWITCH HELPERS
// ======================================================

bool isDipOn(byte pin)
{
  return digitalRead(pin) == DIP_ACTIVE_STATE;
}

// These two return the configuration that should actually be used -
// either the FORCE_SIGNAL_CONFIG values above (no DIP switches read
// at all), or the live DIP switches. Both Learn Mode entry and the
// Default DCC Address feature go through these, so they always agree
// with each other.

byte effectiveSignalType()
{
  if (FORCE_SIGNAL_CONFIG) return FORCE_SIGNAL_TYPE;
  return isDipOn(PIN_DIP_1) ? SIGNAL_TYPE_3_4_ASPECT : SIGNAL_TYPE_2_ASPECT;
}

bool effectiveRouteEnabled()
{
  if (FORCE_SIGNAL_CONFIG) return FORCE_ROUTE_ENABLED;
  return isDipOn(PIN_DIP_2);
}


// ======================================================
// SIGNAL / ROUTE OUTPUT CONTROL
// ======================================================

void allAspectsOff()
{
  analogWrite(PIN_RED, 0);
  analogWrite(PIN_YELLOW, 0);
  analogWrite(PIN_DOUBLE_YELLOW, 0);
  analogWrite(PIN_GREEN, 0);
}

byte calculatePwmDuty(byte baselinePercent)
{
  // Combines this LED's fixed baseline with the one user-adjustable
  // global percentage, then converts the result (0-100) to a 0-255
  // PWM duty cycle for analogWrite().
  uint16_t combinedPercent = ((uint16_t)baselinePercent * config.globalBrightness) / 100;
  return (byte)((combinedPercent * 255UL) / 100);
}

void allOn()
{
  // Lights every LED at once, each at its own baseline - used only
  // during Brightness Adjust, so all 5 can be compared side by side.
  // Never used during normal running, where exactly one aspect (plus
  // Route if active) should ever be lit at a time.
  analogWrite(PIN_RED, calculatePwmDuty(BRIGHTNESS_BASELINE_RED));
  analogWrite(PIN_YELLOW, calculatePwmDuty(BRIGHTNESS_BASELINE_YELLOW));
  analogWrite(PIN_DOUBLE_YELLOW, calculatePwmDuty(BRIGHTNESS_BASELINE_DOUBLE_YELLOW));
  analogWrite(PIN_GREEN, calculatePwmDuty(BRIGHTNESS_BASELINE_GREEN));
  analogWrite(PIN_ROUTE, calculatePwmDuty(BRIGHTNESS_BASELINE_ROUTE));
}

const __FlashStringHelper* aspectName(byte aspect)
{
  switch (aspect)
  {
    case ASPECT_RED:           return F("Red");
    case ASPECT_YELLOW:        return F("Yellow");
    case ASPECT_DOUBLE_YELLOW: return F("Double Yellow");
    case ASPECT_GREEN:         return F("Green");
    default:                   return F("Unknown");
  }
}

// Turns the WDTO_* enum value actually configured in
// WATCHDOG_TIMEOUT_SETTING back into readable text, so the build banner
// can never silently go stale if that constant is ever changed.
const __FlashStringHelper* watchdogTimeoutName(uint8_t setting)
{
  switch (setting)
  {
    case WDTO_15MS:  return F("15ms");
    case WDTO_30MS:  return F("30ms");
    case WDTO_60MS:  return F("60ms");
    case WDTO_120MS: return F("120ms");
    case WDTO_250MS: return F("250ms");
    case WDTO_500MS: return F("500ms");
    case WDTO_1S:    return F("1s");
    case WDTO_2S:    return F("2s");
    case WDTO_4S:    return F("4s");
    case WDTO_8S:    return F("8s");
    default:         return F("Unknown");
  }
}

void setAspect(byte aspect)
{
  allAspectsOff();

  switch (aspect)
  {
    case ASPECT_RED:    analogWrite(PIN_RED, calculatePwmDuty(BRIGHTNESS_BASELINE_RED)); break;
    case ASPECT_YELLOW: analogWrite(PIN_YELLOW, calculatePwmDuty(BRIGHTNESS_BASELINE_YELLOW)); break;

    case ASPECT_DOUBLE_YELLOW:
      analogWrite(PIN_DOUBLE_YELLOW, calculatePwmDuty(BRIGHTNESS_BASELINE_DOUBLE_YELLOW));

      if (DOUBLE_YELLOW_ALSO_LIGHTS_YELLOW)
      {
        analogWrite(PIN_YELLOW, calculatePwmDuty(BRIGHTNESS_BASELINE_YELLOW));
      }
      break;

    case ASPECT_GREEN:  analogWrite(PIN_GREEN, calculatePwmDuty(BRIGHTNESS_BASELINE_GREEN)); break;
  }

  config.savedAspect = aspect;

  if (!restoringFromEEPROM)
  {
    Serial.print(F("Aspect -> "));
    Serial.println(aspectName(aspect));
  }

  // Only save when the value will actually ever be read back. Aspect
  // Memory's saved aspect is ignored at boot whenever Aspect Link is
  // enabled (occupancy/cascade determine the real state fresh instead -
  // see applyStartupState()), so saving here in that case would just be
  // needless EEPROM wear on every cascade-derived change, for a value
  // that can never be used.
  if (config.aspectMemoryEnabled && !config.aspectLinkEnabled && !restoringFromEEPROM)
  {
    saveConfig();
  }
}

void setRoute(bool routeOn)
{
  analogWrite(PIN_ROUTE, routeOn ? calculatePwmDuty(BRIGHTNESS_BASELINE_ROUTE) : 0);
  config.savedRoute = routeOn ? ROUTE_ON : ROUTE_OFF;

  if (!restoringFromEEPROM)
  {
    Serial.print(F("Route -> "));
    Serial.println(routeOn ? F("ON") : F("OFF"));
  }

  if (config.aspectMemoryEnabled && !restoringFromEEPROM)
  {
    saveConfig();
  }
}

void applyStartupState()
{
  restoringFromEEPROM = true;

  // Aspect Memory is deliberately ignored when Aspect Link is
  // enabled - occupancy/cascade can determine the TRUE current state
  // fresh, so a saved value would just be a stale guess by comparison.
  if (isConfigured() && config.aspectMemoryEnabled && !config.aspectLinkEnabled)
  {
    setAspect(config.savedAspect);
    setRoute(config.savedRoute == ROUTE_ON);
  }
  else
  {
    setAspect(ASPECT_RED);
    setRoute(false);
  }

  restoringFromEEPROM = false;

  // If Aspect Link is on, don't leave the signal on the
  // temporary Red set above unless the block genuinely is occupied -
  // resolve a real aspect immediately, equivalent to what would happen
  // if occupancy had just cleared. Kept quiet on Serial like the rest
  // of startup - the resulting aspect is already shown in
  // printConfigSummary()'s "Current Aspect" line below.
  if (isConfigured() && config.aspectLinkEnabled)
  {
    previousOccupancyState = isOccupied(); // seed state so the loop doesn't see a false edge

    if (!previousOccupancyState)
    {
      restoringFromEEPROM = true;
      resolveAspectFromCascade();
      restoringFromEEPROM = false;
    }
    // else: genuinely occupied at boot - Red (already set above) is correct
  }
}


// ======================================================
// STATUS LED
// ======================================================

void updateStatusLed()
{
  switch (decoderState)
  {
    case STATE_NORMAL:
      if (!isConfigured())
      {
        digitalWrite(PIN_STATUS_LED, HIGH); // Solid ON: not yet learned
      }
      else if (millis() - lastDccActivityTime > NO_DCC_TIMEOUT_MS)
      {
        updateNoDccBlip(); // Double Blip: learned, but no DCC seen recently
      }
      else
      {
        digitalWrite(PIN_STATUS_LED, LOW); // OFF: normal running, DCC present
      }
      break;

    case STATE_LEARN_WAITING_FOR_ADDRESS:
    case STATE_LEARN_WAITING_FOR_BRIGHTNESS:
    {
      // Both share the same slow flash - both are "a config mode is
      // active, send an address" states, just waiting for a different
      // kind of address.
      unsigned long phase = millis() % SLOW_FLASH_PERIOD_MS;
      digitalWrite(PIN_STATUS_LED, phase < (SLOW_FLASH_PERIOD_MS / 2) ? HIGH : LOW);
      break;
    }
  }
}

void updateNoDccBlip()
{
  // Two short blips then a pause, repeating - visually distinct from
  // both the steady Slow Flash (Learn Mode) and Solid ON (not learned).
  unsigned long phase = millis() % DOUBLE_BLIP_CYCLE_MS;

  bool firstBlip  = phase < DOUBLE_BLIP_ON_MS;
  bool secondBlip = (phase >= (DOUBLE_BLIP_ON_MS + DOUBLE_BLIP_GAP_MS)) &&
                     (phase <  (DOUBLE_BLIP_ON_MS + DOUBLE_BLIP_GAP_MS + DOUBLE_BLIP_ON_MS));

  digitalWrite(PIN_STATUS_LED, (firstBlip || secondBlip) ? HIGH : LOW);
}

// Brief blocking blink sequences for one-off events only (EEPROM save,
// Learn success). These are short, infrequent admin actions, not part
// of the real-time DCC decode path, so a short pause here is acceptable.

void indicateEepromSaving()
{
  for (byte i = 0; i < EEPROM_SAVE_BLINK_COUNT; i++)
  {
    digitalWrite(PIN_STATUS_LED, HIGH);
    delay(FAST_FLASH_MS);
    digitalWrite(PIN_STATUS_LED, LOW);
    delay(FAST_FLASH_MS);
  }
}

void indicateLearnSuccess()
{
  for (byte i = 0; i < LEARN_SUCCESS_BLINK_COUNT; i++)
  {
    digitalWrite(PIN_STATUS_LED, HIGH);
    delay(SUCCESS_FLASH_MS);
    digitalWrite(PIN_STATUS_LED, LOW);
    delay(SUCCESS_FLASH_MS);
  }
}

void indicateFactoryReset()
{
  for (byte i = 0; i < FACTORY_RESET_BLINK_COUNT; i++)
  {
    digitalWrite(PIN_STATUS_LED, HIGH);
    delay(RESET_FLASH_MS);
    digitalWrite(PIN_STATUS_LED, LOW);
    delay(RESET_FLASH_MS);
  }
}


// ======================================================
// LEARN MODE
// ======================================================

void enterLearnMode()
{
  pendingSignalType = effectiveSignalType();
  pendingRouteEnabled = effectiveRouteEnabled();

  decoderState = STATE_LEARN_WAITING_FOR_ADDRESS;
  learnModeEnteredTime = millis();

  Serial.println();
  Serial.println(F("Learn Mode: waiting for one DCC accessory command..."));
  Serial.print(F("  Signal Type   : "));
  Serial.println(pendingSignalType == SIGNAL_TYPE_2_ASPECT ? F("2 Aspect") : F("3/4 Aspect"));
  Serial.print(F("  Route         : "));
  Serial.println(pendingRouteEnabled ? F("Enabled") : F("Disabled"));
  Serial.print(F("  (send "));
  Serial.print(ASPECT_MEMORY_TOGGLE_ADDRESS);
  Serial.print(F("/"));
  Serial.print(ASPECT_LINK_TOGGLE_ADDRESS);
  Serial.print(F("/"));
  Serial.print(END_OF_LINE_TOGGLE_ADDRESS);
  Serial.print(F("/"));
  Serial.print(BRIGHTNESS_CONFIG_ADDRESS);
  Serial.println(F(" instead to toggle Aspect Memory/Aspect Link/End Of Line/Brightness)"));
}

void cancelLearnMode()
{
  decoderState = STATE_NORMAL;
  Serial.println(F("Learn Mode cancelled, no changes made."));
}

void checkLearnModeTimeout()
{
  if (decoderState != STATE_LEARN_WAITING_FOR_ADDRESS)
  {
    return;
  }

  if (millis() - learnModeEnteredTime >= LEARN_MODE_TIMEOUT_MS)
  {
    // Set the state directly rather than calling cancelLearnMode() -
    // that prints its own generic "cancelled" message, which would
    // otherwise show up right under this one for the same event.
    decoderState = STATE_NORMAL;
    Serial.println(F("Learn Mode timed out - no address received, exiting."));
  }
}


// ======================================================
// BRIGHTNESS ADJUST MODE
// ------------------------------------------------------
// Entered from Learn Mode by sending BRIGHTNESS_CONFIG_ADDRESS,
// instead of a normal address to learn. Unlike a normal Learn, this
// does NOT auto-exit once a value is applied - it keeps waiting for
// another, so several values can be tried and compared live. Only
// the 2 second button hold (same action that would otherwise cancel
// a normal Learn) exits it.
// ======================================================

void enterBrightnessAdjustMode()
{
  decoderState = STATE_LEARN_WAITING_FOR_BRIGHTNESS;
  brightnessModeEnteredTime = millis();

  allOn(); // every LED together at the current brightness, for comparison

  Serial.println();
  Serial.print(F("Brightness Adjust: send a value "));
  Serial.print(MIN_BRIGHTNESS_PERCENT);
  Serial.print(F("-"));
  Serial.print(MAX_BRIGHTNESS_PERCENT);
  Serial.print(F(" (steps of "));
  Serial.print(BRIGHTNESS_STEP_PERCENT);
  Serial.println(F(")."));
  Serial.print(F("Current global brightness: "));
  Serial.print(config.globalBrightness);
  Serial.println(F("%"));
  Serial.println(F("Hold the Learn button for 2s when finished."));
}

void exitBrightnessAdjustMode()
{
  decoderState = STATE_NORMAL;

  // Brightness Adjust deliberately lights every LED at once - restore
  // normal single-aspect (+ Route) display now that it's finished.
  restoringFromEEPROM = true;
  setAspect(config.savedAspect);
  setRoute(config.savedRoute == ROUTE_ON);
  restoringFromEEPROM = false;

  Serial.println(F("Brightness Adjust finished."));
}

void applyBrightnessValue(uint16_t value)
{
  bool validRange = (value >= MIN_BRIGHTNESS_PERCENT) && (value <= MAX_BRIGHTNESS_PERCENT);
  bool validStep = (value % BRIGHTNESS_STEP_PERCENT) == 0;

  if (!validRange || !validStep)
  {
    Serial.print(F("Brightness value ignored: "));
    Serial.print(value);
    Serial.print(F(" - must be "));
    Serial.print(MIN_BRIGHTNESS_PERCENT);
    Serial.print(F("-"));
    Serial.print(MAX_BRIGHTNESS_PERCENT);
    Serial.print(F(" in steps of "));
    Serial.print(BRIGHTNESS_STEP_PERCENT);
    Serial.println(F(". Still waiting."));
    return; // stay in STATE_LEARN_WAITING_FOR_BRIGHTNESS
  }

  config.globalBrightness = (byte)value;
  saveConfig();

  allOn(); // re-light everything at the new brightness, live

  brightnessModeEnteredTime = millis(); // still actively adjusting - reset the timeout

  Serial.print(F("Global brightness -> "));
  Serial.print(config.globalBrightness);
  Serial.println(F("%. Send another value, or hold 2s to finish."));
}

void checkBrightnessModeTimeout()
{
  if (decoderState != STATE_LEARN_WAITING_FOR_BRIGHTNESS)
  {
    return;
  }

  if (millis() - brightnessModeEnteredTime >= BRIGHTNESS_MODE_TIMEOUT_MS)
  {
    // Set the state directly rather than calling exitBrightnessAdjustMode()
    // - that prints its own "finished" message, which would otherwise
    // show up right under this one for the same event. Still need to
    // restore normal display though, since allOn() left every LED lit.
    decoderState = STATE_NORMAL;

    restoringFromEEPROM = true;
    setAspect(config.savedAspect);
    setRoute(config.savedRoute == ROUTE_ON);
    restoringFromEEPROM = false;

    Serial.println(F("Brightness Adjust timed out, exiting."));
  }
}


// ======================================================
// OCCUPANCY DETECTION & CASCADE LINK
// ------------------------------------------------------
// Only active when config.aspectLinkEnabled is true - when
// false, no occupancy pin is read and nothing is sent or received on
// the cascade link.
// ======================================================

bool readRawOccupancyPin()
{
  return digitalRead(PIN_OCCUPANCY_INPUT) == OCCUPANCY_ACTIVE_STATE;
}

// updateOccupancyDebounce() must run every loop (see updateOccupancyCascade())
// to keep this current - isOccupied() itself just returns the result.
bool debouncedOccupancyState = false;
bool clearDebouncePending = false;
unsigned long clearDebounceStartedAt = 0;

void updateOccupancyDebounce()
{
  bool rawOccupied = readRawOccupancyPin();

  if (rawOccupied)
  {
    // Any occupied reading is believed immediately, and cancels any
    // clear countdown in progress - going occupied is never delayed.
    debouncedOccupancyState = true;
    clearDebouncePending = false;
    return;
  }

  // Raw pin currently reads clear.
  if (!debouncedOccupancyState)
  {
    return; // already believed clear - nothing to debounce
  }

  if (!clearDebouncePending)
  {
    // Pin has just started reading clear while still believed occupied -
    // start the countdown.
    clearDebouncePending = true;
    clearDebounceStartedAt = millis();
    return;
  }

  if (millis() - clearDebounceStartedAt >= OCCUPANCY_CLEAR_DEBOUNCE_MS)
  {
    // Read clear continuously for the whole window, uninterrupted -
    // now believe it.
    debouncedOccupancyState = false;
    clearDebouncePending = false;
  }
  // else: still within the window, keep waiting - if the pin flickers
  // back to occupied on any later pass, the check above resets this
  // countdown from zero.
}

bool isOccupied()
{
  // Bench-test simulation takes priority over the real pin, and
  // deliberately bypasses the clear-debounce entirely, so occupancy
  // can be exercised over Serial with immediate, predictable results.
  if (serialOccupancySimActive)
  {
    return serialOccupancySimState;
  }

  return debouncedOccupancyState;
}

void resolveAspectFromCascade()
{
  // Called once this board's own occupancy clears (or at boot, if
  // already clear) - re-derives the aspect fresh from whatever is
  // currently known, discarding any previous DCC override. Route is
  // untouched; only the aspect is ever driven by occupancy/cascade.
  if (config.endOfLineEnabled)
  {
    // Nothing wired ahead - no caution aspect to derive, a clear block
    // simply shows Green, regardless of signal type.
    setAspect(ASPECT_GREEN);
    return;
  }

  if (config.signalType == SIGNAL_TYPE_2_ASPECT)
  {
    // 2 Aspect has no intermediate caution aspect - Red only ever
    // comes from THIS block's own occupancy, never from the cascade.
    setAspect(ASPECT_GREEN);
    return;
  }

  switch (lastCascadeRxAspect)
  {
    case ASPECT_RED:    setAspect(ASPECT_YELLOW); break;
    case ASPECT_YELLOW: setAspect(ASPECT_DOUBLE_YELLOW); break;
    default:            setAspect(ASPECT_GREEN); break; // Double Yellow or Green ahead
  }
}

// Establishes a real, live aspect from current occupancy/cascade state,
// rather than leaving the signal on whatever it happened to be showing
// beforehand. Only meaningful (call this) when config.aspectLinkEnabled
// is true. Used identically in every situation that establishes a
// "starting" aspect for a live signal OUTSIDE of a genuine reboot: a
// completed Learn (completeLearnMode()), the bench-convenience
// default-address path (applyDefaultAddress()), and toggling Aspect
// Link on via Learn Mode (toggleAspectLink()) - one shared function so
// these three can never drift out of sync with each other again, the
// way completeLearnMode()/applyDefaultAddress() did before this was
// factored out (found via real 2-board bench testing - Factory Reset
// left a board stuck on Red instead of resolving from the block ahead,
// because applyDefaultAddress() ran the old unconditional-Red logic and
// was never updated when Aspect Link was added).
//
// applyStartupState() (a genuine reboot) does NOT use this - it needs
// its own restoringFromEEPROM-wrapped variant to keep boot's Serial
// output quiet, which this deliberately does not do.
void establishLiveAspectFromOccupancy()
{
  previousOccupancyState = isOccupied(); // seed state so the loop doesn't see a false edge

  if (previousOccupancyState)
  {
    setAspect(ASPECT_RED); // safe even if already Red - occupied always means Red
  }
  else
  {
    resolveAspectFromCascade();
  }
}

void updateCascadeRx()
{
  while (cascadeSerial.available())
  {
    byte receivedByte = cascadeSerial.read();

    // Only accept recognised aspect values - guards against line noise
    // or a genuinely floating/disconnected input being misread.
    if (receivedByte == ASPECT_RED || receivedByte == ASPECT_YELLOW ||
        receivedByte == ASPECT_DOUBLE_YELLOW || receivedByte == ASPECT_GREEN)
    {
      if (receivedByte != lastCascadeRxAspect)
      {
        lastCascadeRxAspect = receivedByte;

        // A genuinely new byte from the block ahead is itself a trigger
        // to re-derive this board's aspect right now - not just something
        // to remember for the next time OUR OWN occupancy happens to
        // clear. Without this, a board whose own block rarely or never
        // goes occupied would just sit on a stale aspect forever, never
        // reacting to the block ahead changing at all. Skipped while
        // this board is occupied (own occupancy always wins locally) or
        // under an active DCC override (which only an occupancy-clear
        // edge is allowed to discard) - both exactly as already applies
        // everywhere else in this priority model.
        if (!isOccupied() && !dccAspectOverrideActive)
        {
          resolveAspectFromCascade();
        }
      }
    }
  }
}

void updateCascadeTx()
{
  // While this block is occupied, ALWAYS broadcast Red, regardless of
  // what is actually displayed locally - this is what stops a DCC
  // override (e.g. for shunting) from ever telling the block behind
  // that the line is clear when it genuinely isn't. Otherwise,
  // broadcast whatever aspect is currently displayed.
  byte aspectToBroadcast = isOccupied() ? ASPECT_RED : config.savedAspect;

  bool aspectChanged  = (aspectToBroadcast != lastCascadeTxAspect);
  bool heartbeatDue   = (millis() - lastCascadeTxSentTime >= CASCADE_HEARTBEAT_MS);

  if (aspectChanged || heartbeatDue)
  {
    cascadeSerial.write(aspectToBroadcast);
    lastCascadeTxAspect = aspectToBroadcast;
    lastCascadeTxSentTime = millis();
  }
}

void updateOccupancyCascade()
{
  if (!config.aspectLinkEnabled)
  {
    return; // feature disabled - no pin reads, no cascade activity at all
  }

  if (decoderState != STATE_NORMAL)
  {
    return; // don't interfere with Learn Mode or Brightness Adjust
  }

  updateOccupancyDebounce(); // keeps debouncedOccupancyState current - see isOccupied()

  bool occupiedNow = isOccupied();

  if (occupiedNow && !previousOccupancyState)
  {
    // Just became occupied - force Red as the default. A DCC command
    // received after this point can still override the display (see
    // processDccCommand()) - only updateCascadeTx() above unconditionally
    // forces Red on the wire while occupied, never the local display.
    setAspect(ASPECT_RED);
  }
  else if (!occupiedNow && previousOccupancyState)
  {
    // Just cleared - re-derive the aspect fresh from the block ahead,
    // discarding whatever was displayed before (including any DCC
    // override that was in effect while occupied). This is the ONE
    // point that ever discards an override - a mere incoming cascade
    // byte must never do this on its own (see updateCascadeRx()).
    dccAspectOverrideActive = false;
    resolveAspectFromCascade();
  }

  previousOccupancyState = occupiedNow;

  updateCascadeRx();
  updateCascadeTx();
}


// ======================================================
// LEARN MODE TOGGLE ADDRESSES (Aspect Memory / Occupancy / End Of Line)
// ------------------------------------------------------
// All three follow the same pattern: toggle the persistent flag, save
// it, give the same short feedback a completed Learn would, and exit
// Learn Mode immediately - there is nothing further to wait for.
// ======================================================

void toggleAspectMemory()
{
  config.aspectMemoryEnabled = !config.aspectMemoryEnabled;
  saveConfig();
  indicateEepromSaving();

  decoderState = STATE_NORMAL;

  Serial.print(F("Aspect Memory -> "));
  Serial.println(config.aspectMemoryEnabled ? F("Enabled") : F("Disabled"));
}

void toggleAspectLink()
{
  config.aspectLinkEnabled = !config.aspectLinkEnabled;
  saveConfig();
  indicateEepromSaving();

  decoderState = STATE_NORMAL;

  // Re-seed occupancy/cascade state immediately, same as at boot, so
  // toggling this on doesn't wait for a spurious "just changed" edge
  // on the next loop to establish a real aspect.
  if (config.aspectLinkEnabled)
  {
    establishLiveAspectFromOccupancy();
  }

  Serial.print(F("Aspect Link -> "));
  Serial.println(config.aspectLinkEnabled ? F("Enabled") : F("Disabled"));
}

void toggleEndOfLine()
{
  config.endOfLineEnabled = !config.endOfLineEnabled;
  saveConfig();
  indicateEepromSaving();

  decoderState = STATE_NORMAL;

  Serial.print(F("End Of Line -> "));
  Serial.println(config.endOfLineEnabled ? F("Enabled") : F("Disabled"));
}


void allocateAddresses(uint16_t baseAddress)
{
  config.aspectAddress1 = baseAddress;

  if (config.signalType == SIGNAL_TYPE_3_4_ASPECT)
  {
    config.aspectAddress2 = baseAddress + 1;
    config.routeAddress = config.routeEnabled ? (baseAddress + 2) : NO_ADDR;
  }
  else
  {
    config.aspectAddress2 = NO_ADDR;
    config.routeAddress = config.routeEnabled ? (baseAddress + 1) : NO_ADDR;
  }
}

void completeLearnMode(uint16_t baseAddress)
{
  // How far past baseAddress the highest address needed will land,
  // given the pending signal type/route - matches allocateAddresses().
  byte highestOffset = (pendingSignalType == SIGNAL_TYPE_3_4_ASPECT ? 1 : 0) +
                        (pendingRouteEnabled ? 1 : 0);
  uint16_t highestAddress = baseAddress + highestOffset;

  if (highestAddress > MAX_DCC_ACCESSORY_ADDRESS)
  {
    Serial.println();
    Serial.print(F("Learn rejected: address "));
    Serial.print(baseAddress);
    Serial.print(F(" would need up to "));
    Serial.print(highestAddress);
    Serial.print(F(", above the max of "));
    Serial.println(MAX_DCC_ACCESSORY_ADDRESS);
    Serial.println(F("Try a lower base address - still waiting in Learn Mode."));
    return; // stay in STATE_LEARN_WAITING_FOR_ADDRESS, nothing is saved
  }

  config.signalType = pendingSignalType;
  config.routeEnabled = pendingRouteEnabled;
  // Aspect Memory is not touched here - it persists independently of
  // relearning a signal's address, only changed via
  // ASPECT_MEMORY_TOGGLE_ADDRESS.

  allocateAddresses(baseAddress);

  config.savedAspect = ASPECT_RED;
  config.savedRoute = ROUTE_OFF;

  indicateEepromSaving();
  saveConfig();
  indicateLearnSuccess();

  decoderState = STATE_NORMAL;

  // Bring outputs into a known state under the freshly learned configuration
  restoringFromEEPROM = true;
  setAspect(ASPECT_RED);
  setRoute(false);
  restoringFromEEPROM = false;

  // If Aspect Link is enabled, don't leave the signal on the Red just
  // set above unless the block genuinely is occupied - resolve a real
  // aspect immediately instead, exactly the same as applyStartupState()
  // already does on a genuine reboot. Without this, relearning a live
  // signal's address on an Aspect-Link-enabled board would leave it
  // stuck on Red until the board next happened to reboot.
  if (config.aspectLinkEnabled)
  {
    establishLiveAspectFromOccupancy();
  }

  printConfigSummary();
}


// ======================================================
// DEFAULT DCC ADDRESS (BENCH TESTING CONVENIENCE)
// ======================================================

void applyDefaultAddress()
{
  // See USE_DEFAULT_DCC_ADDRESS above. Behaves like a real Learn, but
  // uses DEFAULT_DCC_ADDRESS as the base address instead of waiting for
  // a DCC command. Signal type/route/memory come from the same
  // effective*() helpers Learn Mode uses - either the DIP switches, or
  // FORCE_SIGNAL_CONFIG if that's enabled.
  config.signalType = effectiveSignalType();
  config.routeEnabled = effectiveRouteEnabled();
  // Aspect Memory is not touched here - see completeLearnMode().

  allocateAddresses(DEFAULT_DCC_ADDRESS);

  config.savedAspect = ASPECT_RED;
  config.savedRoute = ROUTE_OFF;

  indicateEepromSaving();
  saveConfig();
  indicateLearnSuccess();

  decoderState = STATE_NORMAL;

  restoringFromEEPROM = true;
  setAspect(ASPECT_RED);
  setRoute(false);
  restoringFromEEPROM = false;

  // If Aspect Link is enabled, don't leave the signal on the Red just
  // set above unless the block genuinely is occupied - resolve a real
  // aspect immediately instead, exactly the same as applyStartupState()
  // already does on a genuine reboot. Without this, a board that never
  // actually reboots after this point (e.g. Factory Reset immediately
  // followed by applyDefaultAddress(), with no power cycle in between)
  // would sit on a stale forced Red instead of reflecting whatever the
  // block ahead is actually showing.
  if (config.aspectLinkEnabled)
  {
    establishLiveAspectFromOccupancy();
  }

  printConfigSummary();
}


// ======================================================
// FACTORY RESET
// ======================================================

void factoryReset()
{
  setDefaultConfig();
  EEPROM.put(EEPROM_CONFIG_ADDRESS, config); // validityMarker = 0, so isConfigured() reads false from now on

  // Factory Reset happens live, not via a power cycle, so the Aspect
  // Link runtime state (built up since the last real boot) does NOT
  // reset itself the way it would on a fresh power-up. Clear it
  // explicitly - otherwise a board reset while Aspect Link was active,
  // then quickly re-enabled again without a power cycle, could briefly
  // act on stale leftover state (e.g. an already-active DCC override)
  // until the next genuine occupancy edge sorts it out.
  dccAspectOverrideActive = false;
  clearDebouncePending = false;
  clearDebounceStartedAt = 0;
  debouncedOccupancyState = false;
  updateOccupancyDebounce(); // one real read now, so an already-occupied block is reflected immediately rather than assumed clear
  previousOccupancyState = isOccupied();
  lastCascadeRxAspect = ASPECT_GREEN;
  lastCascadeTxAspect = 0; // 0 never matches a real ASPECT_* value, forces the first send
  lastCascadeTxSentTime = millis();

  indicateFactoryReset();

  decoderState = STATE_NORMAL;

  restoringFromEEPROM = true;
  allAspectsOff();
  setRoute(false);
  restoringFromEEPROM = false;

  Serial.println();
  Serial.println(F("Factory reset complete - all learned addresses cleared."));

  printBuildOptions();

  if (USE_DEFAULT_DCC_ADDRESS)
  {
    applyDefaultAddress();
  }
  else
  {
    printConfigSummary();
  }
}


// ======================================================
// LEARN BUTTON - DEBOUNCE AND LONG PRESS DETECTION
// ======================================================

void onLearnButtonHeld()
{
  if (decoderState == STATE_NORMAL)
  {
    enterLearnMode();
  }
  else if (decoderState == STATE_LEARN_WAITING_FOR_ADDRESS)
  {
    cancelLearnMode();
  }
  else if (decoderState == STATE_LEARN_WAITING_FOR_BRIGHTNESS)
  {
    exitBrightnessAdjustMode();
  }
}

void onFactoryResetHeld()
{
  // A 10 second hold always wins over the 2 second Learn Mode action,
  // which will already have fired earlier in the same button press.
  Serial.println();
  Serial.println(F("Factory reset triggered by long button hold."));
  factoryReset();
}

void updateLearnButton()
{
  int rawReading = digitalRead(PIN_LEARN_BUTTON);

  if (rawReading != lastRawButtonReading)
  {
    lastDebounceTime = millis();
    lastRawButtonReading = rawReading;
  }

  if (millis() - lastDebounceTime < DEBOUNCE_DELAY_MS)
  {
    return; // still settling
  }

  bool pressedNow = (rawReading == BUTTON_PRESSED_STATE);

  if (pressedNow && !debouncedButtonPressed)
  {
    // Button just pressed
    debouncedButtonPressed = true;
    pressStartTime = millis();
    shortHoldActionFired = false;
    longHoldActionFired = false;
  }
  else if (pressedNow && debouncedButtonPressed)
  {
    // Still held - the short (Learn) and long (Factory Reset) hold
    // thresholds are checked independently, so a single continuous
    // hold can fire Learn Mode entry at 2s and then escalate to a
    // Factory Reset at 10s if you just keep holding.
    unsigned long heldFor = millis() - pressStartTime;

    if (!shortHoldActionFired && heldFor >= LEARN_HOLD_MS)
    {
      shortHoldActionFired = true;
      onLearnButtonHeld();
    }

    if (!longHoldActionFired && heldFor >= FACTORY_RESET_HOLD_MS)
    {
      longHoldActionFired = true;
      onFactoryResetHeld();
    }
  }
  else if (!pressedNow && debouncedButtonPressed)
  {
    // Button released
    debouncedButtonPressed = false;
  }
}


// ======================================================
// DCC COMMAND PROCESSING
// ======================================================

void handleAccessoryAddress(uint16_t address, byte direction)
{
  if (decoderState == STATE_LEARN_WAITING_FOR_ADDRESS)
  {
    if (address == BRIGHTNESS_CONFIG_ADDRESS)
    {
      enterBrightnessAdjustMode();
      return;
    }

    if (address == ASPECT_MEMORY_TOGGLE_ADDRESS)
    {
      toggleAspectMemory();
      return;
    }

    if (address == ASPECT_LINK_TOGGLE_ADDRESS)
    {
      toggleAspectLink();
      return;
    }

    if (address == END_OF_LINE_TOGGLE_ADDRESS)
    {
      toggleEndOfLine();
      return;
    }

    completeLearnMode(address);
    return;
  }

  if (decoderState == STATE_LEARN_WAITING_FOR_BRIGHTNESS)
  {
    applyBrightnessValue(address);
    return;
  }

  processDccCommand(address, direction);
}

bool isRelevantAddress(uint16_t address)
{
  // While waiting to learn an address, nothing is "configured" yet, so
  // every address is potentially relevant - that is exactly what you
  // want to see on the bench. Once running normally, only addresses
  // this decoder has actually learned are worth printing; everything
  // else is other decoders' traffic on the same DCC bus.
  if (decoderState == STATE_LEARN_WAITING_FOR_ADDRESS || decoderState == STATE_LEARN_WAITING_FOR_BRIGHTNESS)
  {
    return true;
  }

  if (!isConfigured())
  {
    return false;
  }

  return (address == config.aspectAddress1) ||
         (address == config.aspectAddress2) ||
         (config.routeEnabled && address == config.routeAddress);
}

void processDccCommand(uint16_t address, byte direction)
{
  if (!isConfigured())
  {
    return; // nothing learned yet, ignore all DCC commands
  }

  if (config.routeEnabled && address == config.routeAddress)
  {
    setRoute(direction == DCC_THROWN);
    return;
  }

  if (config.signalType == SIGNAL_TYPE_2_ASPECT)
  {
    if (address != config.aspectAddress1) return;

    dccAspectOverrideActive = true;
    setAspect(direction == DCC_THROWN ? ASPECT_GREEN : ASPECT_RED);
    return;
  }

  // 3/4 Aspect
  if (address == config.aspectAddress1)
  {
    dccAspectOverrideActive = true;
    setAspect(direction == DCC_THROWN ? ASPECT_YELLOW : ASPECT_RED);
    return;
  }

  if (address == config.aspectAddress2)
  {
    dccAspectOverrideActive = true;
    setAspect(direction == DCC_THROWN ? ASPECT_GREEN : ASPECT_DOUBLE_YELLOW);
    return;
  }
}


// ======================================================
// NMRADCC CALLBACKS
// ------------------------------------------------------
// Board-address translation kept identical to the proven
// previous decoder.
// ======================================================

void notifyDccAccTurnoutBoard(uint16_t BoardAddr,
                              uint8_t OutputPair,
                              uint8_t Direction,
                              uint8_t OutputPower)
{
  uint16_t address = (BoardAddr * 4) + OutputPair - 3;

  // Ignore duplicate packets, but only within a short time window -
  // a genuinely new command with the same address/direction must
  // still be processed (e.g. re-sending "thrown" after some other
  // activity), not silently dropped forever.
  unsigned long now = millis();
  bool isDuplicate = (address == lastDccAddress) &&
                      (Direction == lastDccDirection) &&
                      (now - lastDccPacketTime < DUPLICATE_PACKET_WINDOW_MS);

  if (isDuplicate)
  {
    return;
  }

  lastDccAddress = address;
  lastDccDirection = Direction;
  lastDccPacketTime = now;

  if (isRelevantAddress(address))
  {
    Serial.print(F("DCC "));
    Serial.print(address);
    Serial.println(Direction == DCC_THROWN ? F(" THROWN") : F(" CLOSED"));
  }

  handleAccessoryAddress(address, Direction);
}

void notifyDccAccTurnoutOutput(uint16_t Addr,
                               uint8_t Direction,
                               uint8_t OutputPower)
{
  // Debug stub only, kept from the proven previous decoder.
  Serial.print(F("OUTPUT "));
  Serial.print(Addr);
  Serial.print(F(" "));
  Serial.println(Direction);
}

void notifyDccMsg(DCC_MSG *Msg)
{
  // Fires on EVERY valid DCC packet, not just accessory commands aimed
  // at this decoder's addresses. Used only to know "is DCC still
  // present on the track", for the no-signal status LED warning.
  lastDccActivityTime = millis();
}


// ======================================================
// DEBUG / BENCH TEST HELPERS
// ======================================================

void printBuildOptions()
{
  // Surfaces the compile-time toggles up top, so it's obvious from
  // Serial alone what this particular upload is set to do, without
  // needing to dig through the source.
  Serial.println();
  Serial.println(F("Build Options"));
  Serial.println(F("-------------"));

  Serial.print(F("Default DCC Address                  : "));
  if (USE_DEFAULT_DCC_ADDRESS)
  {
    Serial.println(DEFAULT_DCC_ADDRESS);
  }
  else
  {
    Serial.println(F("Disabled"));
  }

  Serial.print(F("Forced Signal Config                 : "));
  if (FORCE_SIGNAL_CONFIG)
  {
    Serial.print(FORCE_SIGNAL_TYPE == SIGNAL_TYPE_2_ASPECT ? F("2 Aspect") : F("3/4 Aspect"));
    Serial.print(F(", Route "));
    Serial.println(FORCE_ROUTE_ENABLED ? F("Enabled") : F("Disabled"));
  }
  else
  {
    Serial.println(F("Disabled (using DIP switches)"));
  }

  Serial.print(F("Default Aspect Memory                : "));
  Serial.println(DEFAULT_ASPECT_MEMORY_ENABLED ? F("Enabled") : F("Disabled"));

  Serial.print(F("Default Aspect Link                  : "));
  Serial.println(DEFAULT_ASPECT_LINK_ENABLED ? F("Enabled") : F("Disabled"));

  Serial.print(F("Default End Of Line                  : "));
  Serial.println(DEFAULT_END_OF_LINE_ENABLED ? F("Enabled") : F("Disabled"));

  Serial.print(F("Double Yellow also lights Yellow     : "));
  Serial.println(DOUBLE_YELLOW_ALSO_LIGHTS_YELLOW ? F("Enabled") : F("Disabled"));

  Serial.print(F("Watchdog Timeout                     : "));
  Serial.println(watchdogTimeoutName(WATCHDOG_TIMEOUT_SETTING));

  Serial.print(F("Brightness baselines (R/Y/DY/G/Route): "));
  Serial.print(BRIGHTNESS_BASELINE_RED);
  Serial.print(F("/"));
  Serial.print(BRIGHTNESS_BASELINE_YELLOW);
  Serial.print(F("/"));
  Serial.print(BRIGHTNESS_BASELINE_DOUBLE_YELLOW);
  Serial.print(F("/"));
  Serial.print(BRIGHTNESS_BASELINE_GREEN);
  Serial.print(F("/"));
  Serial.print(BRIGHTNESS_BASELINE_ROUTE);
  Serial.println(F("%"));

  Serial.print(F("Default global brightness            : "));
  Serial.print(DEFAULT_GLOBAL_BRIGHTNESS_PERCENT);
  Serial.println(F("%"));

  Serial.println();
}

void printConfigSummary()
{
  Serial.println();
  Serial.println(F("Signal Configuration"));
  Serial.println(F("---------------------"));

  if (!isConfigured())
  {
    Serial.println(F("NOT LEARNED - hold the Learn button for 2s to begin."));
    Serial.println();
    return;
  }

  Serial.print(F("Signal Type   : "));
  Serial.println(config.signalType == SIGNAL_TYPE_2_ASPECT ? F("2 Aspect") : F("3/4 Aspect"));

  Serial.print(F("Aspect Addr 1 : "));
  Serial.println(config.aspectAddress1);

  if (config.signalType == SIGNAL_TYPE_3_4_ASPECT)
  {
    Serial.print(F("Aspect Addr 2 : "));
    Serial.println(config.aspectAddress2);
  }

  if (config.routeEnabled)
  {
    Serial.print(F("Route Addr    : "));
    Serial.println(config.routeAddress);
  }
  else
  {
    Serial.println(F("Route         : Disabled"));
  }

  Serial.print(F("Aspect Memory : "));
  Serial.println(config.aspectMemoryEnabled ? F("Enabled") : F("Disabled"));

  Serial.print(F("Aspect Link   : "));
  Serial.println(config.aspectLinkEnabled ? F("Enabled") : F("Disabled"));

  if (config.aspectLinkEnabled)
  {
    Serial.print(F("End Of Line   : "));
    Serial.println(config.endOfLineEnabled ? F("Enabled") : F("Disabled"));

    Serial.print(F("Block Occupied: "));
    Serial.println(isOccupied() ? F("Yes") : F("No"));

    if (clearDebouncePending)
    {
      unsigned long remainingMs = OCCUPANCY_CLEAR_DEBOUNCE_MS - (millis() - clearDebounceStartedAt);
      Serial.print(F("Clear Debounce: pending, "));
      Serial.print(remainingMs);
      Serial.println(F("ms remaining"));
    }

    Serial.print(F("DCC Override  : "));
    Serial.println(dccAspectOverrideActive ? F("Active (blocks cascade re-derive until occupancy next clears)") : F("Not active"));

    if (!config.endOfLineEnabled)
    {
      Serial.print(F("Cascade RX Ast: "));
      Serial.println(aspectName(lastCascadeRxAspect));
    }

    Serial.print(F("Cascade TX Ast: "));
    Serial.println(aspectName(lastCascadeTxAspect));
  }

  Serial.print(F("Current Aspect: "));
  Serial.println(aspectName(config.savedAspect));

  Serial.print(F("Current Route : "));
  Serial.println(config.savedRoute == ROUTE_ON ? F("ON") : F("OFF"));

  Serial.print(F("Brightness    : "));
  Serial.print(config.globalBrightness);
  Serial.println(F("%"));

  printTestCommands();

  Serial.println();
}

void printTestCommands()
{
  // Bench-testing convenience only - shows the exact `dcc` commands
  // to type for each aspect/route state, using whatever addresses
  // have actually been learned. Purely a Serial Monitor aid; has no
  // effect on real DCC decoding.
  Serial.println();
  Serial.println(F("Test Commands (Serial Monitor)"));
  Serial.println(F("-------------------------------"));

  Serial.print(F("Red aspect    : dcc "));
  Serial.print(config.aspectAddress1);
  Serial.println(F(" closed"));

  if (config.signalType == SIGNAL_TYPE_2_ASPECT)
  {
    Serial.print(F("Green aspect  : dcc "));
    Serial.print(config.aspectAddress1);
    Serial.println(F(" thrown"));
  }
  else
  {
    Serial.print(F("Yellow aspect : dcc "));
    Serial.print(config.aspectAddress1);
    Serial.println(F(" thrown"));

    Serial.print(F("Double Yellow : dcc "));
    Serial.print(config.aspectAddress2);
    Serial.println(F(" closed"));

    Serial.print(F("Green aspect  : dcc "));
    Serial.print(config.aspectAddress2);
    Serial.println(F(" thrown"));
  }

  if (config.routeEnabled)
  {
    Serial.print(F("Route ON      : dcc "));
    Serial.print(config.routeAddress);
    Serial.println(F(" thrown"));

    Serial.print(F("Route OFF     : dcc "));
    Serial.print(config.routeAddress);
    Serial.println(F(" closed"));
  }
}

void handleSerialTestCommand()
{
  String cmd = Serial.readStringUntil('\n');
  cmd.trim();

  if (cmd.length() == 0)
  {
    return;
  }

  if (cmd == "status")
  {
    printConfigSummary();
    return;
  }

  if (cmd == "build")
  {
    printBuildOptions();
    return;
  }

  if (cmd == "help")
  {
    Serial.println();
    Serial.println(F("Bench Test Commands"));
    Serial.println(F("--------------------"));
    Serial.println(F("status  - print current configuration and state"));
    Serial.println(F("build   - print the compile-time build options banner"));
    Serial.println(F("learn   - simulate a 2 second Learn button hold, type again to cancel"));
    Serial.println(F("reset   - simulate a 10 second Learn button hold (factory reset)"));
    Serial.println(F("dcc <addr> <closed|thrown> - simulate a DCC accessory command"));
    Serial.println(F("occ <on|off|auto> - simulate the occupancy input (auto = use the real pin)"));
    Serial.println(F("cascade <red|yellow|doubleyellow|green> - simulate a received cascade byte"));
    Serial.println();
    return;
  }

  if (cmd == "learn")
  {
    // Simulates a 2 second Learn button hold, useful before the
    // physical button is wired up on the breadboard.
    onLearnButtonHeld();
    return;
  }

  if (cmd == "reset")
  {
    // Simulates a 10 second Learn button hold (Factory Reset), useful
    // for testing the reset path without holding the physical button.
    onFactoryResetHeld();
    return;
  }

  if (cmd.startsWith("dcc "))
  {
    int firstSpace = cmd.indexOf(' ');
    int secondSpace = cmd.indexOf(' ', firstSpace + 1);

    if (secondSpace == -1)
    {
      Serial.println(F("Usage: dcc <address> <closed|thrown>"));
      return;
    }

    uint16_t address = cmd.substring(firstSpace + 1, secondSpace).toInt();
    String stateText = cmd.substring(secondSpace + 1);

    byte direction = (stateText == "thrown") ? DCC_THROWN : DCC_CLOSED;

    Serial.print(F("[TEST] DCC "));
    Serial.print(address);
    Serial.println(direction == DCC_THROWN ? F(" THROWN") : F(" CLOSED"));

    handleAccessoryAddress(address, direction);
    return;
  }

  if (cmd.startsWith("occ "))
  {
    String stateText = cmd.substring(4);

    if (stateText == "on")
    {
      serialOccupancySimActive = true;
      serialOccupancySimState = true;
      Serial.println(F("[TEST] Occupancy input simulated: OCCUPIED"));
    }
    else if (stateText == "off")
    {
      serialOccupancySimActive = true;
      serialOccupancySimState = false;
      Serial.println(F("[TEST] Occupancy input simulated: CLEAR"));
    }
    else if (stateText == "auto")
    {
      serialOccupancySimActive = false;
      Serial.println(F("[TEST] Occupancy input: back to reading the real pin."));
    }
    else
    {
      Serial.println(F("Usage: occ <on|off|auto>"));
    }
    return;
  }

  if (cmd.startsWith("cascade "))
  {
    if (!config.aspectLinkEnabled)
    {
      Serial.println(F("[TEST] Aspect Link is disabled - a real board would never read this byte at all. Enable it first (learn / dcc 888 thrown / learn)."));
      return;
    }

    String aspectText = cmd.substring(8);
    byte simulatedAspect;

    if (aspectText == "red")               simulatedAspect = ASPECT_RED;
    else if (aspectText == "yellow")       simulatedAspect = ASPECT_YELLOW;
    else if (aspectText == "doubleyellow") simulatedAspect = ASPECT_DOUBLE_YELLOW;
    else if (aspectText == "green")        simulatedAspect = ASPECT_GREEN;
    else
    {
      Serial.println(F("Usage: cascade <red|yellow|doubleyellow|green>"));
      return;
    }

    lastCascadeRxAspect = simulatedAspect;
    Serial.print(F("[TEST] Cascade RX simulated as: "));
    Serial.println(aspectName(simulatedAspect));

    // Mirror what a genuinely new byte on the real link now does (see
    // updateCascadeRx()) - re-derive immediately, subject to the same
    // guards: own occupancy always wins locally, and an active DCC
    // override is only ever discarded by an occupancy-clear edge.
    if (!isOccupied() && !dccAspectOverrideActive)
    {
      resolveAspectFromCascade();
    }
    else if (isOccupied())
    {
      Serial.println(F("[TEST] (not applied - this board's own occupancy is active)"));
    }
    else
    {
      Serial.println(F("[TEST] (not applied - a DCC override is currently active)"));
    }

    return;
  }

  Serial.println(F("Unknown command. Type 'help' for the list of commands."));
}


// ======================================================
// SETUP / LOOP
// ======================================================

void setup()
{
  // Must be the very first thing done, before anything else - clears
  // any watchdog state left armed by the bootloader after a previous
  // watchdog-triggered reset, so it can't fire again before setup() has
  // even finished. Re-armed deliberately at the end of setup(), once
  // everything is actually initialised - see WATCHDOG_TIMEOUT_SETTING.
  wdt_disable();

  Serial.begin(115200);

  pinMode(PIN_RED, OUTPUT);
  pinMode(PIN_YELLOW, OUTPUT);
  pinMode(PIN_DOUBLE_YELLOW, OUTPUT);
  pinMode(PIN_GREEN, OUTPUT);
  pinMode(PIN_ROUTE, OUTPUT);
  pinMode(PIN_STATUS_LED, OUTPUT);

  pinMode(PIN_LEARN_BUTTON, INPUT_PULLUP);
  pinMode(PIN_DIP_1, INPUT_PULLUP);
  pinMode(PIN_DIP_2, INPUT_PULLUP);
  pinMode(PIN_OCCUPANCY_INPUT, INPUT_PULLUP);

  cascadeSerial.begin(CASCADE_SERIAL_BAUD);

  allAspectsOff();
  digitalWrite(PIN_ROUTE, LOW);
  digitalWrite(PIN_STATUS_LED, LOW);

  loadConfig();
  applyStartupState();

  // Interrupt 0 = pin D2 on the Nano/Uno. If PIN_DCC_INPUT is changed
  // to D3, this first argument must change to 1 (interrupt 1).
  Dcc.pin(0, PIN_DCC_INPUT, 1);

  Dcc.init(MAN_ID_DIY, 10, FLAGS_DCC_ACCESSORY_DECODER, 0);

  Serial.println(F("DCC Signal Decoder - v1.4"));
  printBuildOptions();
  printConfigSummary();
  Serial.println(F("Commands: status | build | learn | reset | dcc <addr> <closed|thrown> | occ <on|off|auto> | cascade <aspect> | help"));

  wdt_enable(WATCHDOG_TIMEOUT_SETTING); // armed last, only once setup() has genuinely finished
}

void loop()
{
  wdt_reset(); // "feed the dog" - must happen every pass, comfortably more often than WATCHDOG_TIMEOUT_SETTING

  Dcc.process();

  updateLearnButton();
  updateStatusLed();
  checkLearnModeTimeout();
  checkBrightnessModeTimeout();
  updateOccupancyCascade();

  if (Serial.available())
  {
    handleSerialTestCommand();
  }
}
