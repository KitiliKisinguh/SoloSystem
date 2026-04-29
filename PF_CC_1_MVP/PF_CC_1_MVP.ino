// =====================================
// PF_CC_1_MVP
// Interlocked plasma chamber MVP controller (Uno)
//
// Key policies from agreed spec:
// - Inputs create requests; state owns outputs
// - E-stop safety is hardwired (independent of Uno monitoring)
// - Lock is power-to-unlock (fail-locked)
// - Unlock only in SAFE_RELEASE
// - Once run starts (or stop occurs in STARTING), destruct is required before release
// - No auto-restart and no auto-unlock after reboot/brownout
// =====================================

#include <EEPROM.h>

// ---------------- Pin map (adjust to your harness) ----------------
// Outputs (relay board style, active-low assumed by RELAY_ON/RELAY_OFF)
const byte OUT_PLASMA_REQ     = 2;
const byte OUT_PUMP           = 3;
const byte OUT_VALVE_A        = 4;
const byte OUT_VALVE_DESTRUCT = 5;
const byte OUT_LOCK_RELEASE   = 6;  // power-to-unlock
const byte OUT_LAMP_GREEN     = 8;
const byte OUT_LAMP_RED       = 7;
const byte OUT_LAMP_BLUE      = 10;
const int SOLO_OUT_PIN        = A2;// Digital Output Solo System

// Inputs
const byte IN_START_PB     = 11;   // active-low
const byte IN_STOP_PB      = 12;   // NC with INPUT_PULLUP: healthy=LOW, pressed/open=HIGH
const byte IN_MODE_6H_SEL  = 9;    // NO selector, INPUT_PULLUP, active-low
const byte IN_ESTOP_OK     = A0;   // optional monitor input (unused when USE_IN_ESTOP_OK=false)
const byte IN_DOOR_CLOSED  = A1;   // active-low with pullup by default
const byte IN_LOCK_ENGAGED = 13;   // interlock status contact (truth validated below)
const byte IN_FLOW_OK      = A3;   // optional active-low with pullup by default
const byte IN_SAFETY_OK    = A4;   // optional active-low with pullup by default
const byte IN_PUMP_FB      = A5;   // optional active-low with pullup by default

// ---------------- Optional permissives (MVP toggles) ----------------
const bool USE_IN_DOOR_CLOSED  = false;  // current validated input set uses START/STOP/MODE only
const bool USE_IN_LOCK_ENGAGED = true;
const bool USE_IN_SAFETY_OK    = false;
const bool USE_IN_FLOW_OK      = false;   // if true, destruct completion requires flow proof
const bool USE_IN_PUMP_FB      = false;
const bool USE_IN_ESTOP_OK     = false;   // hardwired-only E-stop mode

// ---------------- Electrical logic config ----------------
const byte RELAY_ON  = LOW;
const byte RELAY_OFF = HIGH;
// Output inversion flags aligned to validated hardware behavior.
// true  => requestOn is inverted before driving RELAY_ON/RELAY_OFF
// false => requestOn drives relay directly
const bool PLASMA_OUTPUT_INVERTED       = true;
const bool PUMP_OUTPUT_INVERTED         = true;
const bool VALVE_A_OUTPUT_INVERTED      = true;
const bool VALVE_DESTRUCT_OUTPUT_INVERTED = true;
const bool LOCK_RELEASE_OUTPUT_INVERTED = true;
const bool LAMP_GREEN_OUTPUT_INVERTED   = true;
const bool LAMP_RED_OUTPUT_INVERTED     = true;
const bool LAMP_BLUE_OUTPUT_INVERTED    = true;

const bool DOOR_CLOSED_ACTIVE_LOW  = true;
const bool LOCK_ENGAGED_ACTIVE_LOW = false; // corrected: HIGH means lock engaged
const bool FLOW_OK_ACTIVE_LOW      = true;
const bool SAFETY_OK_ACTIVE_LOW    = true;
const bool PUMP_FB_ACTIVE_LOW      = true;
const bool START_ACTIVE_LOW        = true;  // START validated as NO + INPUT_PULLUP (press => LOW)
const bool STOP_ACTIVE_LOW         = false; // NC + pullup => pressed is HIGH

// E-stop status convention (if enabled): HIGH = healthy, LOW = active/unsafe
const bool ESTOP_OK_ACTIVE_HIGH = true;

// ---------------- Timers ----------------
const unsigned long T_INPUT_DEBOUNCE_MS   = 50;
const unsigned long T_DOOR_DEBOUNCE_MS    = 50;
const unsigned long T_LOCK_DEBOUNCE_MS    = 50;
const unsigned long T_LOCK_SENSE_SETTLE_MS = 500;
const unsigned long T_LOCK_TIMEOUT_MS     = 10000;
const unsigned long T_PRESTART_AIRFLOW_MS = 3000;
const unsigned long T_POST_PLASMA_DELAY_MS = 200;
const unsigned long T_DESTRUCT_DURATION_MS = 900000; // 15 min locked value (includes safety factor)
const unsigned long T_UNLOCK_WINDOW_MS     = 15000;
const unsigned long T_LOOP_POLL_MS         = 5;
const unsigned long T_RESET_HOLD_MS        = 2000; // STOP long-press for reset in FAULT/ESTOP
const unsigned long T_STATUS_BLINK_MS = 1000; // 1s on / 1s off
const unsigned long T_DEMO_ABORT_HOLD_MS   = 5000; // START+STOP hold for demo safe stop/unlock

// Process profile (from tuned cycle)
const unsigned long RAMP_ON_MS      = 108000;
const unsigned long RAMP_COAST_MS   = 30000;
const bool          USE_TRIM_PULSE  = false;
const unsigned long TRIM_ON_MS      = 2000;
const unsigned long TRIM_COAST_MS   = 30000;
const unsigned long HOLD_TOTAL_2H_MS = 7200000;   // 2 hours
const unsigned long HOLD_TOTAL_6H_MS = 21600000;  // 6 hours
const unsigned long HOLD_ON_MS      = 5000;
const unsigned long HOLD_OFF_MS     = 192000;

// ---------------- Persistent flags ----------------
const int EEPROM_ADDR_MAGIC = 0;
const int EEPROM_ADDR_FLAGS = 1;
const byte EEPROM_MAGIC = 0xA5;
bool Solo_System_State= 0;
// bit0 cycle_interrupted, bit1 destruct_required

// ---------------- FSM ----------------
enum State {
  ST_BOOT = 0,
  ST_BOOT_RECOVER,
  ST_SAFE_OPEN,
  ST_LOCKING,
  ST_READY_LOCKED,
  ST_STARTING,
  ST_RUNNING,
  ST_STOPPING,
  ST_DESTRUCT,
  ST_SAFE_RELEASE,
  ST_FAULT,
  ST_ESTOP
};

State state = ST_BOOT;
unsigned long stateEnteredMs = 0;

// Internal latches/flags
bool fault_latched = false;
bool estop_latched = false;
bool destruct_required = false;
bool cycle_interrupted = false;

// Requests (one-shot edges)
bool req_start = false;
bool req_stop  = false;

// Debounce storage
bool lastRawStart = HIGH, stableStart = HIGH;
bool lastRawStop  = HIGH, stableStop  = HIGH;
unsigned long lastStartChangeMs = 0;
unsigned long lastStopChangeMs  = 0;
bool lastRawMode6h = false;
bool stableMode6h  = false;
unsigned long lastModeChangeMs = 0;

bool lastDoorRaw = false;
bool door_stable_closed = false;
unsigned long lastDoorChangeMs = 0;
bool lastReportedDoorStable = false;
bool lastLockRaw = false;
bool lock_stable_engaged = false;
unsigned long lastLockChangeMs = 0;
unsigned long lockSenseMaskUntilMs = 0;

// Running profile substate
enum RunPhase {
  RP_RAMP_ON = 0,
  RP_RAMP_COAST,
  RP_TRIM_ON,
  RP_TRIM_COAST,
  RP_HOLD_ON,
  RP_HOLD_OFF,
  RP_COMPLETE
};
RunPhase runPhase = RP_RAMP_ON;
unsigned long runPhaseStartMs = 0;
unsigned long holdPhaseStartMs = 0;
unsigned long holdTargetMsActive = HOLD_TOTAL_2H_MS;

// Destruct tracking
unsigned long destructGoodMs = 0;
unsigned long destructLastTickMs = 0;

const __FlashStringHelper* stateName(State s) {
  switch (s) {
    case ST_BOOT:         return F("BOOT");
    case ST_BOOT_RECOVER: return F("BOOT_RECOVER");
    case ST_SAFE_OPEN:    return F("SAFE_OPEN");
    case ST_LOCKING:      return F("LOCKING");
    case ST_READY_LOCKED: return F("READY_LOCKED");
    case ST_STARTING:     return F("STARTING");
    case ST_RUNNING:      return F("RUNNING");
    case ST_STOPPING:     return F("STOPPING");
    case ST_DESTRUCT:     return F("DESTRUCT");
    case ST_SAFE_RELEASE: return F("SAFE_RELEASE");
    case ST_FAULT:        return F("FAULT");
    case ST_ESTOP:        return F("ESTOP");
    default:              return F("UNKNOWN");
  }
}

// ---------------- Helpers ----------------
void persistFlags() {
  if (EEPROM.read(EEPROM_ADDR_MAGIC) != EEPROM_MAGIC) {
    EEPROM.write(EEPROM_ADDR_MAGIC, EEPROM_MAGIC);
  }
  byte flags = 0;
  if (cycle_interrupted) flags |= 0x01;
  if (destruct_required) flags |= 0x02;
  EEPROM.write(EEPROM_ADDR_FLAGS, flags);
}

void restoreFlags() {
  if (EEPROM.read(EEPROM_ADDR_MAGIC) == EEPROM_MAGIC) {
    byte flags = EEPROM.read(EEPROM_ADDR_FLAGS);
    cycle_interrupted = (flags & 0x01) != 0;
    destruct_required = (flags & 0x02) != 0;
  } else {
    cycle_interrupted = false;
    destruct_required = false;
    persistFlags();
  }
}

bool readActiveLow(byte pin, bool activeLow) {
  bool raw = digitalRead(pin);
  return activeLow ? (raw == LOW) : (raw == HIGH);
}

bool estop_ok() {
  if (!USE_IN_ESTOP_OK) return true;
  bool raw = digitalRead(IN_ESTOP_OK);
  return ESTOP_OK_ACTIVE_HIGH ? (raw == HIGH) : (raw == LOW);
}

bool door_closed_raw() {
  if (!USE_IN_DOOR_CLOSED) return true;
  return readActiveLow(IN_DOOR_CLOSED, DOOR_CLOSED_ACTIVE_LOW);
}

bool lock_engaged_raw() {
  if (!USE_IN_LOCK_ENGAGED) return true;
  return readActiveLow(IN_LOCK_ENGAGED, LOCK_ENGAGED_ACTIVE_LOW);
}

bool unlockCommandRequestedInState(State s) {
  return (s == ST_SAFE_OPEN || s == ST_SAFE_RELEASE);
}

bool lock_engaged() {
  if (!USE_IN_LOCK_ENGAGED) return true;
  if (unlockCommandRequestedInState(state)) return true;
  if (millis() < lockSenseMaskUntilMs) return true;
  return lock_stable_engaged;
}

bool safety_ok() {
  if (!USE_IN_SAFETY_OK) return true;
  return readActiveLow(IN_SAFETY_OK, SAFETY_OK_ACTIVE_LOW);
}

bool flow_ok() {
  if (!USE_IN_FLOW_OK) return true;
  return readActiveLow(IN_FLOW_OK, FLOW_OK_ACTIVE_LOW);
}

bool pump_fb_ok() {
  if (!USE_IN_PUMP_FB) return true;
  return readActiveLow(IN_PUMP_FB, PUMP_FB_ACTIVE_LOW);
}

bool permissives_for_run() {
  return door_stable_closed && lock_engaged() && safety_ok() && estop_ok();
}

bool mode6hSelected() {
  return stableMode6h;
}

bool stopPressedRaw(bool rawStop) {
  return STOP_ACTIVE_LOW ? (rawStop == LOW) : (rawStop == HIGH);
}

bool stopLongPressResetEvent() {
  static bool wasPressed = false;
  static unsigned long pressedStart = 0;
  static bool fired = false;
  bool rawStop = digitalRead(IN_STOP_PB);
  bool pressed = stopPressedRaw(rawStop);

  if (pressed && !wasPressed) {
    pressedStart = millis();
    fired = false;
  }

  if (!pressed) {
    wasPressed = false;
    pressedStart = 0;
    fired = false;
    return false;
  }

  wasPressed = true;
  if (!fired && (millis() - pressedStart >= T_RESET_HOLD_MS)) {
      fired = true;
      return true;
  }
  return false;
}

bool demoAbortHoldEvent() {
  static bool bothWasPressed = false;
  static unsigned long bothPressedStart = 0;
  static bool fired = false;

  bool rawStart = digitalRead(IN_START_PB);
  bool rawStop = digitalRead(IN_STOP_PB);
  bool startPressed = START_ACTIVE_LOW ? (rawStart == LOW) : (rawStart == HIGH);
  bool stopPressed = stopPressedRaw(rawStop);
  bool bothPressed = startPressed && stopPressed;

  if (bothPressed && !bothWasPressed) {
    bothPressedStart = millis();
    fired = false;
  }

  if (!bothPressed) {
    bothWasPressed = false;
    bothPressedStart = 0;
    fired = false;
    return false;
  }

  bothWasPressed = true;
  if (!fired && (millis() - bothPressedStart >= T_DEMO_ABORT_HOLD_MS)) {
    fired = true;
    return true;
  }
  return false;
}

void writeOutput(byte pin, bool on) {
  bool effectiveOn = on;

  if (pin == OUT_PUMP && PUMP_OUTPUT_INVERTED) effectiveOn = !on;
  if (pin == OUT_VALVE_A && VALVE_A_OUTPUT_INVERTED) effectiveOn = !on;
  if (pin == OUT_VALVE_DESTRUCT && VALVE_DESTRUCT_OUTPUT_INVERTED) effectiveOn = !on;
  if (pin == OUT_LOCK_RELEASE && LOCK_RELEASE_OUTPUT_INVERTED) effectiveOn = !on;
  if (pin == OUT_LAMP_GREEN && LAMP_GREEN_OUTPUT_INVERTED) effectiveOn = !on;
  if (pin == OUT_LAMP_RED && LAMP_RED_OUTPUT_INVERTED) effectiveOn = !on;
  if (pin == OUT_LAMP_BLUE && LAMP_BLUE_OUTPUT_INVERTED) effectiveOn = !on;

  digitalWrite(pin, effectiveOn ? RELAY_ON : RELAY_OFF);
}

void writePlasmaOutput(bool plasmaRequestOn) {
  bool effectiveOn = PLASMA_OUTPUT_INVERTED ? !plasmaRequestOn : plasmaRequestOn;
  digitalWrite(OUT_PLASMA_REQ, effectiveOn ? RELAY_ON : RELAY_OFF);
}

void applyOutputsForState() {
  // Defaults (safe)
  bool out_lock_release = false; // false => locked
  bool out_plasma = false;
  bool out_pump = false;
  bool out_valve_a = false;
  bool out_valve_destruct = false;
  bool lamp_red = false;
  bool lamp_blue = false;
  bool lamp_green = false;
  bool Solo_System_State = false; 

  switch (state) {
    case ST_SAFE_OPEN:
      out_lock_release = true; // unlock allowed only here / safe release
      lamp_green = true;
      break;

    case ST_LOCKING:
      lamp_blue = true;
      break;

    case ST_READY_LOCKED:
      lamp_blue = true;
      break;

    case ST_STARTING:
      out_valve_a = true;
      lamp_blue = true;
      Solo_System_State= true;
      break;

    case ST_RUNNING:
      // Base running outputs
      out_valve_a = true;
      lamp_blue = true; // solid blue when running
      // Plasma based on run phase
      if (runPhase == RP_RAMP_ON || runPhase == RP_TRIM_ON || runPhase == RP_HOLD_ON) {
        out_plasma = true;
        lamp_green = true; // green + blue w  hen plasma is ON
      }
      break;

    case ST_STOPPING:
      lamp_blue = true;
      Solo_System_State=false;
      break;

    case ST_DESTRUCT:
      out_pump = true;
      out_valve_destruct = true;
      lamp_blue = ((millis() / T_STATUS_BLINK_MS) % 2) == 0; // blink during destruct
      break;

    case ST_SAFE_RELEASE:
      out_lock_release = true;
      lamp_green = true;
      break;

    case ST_FAULT:
      lamp_red = true;
      break;

    case ST_ESTOP:
      lamp_red = true;
      Solo_System_State = false;
      break;

    case ST_BOOT:
    case ST_BOOT_RECOVER:
    default:
      lamp_red = true;
      break;
  }

  // Hard override: if estop unhealthy, ensure hazardous requests are off
  if (!estop_ok()) {
    out_plasma = false;
    out_pump = false;
    out_valve_a = false;
    out_valve_destruct = false;
    out_lock_release = false;
    lamp_green = false;
    lamp_blue = false;
    lamp_red = true;
  }

  writeOutput(OUT_LOCK_RELEASE, out_lock_release);
  writePlasmaOutput(out_plasma);
  writeOutput(OUT_PUMP, out_pump);
  writeOutput(OUT_VALVE_A, out_valve_a);
  writeOutput(OUT_VALVE_DESTRUCT, out_valve_destruct);
  writeOutput(OUT_LAMP_RED, lamp_red);
  writeOutput(OUT_LAMP_BLUE, lamp_blue);
  writeOutput(OUT_LAMP_GREEN, lamp_green);
  SoloSystemWriteEnable();
}

void enterState(State next) {
  State prev = state;
  state = next;
  stateEnteredMs = millis();

  // Ignore interlock status briefly after unlock coil is dropped.
  if (unlockCommandRequestedInState(prev) && !unlockCommandRequestedInState(next)) {
    lockSenseMaskUntilMs = millis() + T_LOCK_SENSE_SETTLE_MS;
  }

  Serial.print(F("["));
  Serial.print(stateEnteredMs);
  Serial.print(F(" ms] STATE: "));
  Serial.print(stateName(prev));
  Serial.print(F(" -> "));
  Serial.println(stateName(next));

  if (state == ST_RUNNING) {
    runPhase = RP_RAMP_ON;
    runPhaseStartMs = millis();
    holdPhaseStartMs = 0;
    holdTargetMsActive = mode6hSelected() ? HOLD_TOTAL_6H_MS : HOLD_TOTAL_2H_MS;
    Serial.print(F("RUN MODE: "));
    Serial.println(mode6hSelected() ? F("6H selected") : F("2H default"));
    destruct_required = true;  // once entering run path, system owes destruct
    cycle_interrupted = true;  // clear only after safe release
    persistFlags();
  }

  if (state == ST_DESTRUCT) {
    destructLastTickMs = millis();
    destructGoodMs = 0;
  }

  if (state == ST_SAFE_RELEASE) {
    // Destruct completed, clear owed flags
    destruct_required = false;
    cycle_interrupted = false;
    fault_latched = false;
    estop_latched = false;
    persistFlags();
  }
}

void updateDebouncedInputs() {
  // Start
  bool rawStart = digitalRead(IN_START_PB);
  if (rawStart != lastRawStart) {
    lastRawStart = rawStart;
    lastStartChangeMs = millis();
  }
  if (millis() - lastStartChangeMs >= T_INPUT_DEBOUNCE_MS && rawStart != stableStart) {
    stableStart = rawStart;
    bool startPressed = START_ACTIVE_LOW ? (stableStart == LOW) : (stableStart == HIGH);
    if (startPressed) req_start = true;
  }

  // Stop
  bool rawStop = digitalRead(IN_STOP_PB);
  if (rawStop != lastRawStop) {
    lastRawStop = rawStop;
    lastStopChangeMs = millis();
  }
  if (millis() - lastStopChangeMs >= T_INPUT_DEBOUNCE_MS && rawStop != stableStop) {
    stableStop = rawStop;
    if (stopPressedRaw(stableStop)) req_stop = true;
  }

  // Mode select (NO switch with INPUT_PULLUP: LOW => 6h mode)
  bool rawMode6h = (digitalRead(IN_MODE_6H_SEL) == LOW);
  if (rawMode6h != lastRawMode6h) {
    lastRawMode6h = rawMode6h;
    lastModeChangeMs = millis();
  }
  if (millis() - lastModeChangeMs >= T_INPUT_DEBOUNCE_MS && rawMode6h != stableMode6h) {
    stableMode6h = rawMode6h;
    Serial.print(F("MODE SELECTOR -> "));
    Serial.println(stableMode6h ? F("6H") : F("2H"));
  }

  // Door stable closed
  bool rawDoor = door_closed_raw();
  if (rawDoor != lastDoorRaw) {
    lastDoorRaw = rawDoor;
    lastDoorChangeMs = millis();
  }
  if (millis() - lastDoorChangeMs >= T_DOOR_DEBOUNCE_MS) {
    door_stable_closed = rawDoor;
  }

  if (door_stable_closed != lastReportedDoorStable) {
    lastReportedDoorStable = door_stable_closed;
    Serial.print(F("DOOR SWITCH -> "));
    Serial.print(door_stable_closed ? F("CLOSED") : F("OPEN"));
    Serial.print(F(" | STATE="));
    Serial.print(stateName(state));
    Serial.print(F(" | "));
    Serial.println(unlockCommandRequestedInState(state)
      ? F("unlock permitted")
      : F("unlock not permitted"));
  }

  // Lock stable engaged
  bool rawLock = lock_engaged_raw();
  if (rawLock != lastLockRaw) {
    lastLockRaw = rawLock;
    lastLockChangeMs = millis();
  }
  if (millis() - lastLockChangeMs >= T_LOCK_DEBOUNCE_MS) {
    lock_stable_engaged = rawLock;
  }
}

void handleGlobalSafety() {
  if (!estop_ok()) {
    estop_latched = true;
    fault_latched = true;
    enterState(ST_ESTOP);
    return;
  }

  // If permissives are lost during active sequence, fault.
  // Destruct is only latched as required once we are in RUNNING.
  if ((state == ST_STARTING || state == ST_RUNNING) && !permissives_for_run()) {
    fault_latched = true;
    if (state == ST_RUNNING) {
      // Plasma-capable state reached: owe destruct before release.
      destruct_required = true;
      persistFlags();
    } else {
      // STARTING fault before RUNNING: do not force full destruct on reset.
      Serial.println(F("STARTING fault before RUNNING: destruct not latched."));
    }
    enterState(ST_FAULT);
    return;
  }
}

void updateRunningPhase() {
  unsigned long now = millis();
  unsigned long phaseElapsed = now - runPhaseStartMs;

  switch (runPhase) {
    case RP_RAMP_ON:
      if (phaseElapsed >= RAMP_ON_MS) {
        runPhase = RP_RAMP_COAST;
        runPhaseStartMs = now;
      }
      break;

    case RP_RAMP_COAST:
      if (phaseElapsed >= RAMP_COAST_MS) {
        if (USE_TRIM_PULSE) {
          runPhase = RP_TRIM_ON;
        } else {
          runPhase = RP_HOLD_ON;
          holdPhaseStartMs = now;
        }
        runPhaseStartMs = now;
      }
      break;

    case RP_TRIM_ON:
      if (phaseElapsed >= TRIM_ON_MS) {
        runPhase = RP_TRIM_COAST;
        runPhaseStartMs = now;
      }
      break;

    case RP_TRIM_COAST:
      if (phaseElapsed >= TRIM_COAST_MS) {
        runPhase = RP_HOLD_ON;
        holdPhaseStartMs = now;
        runPhaseStartMs = now;
      }
      break;

    case RP_HOLD_ON: {
      if (phaseElapsed >= HOLD_ON_MS) {
        runPhase = RP_HOLD_OFF;
        runPhaseStartMs = now;
      }
      unsigned long holdElapsed = now - holdPhaseStartMs;
      if (holdElapsed >= holdTargetMsActive) {
        runPhase = RP_COMPLETE;
      }
      break;
    }

    case RP_HOLD_OFF: {
      if (phaseElapsed >= HOLD_OFF_MS) {
        runPhase = RP_HOLD_ON;
        runPhaseStartMs = now;
      }
      unsigned long holdElapsed = now - holdPhaseStartMs;
      if (holdElapsed >= holdTargetMsActive) {
        runPhase = RP_COMPLETE;
      }
      break;
    }

    case RP_COMPLETE:
    default:
      break;
  }
}

void updateStateMachine() {
  unsigned long now = millis();

  switch (state) {
    case ST_BOOT:
      // Commissioning override:
      // Hold START + STOP during boot to clear persisted recovery flags.
      {
        bool rawStart = digitalRead(IN_START_PB);
        bool startPressed = START_ACTIVE_LOW ? (rawStart == LOW) : (rawStart == HIGH);
        bool stopPressed = stopPressedRaw(digitalRead(IN_STOP_PB));
        if (startPressed && stopPressed) {
          cycle_interrupted = false;
          destruct_required = false;
          fault_latched = false;
          estop_latched = false;
          persistFlags();
          Serial.println(F("BOOT OVERRIDE: recovery flags cleared by START+STOP hold."));
        }
      }

      if (!estop_ok()) {
        estop_latched = true;
        fault_latched = true;
        enterState(ST_ESTOP);
        break;
      }
      if (cycle_interrupted || destruct_required) {
        enterState(ST_BOOT_RECOVER);
      } else {
        enterState(ST_SAFE_OPEN);
      }
      break;

    case ST_BOOT_RECOVER:
      // Conservative: no auto-unlock. If destruct is owed and chamber is closed,
      // perform destruct; else remain fault until operator resolves conditions.
      if (!estop_ok()) {
        enterState(ST_ESTOP);
        break;
      }

      if (destruct_required || cycle_interrupted) {
        if (door_stable_closed) {
          enterState(ST_DESTRUCT);
        } else {
          fault_latched = true;
          enterState(ST_FAULT);
        }
      } else {
        enterState(ST_SAFE_OPEN);
      }
      break;

    case ST_SAFE_OPEN:
      // Unlock permitted in this state.
      // Do not auto-lock; lock sequence starts only on START request.
      if (req_start) {
        if (door_stable_closed) {
          enterState(ST_LOCKING);
        } else {
          // Ignore stale start requests while door is open.
          req_start = false;
          Serial.println(F("START ignored in SAFE_OPEN: door not closed."));
        }
      }
      break;

    case ST_LOCKING:
      if (!door_stable_closed) {
        enterState(ST_SAFE_OPEN);
        break;
      }
      if (lock_engaged()) {
        enterState(ST_READY_LOCKED);
        break;
      }
      if (now - stateEnteredMs >= T_LOCK_TIMEOUT_MS) {
        fault_latched = true;
        enterState(ST_FAULT);
      }
      break;

    case ST_READY_LOCKED:
      if (!door_stable_closed) {
        fault_latched = true;
        enterState(ST_FAULT);
        break;
      }
      if (req_start && permissives_for_run()) {
        req_start = false;
        enterState(ST_STARTING);
        break;
      }
      if (req_stop) {
        req_stop = false;
        // Conservative: stop request while armed -> controlled stop path + destruct
        destruct_required = true;
        persistFlags();
        enterState(ST_STOPPING);
      }
      break;

    case ST_STARTING:
      // Stop in STARTING: immediate STOPPING then DESTRUCT (as agreed)
      if (req_stop) {
        req_stop = false;
        destruct_required = true;
        persistFlags();
        enterState(ST_STOPPING);
        break;
      }

      if (!permissives_for_run()) {
        fault_latched = true;
        enterState(ST_FAULT);
        break;
      }

      if (now - stateEnteredMs >= T_PRESTART_AIRFLOW_MS) {
        enterState(ST_RUNNING);
      }
      break;

    case ST_RUNNING:
      if (req_stop) {
        req_stop = false;
        destruct_required = true;
        persistFlags();
        enterState(ST_STOPPING);
        break;
      }

      updateRunningPhase();

      // Optional feedback checks while running
      if (!pump_fb_ok()) {
        fault_latched = true;
        enterState(ST_FAULT);
        break;
      }

      if (runPhase == RP_COMPLETE) {
        enterState(ST_STOPPING);
      }
      break;

    case ST_STOPPING:
      // Plasma is already forced off by state outputs.
      if (now - stateEnteredMs >= T_POST_PLASMA_DELAY_MS) {
        enterState(ST_DESTRUCT);
      }
      break;

    case ST_DESTRUCT: {
      unsigned long tickNow = now;
      unsigned long dt = tickNow - destructLastTickMs;
      destructLastTickMs = tickNow;

      bool destructGood = true;
      // Minimum agreed MVP: timer + minimum pump/airflow permissive if available
      // In this implementation, pump is commanded ON in ST_DESTRUCT.
      if (USE_IN_FLOW_OK && !flow_ok()) destructGood = false;
      if (USE_IN_PUMP_FB && !pump_fb_ok()) destructGood = false;

      if (destructGood) destructGoodMs += dt;

      unsigned long elapsed = now - stateEnteredMs;
      if (elapsed >= T_DESTRUCT_DURATION_MS && destructGoodMs >= T_DESTRUCT_DURATION_MS) {
        enterState(ST_SAFE_RELEASE);
      }
      break;
    }

    case ST_SAFE_RELEASE:
      // Recovery flags are cleared on entry to SAFE_RELEASE.
      // Immediately return to SAFE_OPEN for normal idle behavior.
      enterState(ST_SAFE_OPEN);
      break;

    case ST_FAULT:
      // Unlock is NOT allowed in FAULT (MVP policy)
      if (stopLongPressResetEvent()) {
        if (estop_ok()) {
          // Clear stale edge requests from the reset-hold action.
          req_stop = false;
          req_start = false;
          enterState(ST_BOOT_RECOVER);
        }
      }
      break;

    case ST_ESTOP:
      // Latching behavior: no auto-recovery.
      if (stopLongPressResetEvent()) {
        if (estop_ok()) {
          // Clear stale edge requests from the reset-hold action.
          req_stop = false;
          req_start = false;
          enterState(ST_BOOT_RECOVER);
        }
      }
      break;
  }
}

void SoloSystemWriteEnable(){
  //Digital Write to Solo System MCU
  if(Solo_System_State){
    digitalWrite(SOLO_OUT_PIN, LOW);
  }
  else{
    digitalWrite(SOLO_OUT_PIN, HIGH);
  }

}

void clearOneShotRequests() {
  // Start/Stop are edge requests; leave them true until consumed in state logic.
  // Nothing to do here; state machine consumes and clears.
}

void setup() {
  pinMode(OUT_PLASMA_REQ, OUTPUT);
  pinMode(OUT_PUMP, OUTPUT);
  pinMode(OUT_VALVE_A, OUTPUT);
  pinMode(OUT_VALVE_DESTRUCT, OUTPUT);
  pinMode(OUT_LOCK_RELEASE, OUTPUT);
  pinMode(OUT_LAMP_GREEN, OUTPUT);
  pinMode(OUT_LAMP_RED, OUTPUT);
  pinMode(OUT_LAMP_BLUE, OUTPUT);
  pinMode(SOLO_OUT_PIN, OUTPUT);

  pinMode(IN_START_PB, INPUT_PULLUP);
  pinMode(IN_STOP_PB, INPUT_PULLUP);
  pinMode(IN_MODE_6H_SEL, INPUT_PULLUP);

  digitalWrite(SOLO_OUT_PIN, HIGH);

  if (USE_IN_ESTOP_OK) {
    // estop_ok should be actively driven by safety hardware
    pinMode(IN_ESTOP_OK, INPUT);
  }

  if (USE_IN_DOOR_CLOSED) pinMode(IN_DOOR_CLOSED, INPUT_PULLUP);
  if (USE_IN_LOCK_ENGAGED) pinMode(IN_LOCK_ENGAGED, INPUT_PULLUP);
  if (USE_IN_FLOW_OK) pinMode(IN_FLOW_OK, INPUT_PULLUP);
  if (USE_IN_SAFETY_OK) pinMode(IN_SAFETY_OK, INPUT_PULLUP);
  if (USE_IN_PUMP_FB) pinMode(IN_PUMP_FB, INPUT_PULLUP);

  Serial.begin(9600);
  Serial.println("PF_CC_1_MVP boot");
  Serial.println("Run mode select: D9 NO switch (open=2H, closed=6H)");
  Serial.println("Reset action: hold STOP 2s in FAULT/ESTOP");

  // Establish initial mode selector state.
  stableMode6h = (digitalRead(IN_MODE_6H_SEL) == LOW);
  lastRawMode6h = stableMode6h;
  lastModeChangeMs = millis();
  Serial.print(F("MODE SELECTOR (boot) -> "));
  Serial.println(stableMode6h ? F("6H") : F("2H"));

  restoreFlags();
  Serial.print(F("BOOT FLAGS: cycle_interrupted="));
  Serial.print(cycle_interrupted ? F("1") : F("0"));
  Serial.print(F(", destruct_required="));
  Serial.println(destruct_required ? F("1") : F("0"));
  Serial.println((cycle_interrupted || destruct_required)
    ? F("BOOT DECISION: BOOT_RECOVER (recovery/destruct owed)")
    : F("BOOT DECISION: SAFE_OPEN (no recovery owed)"));

  // Establish initial debounced door state
  lastDoorRaw = door_closed_raw();
  door_stable_closed = lastDoorRaw;
  lastDoorChangeMs = millis();
  lastReportedDoorStable = door_stable_closed;
  lastLockRaw = lock_engaged_raw();
  lock_stable_engaged = lastLockRaw;
  lastLockChangeMs = millis();

  enterState(ST_BOOT);
  applyOutputsForState();

  Serial.print(F("["));
  Serial.print(millis());
  Serial.print(F(" ms] ACTIVE STATE: "));
  Serial.println(stateName(state));
}

void loop() {
  updateDebouncedInputs();

  // Demo override: immediate safe stop + unlock (bypasses normal destruct sequence).
  if (demoAbortHoldEvent()) {
    if (estop_ok()) {
      req_start = false;
      req_stop = false;
      fault_latched = false;
      estop_latched = false;
      destruct_required = false;
      cycle_interrupted = false;
      persistFlags();
      Serial.println(F("DEMO OVERRIDE: START+STOP 5s -> SAFE_OPEN (all stop, unlock)."));
      enterState(ST_SAFE_OPEN);
    } else {
      Serial.println(F("DEMO OVERRIDE BLOCKED: E-stop not healthy."));
    }
  }

  handleGlobalSafety();
  updateStateMachine();
  applyOutputsForState();
  clearOneShotRequests();
  delay(T_LOOP_POLL_MS);
}
