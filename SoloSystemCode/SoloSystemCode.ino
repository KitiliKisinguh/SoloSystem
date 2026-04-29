const int switchPin  = 5;   // Switch input
const int RELAY4     = 10;  // Plasma relay
const int RELAY3     = 8;   // Pump / VFD control
const int SBDChamber = 2;   // SBD chamber input interrupt pin

const unsigned long debounceTime = 50;
const unsigned long pumpDelayTime = 10000; // 10 seconds

bool systemRequested = false;
bool systemRunning = false;

volatile bool sbdChanged = false;
volatile bool sbdState = false;

int lastReading = LOW;
int stableState = LOW;
unsigned long lastChangeTime = 0;

unsigned long stateStartTime = 0;

enum SystemState {
  SYSTEM_OFF,
  PUMP_STARTING,
  PLASMA_ON,
  PLASMA_STOPPING
};

SystemState currentState = SYSTEM_OFF;

void onSBDChange() {
  sbdState = digitalRead(SBDChamber);
  sbdChanged = true;
}

void requestSystemOn() {
  if (currentState == SYSTEM_OFF) {
    digitalWrite(RELAY3, HIGH);   // Pump ON
    Serial.println("Pump starting...");

    stateStartTime = millis();
    currentState = PUMP_STARTING;
  }
}

void requestSystemOff() {
  if (currentState == PLASMA_ON || currentState == PUMP_STARTING) {
    digitalWrite(RELAY4, LOW);    // Plasma OFF first
    Serial.println("Plasma OFF, pump ramping down...");

    stateStartTime = millis();
    currentState = PLASMA_STOPPING;
  }
}

void updateSystemState() {
  switch (currentState) {

    case SYSTEM_OFF:
      digitalWrite(RELAY3, LOW);
      digitalWrite(RELAY4, LOW);
      systemRunning = false;
      break;

    case PUMP_STARTING:
      if (millis() - stateStartTime >= pumpDelayTime) {
        digitalWrite(RELAY4, HIGH);   // Plasma ON after pump delay
        Serial.println("Pump ON | Plasma ON");

        systemRunning = true;
        currentState = PLASMA_ON;
      }
      break;

    case PLASMA_ON:
      // System remains active until requested OFF
      break;

    case PLASMA_STOPPING:
      if (millis() - stateStartTime >= pumpDelayTime) {
        digitalWrite(RELAY3, LOW);    // Pump OFF after ramp-down
        Serial.println("Pump OFF");

        systemRunning = false;
        currentState = SYSTEM_OFF;
      }
      break;
  }
}

void setup() {
  pinMode(switchPin, INPUT_PULLUP);
  pinMode(SBDChamber, INPUT_PULLUP);

  pinMode(RELAY4, OUTPUT);
  pinMode(RELAY3, OUTPUT);

  Serial.begin(115200);

  digitalWrite(RELAY4, LOW);  // Plasma OFF
  digitalWrite(RELAY3, LOW);  // Pump OFF

  attachInterrupt(digitalPinToInterrupt(SBDChamber), onSBDChange, CHANGE);

  Serial.println("System Ready");
}

void loop() {
  int reading = digitalRead(switchPin);

  if (reading != lastReading) {
    lastChangeTime = millis();
    lastReading = reading;
  }

  if ((millis() - lastChangeTime) > debounceTime) {
    if (reading != stableState) {
      stableState = reading;

      if (stableState == HIGH) {
        Serial.println("Switch Enabled");
        requestSystemOn();
      } 
      else {
        Serial.println("Switch Disabled");
        requestSystemOff();
      }
    }
  }

  if (sbdChanged) {
    sbdChanged = false;

    if (sbdState == HIGH) {
      Serial.println("SBD signal HIGH");
      requestSystemOn();
    } 
    else {
      Serial.println("SBD signal LOW");
      requestSystemOff();
    }
  }

  updateSystemState();
}
