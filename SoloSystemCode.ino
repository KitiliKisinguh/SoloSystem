const int switchPin = 5;   // Switch from D5 to GND 
const int RELAY4  = 10;   // Relay4
const int RELAY3= 8; //VFD Control
const int trigPin=7;


bool SwitchFlag= true; //Switch is Inverted
bool pumpON=false;
bool waveOn = false; 

const unsigned long debounceTime = 50;  // ms
const unsigned long rampDelay = 10000; // 10 seconds

int lastReading = HIGH;        // raw last read state
int stableState = HIGH;        // debounced stable state
unsigned long lastChangeTime = 0;


enum State {
  OFF_STATE,
  STARTING_PUMP,
  RUNNING,
  STOPPING_PUMP
};

State systemState = OFF_STATE;
unsigned long stateStartTime = 0;

void setup() {
  pinMode(switchPin, INPUT_PULLUP); //Active Low
  pinMode(RELAY4, OUTPUT);
  pinMode(RELAY3, OUTPUT);
  Serial.begin(115200);
  // relay OFF initially
  digitalWrite(RELAY4, HIGH); //When High Plasma OFF  
  digitalWrite(RELAY3, LOW); // Pump off
}

void loop() {
  int reading = digitalRead(switchPin);
  // Serial.print("Switch Input: ");
  // Serial.println(reading);   // prints 0 or 1; 0 Switch is OFF

  // If input changed, reset debounce timer
  if (reading != lastReading) {
    lastChangeTime = millis();
    lastReading = reading;
  }

  // If input has stayed unchanged long enough, accept it
  if ((millis() - lastChangeTime) > debounceTime) {
  if (reading != stableState) {
    stableState = reading;

    // Switch enabled = HIGH
    if (stableState == HIGH) {
      digitalWrite(RELAY3, HIGH);   // Pump ON
      SwitchFlag = true;
      Serial.println("Switch Enabled");

      stateStartTime = millis();
      systemState = STARTING_PUMP;
    } 
    else {
      digitalWrite(RELAY4, HIGH);    // Plasma OFF
      SwitchFlag = false;
      Serial.println("Switch Disabled");

      stateStartTime = millis();
      systemState = STOPPING_PUMP;
    }
  }
}

  switch (systemState) {
    case STARTING_PUMP:
      if (millis() - stateStartTime >= rampDelay) {
        digitalWrite(RELAY4, LOW);   // Plasma ON
        Serial.println("Plasma ON");
        systemState = RUNNING;
      }
      break;

    case STOPPING_PUMP:
      if (millis() - stateStartTime >= rampDelay) {
        digitalWrite(RELAY3, LOW);   // Pump OFF
        Serial.println("Pump OFF");
        systemState = OFF_STATE;
      }
      break;
  }
}  
