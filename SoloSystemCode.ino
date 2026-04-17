const int switchPin = 5;   // Switch from D5 to GND
const int RELAY4  = 10;   // Relay4
const int RELAY3= 8; //VFD Control
const int trigPin=7;
//Plasma only High When Pump is On
bool SwitchFlag= true; //Switch is Inverted
bool pumpON=false;
bool waveOn = false; 
const unsigned long debounceTime = 50;  // ms

int lastReading = HIGH;        // raw last read state
int stableState = HIGH;        // debounced stable state
unsigned long lastChangeTime = 0;

void setup() {
  pinMode(switchPin, INPUT_PULLUP);
  pinMode(RELAY4, OUTPUT);
  pinMode(RELAY3, OUTPUT);
  Serial.begin(115200);
  // relay OFF initially
  digitalWrite(RELAY4, HIGH); //When High Plasma OFF  
  digitalWrite(RELAY3, LOW); 
}

void loop() {
  int reading = digitalRead(switchPin);
  Serial.print("Switch Input: ");
  Serial.println(reading);   // prints 0 or 1; 0 Switch is OFF

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
      digitalWrite(RELAY3, HIGH);   // Pump on 
      SwitchFlag = true;
      Serial.println("Switch Enabled");

      delay(10000); //Pump ramp up: 10 Seconds

      digitalWrite(RELAY4, LOW);   // Plasma on
      Serial.println("Pump ON");
    } 
    else {
      digitalWrite(RELAY4, HIGH);    // Plasma / relay OFF
      SwitchFlag = false;
      Serial.println("Switch Disabled");

      delay(10000); //Pump ramp down: 10 seconds

      digitalWrite(RELAY3, LOW);    // Pump OFF
      Serial.println("Pump OFF");
    }
  }
}
  //VFD Control(Pump control)
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    if (cmd == "ON") {
      digitalWrite(RELAY3, HIGH);
      Serial.println("PUMP ON");
      pumpON= true;
    } 
    else if (cmd == "OFF") {
      digitalWrite(RELAY3, LOW);
      Serial.println("PUMP OFF");
      pumpON= false;
    } 
    
//Sig_gen trig
    else if (cmd == "SIGON") {
      waveOn = true;
      Serial.println("Signal Generator ON");
    } 
    else if (cmd == "SIGOFF") {
      waveOn = false;
      Serial.println("Signal Generator OFF");
    } 
    else {
      Serial.println("Commands: ON, OFF, SIGON, SIGOFF");
    }
}

  // Inverted output for FY6900
  digitalWrite(trigPin, waveOn ? LOW : HIGH);
}