#include <Servo.h>

// ---- Candidate pins to scan (Teensy 4.1 common PWM-capable) ----
int pins[] = {2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23};
const int N = sizeof(pins)/sizeof(pins[0]);

// ---- Assigned pins for control mode (you can change via serial) ----
int STEER_PIN = 2;   // will be changed with SET_STEER_PIN:<n>
int ESC_PIN   = 3;   // will be changed with SET_ESC_PIN:<n>

Servo servoSteer;
Servo servoEsc;

const int US_MIN=1000, US_MID=1500, US_MAX=2000;
unsigned long lastCmdMs = 0;
const unsigned long FAILSAFE_MS = 800;

String line;

int clampUs(int us){ if(us<US_MIN) return US_MIN; if(us>US_MAX) return US_MAX; return us; }
int normToUs(float v){
  if(v<-1) v=-1; if(v>1) v=1;
  return clampUs((int)(US_MID + v*(US_MAX-US_MID)));
}

void toNeutral(){
  servoSteer.writeMicroseconds(US_MID);
  if (servoEsc.attached()) servoEsc.writeMicroseconds(US_MID);
}

void attachControlServos() {
  if (servoSteer.attached()) servoSteer.detach();
  if (servoEsc.attached())   servoEsc.detach();
  servoSteer.attach(STEER_PIN, US_MIN, US_MAX);
  servoSteer.writeMicroseconds(US_MID);
  // ESC is optional; attach only if you plan to use it
  servoEsc.attach(ESC_PIN, US_MIN, US_MAX);
  servoEsc.writeMicroseconds(US_MID);
}

void setup() {
  Serial.begin(115200);
  delay(1200);
  Serial.println("Teensy ready. Commands: SCAN | SET_STEER_PIN:n | SET_ESC_PIN:n | s:<-1..1> t:<-1..1>");
  attachControlServos();
  lastCmdMs = millis();
}

void doScan() {
  Serial.println("SCAN start");
  // Use a temporary Servo so we don't disturb the assigned control pins
  Servo s;
  for (int i=0; i<N; ++i) {
    int p = pins[i];
    Serial.print("TRY "); Serial.println(p);
    s.attach(p, US_MIN, US_MAX);
    s.writeMicroseconds(1000); delay(350);
    s.writeMicroseconds(2000); delay(350);
    s.writeMicroseconds(1500); delay(250);
    s.detach();
    delay(150);
  }
  Serial.println("SCAN end");
  // Re-attach control servos on chosen pins
  attachControlServos();
}

void handleLine(const String& cmd){
  if (cmd.equalsIgnoreCase("SCAN")) {
    doScan();
    return;
  }

  if (cmd.startsWith("SET_STEER_PIN:")) {
    int p = cmd.substring(14).toInt();
    STEER_PIN = p;
    Serial.print("STEER_PIN="); Serial.println(STEER_PIN);
    attachControlServos();
    return;
  }

  if (cmd.startsWith("SET_ESC_PIN:")) {
    int p = cmd.substring(12).toInt();
    ESC_PIN = p;
    Serial.print("ESC_PIN="); Serial.println(ESC_PIN);
    attachControlServos();
    return;
  }

  // Control: "s:<-1..1> t:<-1..1>" (order doesn't matter; each optional)
  float steer = 0.0f, thr = 0.0f;
  int si = cmd.indexOf("s:");
  int ti = cmd.indexOf("t:");
  if (si >= 0) steer = cmd.substring(si+2).toFloat();
  if (ti >= 0) thr   = cmd.substring(ti+2).toFloat();

  servoSteer.writeMicroseconds(normToUs(steer));
  if (servoEsc.attached()) servoEsc.writeMicroseconds(normToUs(thr));
  lastCmdMs = millis();
}

void loop() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (line.length()) { handleLine(line); line = ""; }
    } else {
      line += c;
    }
  }

  if (millis() - lastCmdMs > FAILSAFE_MS) {
    toNeutral();
  }
}
