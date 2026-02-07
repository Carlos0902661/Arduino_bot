// Archived Phase-1 test sketch: line-follow + initiator gate + clocked-bar decoding via Serial.
// Uses Serial simulation for sensor input and barcode timing.
// Not intended for production firmware.
// Target: Arduino Nano (ATmega328P)
// File: phase1_motor_linefollow_serial_barcode.ino

// ---------------------------
// Tunable constants
// ---------------------------
const unsigned long gate_debounce_ms = 120; // ms of continuous 111 to trigger initiator
const unsigned long CELL_MS = 120;          // ms per barcode cell
const unsigned long TOL_MS = 35;            // warn if BAR dt not near multiple of CELL_MS

const int baseSpeed = 140;   // 0..255
const int turnAdjust = 80;   // 0..255

// ---------------------------
// Motor pins (DRV8833)
// ---------------------------
const int LEFT_IN1  = 4;  // D4
const int LEFT_IN2  = 5;  // D5 (PWM)
const int RIGHT_IN1 = 6;  // D6 (PWM)
const int RIGHT_IN2 = 7;  // D7

bool invertLeft = false;
bool invertRight = false;

// ---------------------------
// State machine
// ---------------------------
enum State {
  FOLLOW,
  READING
};

State state = FOLLOW;

// Simulated sensors
int L = 0, C = 0, R = 0;

// Gate debounce + latch
bool gateLatched = false;
bool gateTiming = false;
unsigned long gateStartMs = 0;

// Reading / decoding
String bitBuffer = "";
bool readingActive = false;

// Status print timing
unsigned long lastStatusMs = 0;
const unsigned long statusIntervalMs = 4000;

// ---------------------------
// Motor helpers
// ---------------------------
void stopMotors() {
  // Safe stop (coast)
  analogWrite(LEFT_IN1, 0);
  analogWrite(LEFT_IN2, 0);
  analogWrite(RIGHT_IN1, 0);
  analogWrite(RIGHT_IN2, 0);
}

void setMotorPWM(int left, int right) {
  // Signed PWM (-255..255). Positive = forward.
  if (invertLeft) left = -left;
  if (invertRight) right = -right;

  left = constrain(left, -255, 255);
  right = constrain(right, -255, 255);

  // DRV8833: drive IN1/IN2 as forward/reverse (one side PWM)
  if (left >= 0) {
    analogWrite(LEFT_IN1, left); // Note: D4 is not PWM on Nano; used as digital here
    analogWrite(LEFT_IN2, 0);
  } else {
    analogWrite(LEFT_IN1, 0);
    analogWrite(LEFT_IN2, -left);
  }

  if (right >= 0) {
    analogWrite(RIGHT_IN1, right);
    analogWrite(RIGHT_IN2, 0);
  } else {
    analogWrite(RIGHT_IN1, 0);
    analogWrite(RIGHT_IN2, -right);
  }
}

// ---------------------------
// Utility
// ---------------------------
void printStatus() {
  Serial.print("[STATUS] state=");
  Serial.print(state == FOLLOW ? "FOLLOW" : "READING");
  Serial.print(" LCR=");
  Serial.print(L); Serial.print(C); Serial.print(R);
  Serial.print(" gateTiming=");
  Serial.print(gateTiming ? "1" : "0");
  Serial.print(" gateLatched=");
  Serial.print(gateLatched ? "1" : "0");
  Serial.print(" bitBuffer=");
  Serial.println(bitBuffer);
}

void logLCR() {
  Serial.print("[LCR] ");
  Serial.print(L); Serial.print(C); Serial.print(R);
  Serial.println();
}

bool is111() {
  return (L == 1 && C == 1 && R == 1);
}

// ---------------------------
// Gate debounce + latch
// ---------------------------
void updateGateDetection() {
  if (is111()) {
    if (!gateTiming && !gateLatched) {
      gateTiming = true;
      gateStartMs = millis();
    }
    if (gateTiming && !gateLatched) {
      unsigned long dt = millis() - gateStartMs;
      if (dt >= gate_debounce_ms) {
        gateLatched = true;
        gateTiming = false;
        state = READING;
        readingActive = true;
        Serial.println("[EVENT] Initiator gate detected -> READING");
      }
    }
  } else {
    if (gateTiming) {
      gateTiming = false;
    }
    if (gateLatched) {
      gateLatched = false;
      Serial.println("[EVENT] Gate latch reset (left 111)");
    }
  }
}

// ---------------------------
// Line following (simple rule table)
// ---------------------------
void doFollow() {
  int leftPWM = 0;
  int rightPWM = 0;

  if (L == 0 && C == 1 && R == 0) {
    leftPWM = baseSpeed;
    rightPWM = baseSpeed;
  } else if ((L == 1 && C == 0 && R == 0) || (L == 1 && C == 1 && R == 0)) {
    leftPWM = baseSpeed - turnAdjust;
    rightPWM = baseSpeed + turnAdjust;
  } else if ((L == 0 && C == 0 && R == 1) || (L == 0 && C == 1 && R == 1)) {
    leftPWM = baseSpeed + turnAdjust;
    rightPWM = baseSpeed - turnAdjust;
  } else if (L == 0 && C == 0 && R == 0) {
    leftPWM = 0;
    rightPWM = 0;
  } else {
    // Any other pattern, default safe
    leftPWM = 0;
    rightPWM = 0;
  }

  setMotorPWM(leftPWM, rightPWM);
}

// ---------------------------
// Serial command parsing
// ---------------------------
String readLine() {
  static String line = "";
  while (Serial.available() > 0) {
    char ch = (char)Serial.read();
    if (ch == '\r') {
      // ignore
    } else if (ch == '\n') {
      String out = line;
      line = "";
      return out;
    } else {
      line += ch;
    }
  }
  return "";
}

void handleBarCommand(unsigned long dtMs) {
  // Decode dt directly into zeros + 1
  float kf = (float)dtMs / (float)CELL_MS;
  int k = (int)lround(kf);

  if (k < 1) k = 1;

  unsigned long nearest = (unsigned long)k * CELL_MS;
  if (dtMs + TOL_MS < nearest || dtMs > nearest + TOL_MS) {
    Serial.print("[WARN] BAR dt not near multiple: dt=");
    Serial.print(dtMs);
    Serial.print(" nearest=");
    Serial.println(nearest);
  }

  // Append (k-1) zeros then a 1
  for (int i = 0; i < k - 1; i++) {
    bitBuffer += "0";
  }
  bitBuffer += "1";

  Serial.print("[BAR] dt=");
  Serial.print(dtMs);
  Serial.print(" k=");
  Serial.print(k);
  Serial.print(" appended=");
  for (int i = 0; i < k - 1; i++) Serial.print("0");
  Serial.println("1");
  Serial.print("[BITS] ");
  Serial.println(bitBuffer);
}

void handleCommand(const String &cmd) {
  if (cmd.length() == 0) return;

  if (cmd.length() == 3 && (cmd[0] == '0' || cmd[0] == '1')
      && (cmd[1] == '0' || cmd[1] == '1') && (cmd[2] == '0' || cmd[2] == '1')) {
    L = (cmd[0] == '1');
    C = (cmd[1] == '1');
    R = (cmd[2] == '1');
    logLCR();
    return;
  }

  if (cmd == "S") {
    printStatus();
    return;
  }

  if (cmd == "RESET") {
    state = FOLLOW;
    readingActive = false;
    bitBuffer = "";
    gateLatched = false;
    gateTiming = false;
    Serial.println("[EVENT] RESET -> FOLLOW, bits cleared");
    return;
  }

  if (cmd == "END" || cmd == "R") {
    state = FOLLOW;
    readingActive = false;
    Serial.println("[EVENT] END -> FOLLOW (bits preserved)");
    return;
  }

  if (cmd.startsWith("BAR ")) {
    if (state != READING) {
      Serial.println("[INFO] BAR ignored (not in READING)");
      return;
    }
    String num = cmd.substring(4);
    unsigned long dtMs = (unsigned long)num.toInt();
    if (dtMs == 0) {
      Serial.println("[WARN] BAR dt invalid or zero");
      return;
    }
    handleBarCommand(dtMs);
    return;
  }

  Serial.print("[WARN] Unknown command: ");
  Serial.println(cmd);
}

// ---------------------------
// Arduino setup/loop
// ---------------------------
void setup() {
  Serial.begin(115200);

  pinMode(LEFT_IN1, OUTPUT);
  pinMode(LEFT_IN2, OUTPUT);
  pinMode(RIGHT_IN1, OUTPUT);
  pinMode(RIGHT_IN2, OUTPUT);

  stopMotors();

  Serial.println("[BOOT] Phase-1 line-follow + clocked-bar decode ready");
}

void loop() {
  String cmd = readLine();
  if (cmd.length() > 0) {
    handleCommand(cmd);
  }

  if (state == FOLLOW) {
    updateGateDetection();
    doFollow();

    if (millis() - lastStatusMs >= statusIntervalMs) {
      lastStatusMs = millis();
      printStatus();
    }
  } else if (state == READING) {
    // Stay in READING until END/RESET command
    if (millis() - lastStatusMs >= statusIntervalMs) {
      lastStatusMs = millis();
      printStatus();
    }
  }
}

/*
Testing in Serial Monitor (newline-terminated):
1) Send "010" to simulate centered line (FOLLOW).
2) Send "111" and keep it for > gate_debounce_ms to trigger initiator.
3) Now send BAR events (in READING):
   - "BAR 120" -> appends "1"
   - "BAR 240" -> appends "01"
   Example: initiator, then BAR 120 then BAR 240 yields bitstring "101".
4) Send "END" (or "R") to return to FOLLOW (bits preserved).
5) Send "RESET" to clear bits and return to FOLLOW.
*/
