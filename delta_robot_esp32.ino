/*
  Delta Robot Slave Controller (ESP32)
  ------------------------------------
  This firmware does NOT do any kinematics or math. It is a "dumb"
  slave that receives absolute target step positions for 3 motors
  plus a move duration, and pulses the stepper drivers to get there.

  Motor 1 & Motor 2: standard STEP/DIR drivers
  Motor 3: CL57RS closed-loop driver in CW/CCW pulse mode
           (no DIR pin - instead you pulse the CW input to move
           clockwise, or pulse the CCW input to move counter-clockwise)

  Serial protocol (line based, 115200 baud, newline terminated):
    M <s1> <s2> <s3> <t_ms>   -> move to absolute step positions s1,s2,s3
                                  over t_ms milliseconds (all axes finish
                                  together)
    H                         -> run homing routine (limit switches)
    EN 1 / EN 0               -> enable / disable all drivers
    STOP                      -> abort current move immediately
    P?                        -> report current step positions

  Responses sent back to Python:
    OK                        -> command accepted
    DONE                      -> move finished
    HOMED                     -> homing finished
    POS <s1> <s2> <s3>        -> current step position report
    ERR <message>             -> something went wrong
*/

// ---------------- PIN DEFINITIONS ----------------
// Motor 1 - standard STEP/DIR driver
#define M1_STEP_PIN   25
#define M1_DIR_PIN    26

// Motor 2 - standard STEP/DIR driver
#define M2_STEP_PIN   27
#define M2_DIR_PIN    14

// Motor 3 - CL57RS closed loop driver, CW/CCW pulse inputs
#define M3_CW_PIN     32
#define M3_CCW_PIN    33

// Shared enable line(s). If your drivers use separate enable pins,
// add more #defines and set them all in setEnabled().
#define EN_PIN        13
#define EN_ACTIVE_LOW true   // most STEP/DIR drivers enable on LOW

// Optional homing limit switches (one per arm), wired to GND with
// internal pullups. Leave commented out / ignore if you home manually.
#define LIMIT1_PIN    34
#define LIMIT2_PIN    35
#define LIMIT3_PIN    36
#define LIMIT_ACTIVE_LOW true

// Minimum pulse width (microseconds) - check your driver datasheet.
// CL57RS typically wants >= 5us high time.
const uint16_t PULSE_WIDTH_US = 8;

// ---------------- AXIS STATE ----------------
struct Axis {
  long currentSteps = 0;
  long targetSteps  = 0;
  long stepsRemaining = 0;
  int  direction = 1;          // 1 = forward/CW, -1 = backward/CCW
  unsigned long intervalUs = 0; // time between steps for this axis
  unsigned long lastStepUs = 0;
  bool moving = false;
};

Axis axis[3];
bool moveInProgress = false;
unsigned long moveStartMs = 0;
unsigned long moveDurationMs = 0;

// ---------------- LOW LEVEL STEP PULSES ----------------

// Motor 1: standard step/dir pulse
void pulseM1(int dir) {
  digitalWrite(M1_DIR_PIN, dir > 0 ? HIGH : LOW);
  digitalWrite(M1_STEP_PIN, HIGH);
  delayMicroseconds(PULSE_WIDTH_US);
  digitalWrite(M1_STEP_PIN, LOW);
}

// Motor 2: standard step/dir pulse
void pulseM2(int dir) {
  digitalWrite(M2_DIR_PIN, dir > 0 ? HIGH : LOW);
  digitalWrite(M2_STEP_PIN, HIGH);
  delayMicroseconds(PULSE_WIDTH_US);
  digitalWrite(M2_STEP_PIN, LOW);
}

// Motor 3: CL57RS - pulse CW line to go one way, CCW line to go the
// other way. Never pulse both at once.
void pulseM3(int dir) {
  if (dir > 0) {
    digitalWrite(M3_CW_PIN, HIGH);
    delayMicroseconds(PULSE_WIDTH_US);
    digitalWrite(M3_CW_PIN, LOW);
  } else {
    digitalWrite(M3_CCW_PIN, HIGH);
    delayMicroseconds(PULSE_WIDTH_US);
    digitalWrite(M3_CCW_PIN, LOW);
  }
}

void pulseAxis(int i, int dir) {
  switch (i) {
    case 0: pulseM1(dir); break;
    case 1: pulseM2(dir); break;
    case 2: pulseM3(dir); break;
  }
}

void setEnabled(bool enabled) {
  bool level = EN_ACTIVE_LOW ? !enabled : enabled;
  digitalWrite(EN_PIN, level ? HIGH : LOW);
}

// ---------------- MOVE PLANNING ----------------

// Starts a coordinated move: every axis begins now and is spaced so
// that all three finish at (approximately) the same time = durationMs.
void startMove(long t1, long t2, long t3, unsigned long durationMs) {
  long targets[3] = {t1, t2, t3};

  for (int i = 0; i < 3; i++) {
    long diff = targets[i] - axis[i].currentSteps;
    axis[i].targetSteps = targets[i];
    axis[i].direction = (diff >= 0) ? 1 : -1;
    axis[i].stepsRemaining = labs(diff);

    if (axis[i].stepsRemaining > 0 && durationMs > 0) {
      axis[i].intervalUs = (durationMs * 1000UL) / axis[i].stepsRemaining;
      axis[i].moving = true;
    } else {
      axis[i].intervalUs = 0;
      axis[i].moving = false;
    }
    axis[i].lastStepUs = micros();
  }

  moveInProgress = true;
  moveStartMs = millis();
  moveDurationMs = durationMs;
}

// Call continuously from loop(). Non-blocking - fires a pulse on
// whichever axis is due for one, independently for all 3 motors.
void serviceMove() {
  if (!moveInProgress) return;

  bool anyMoving = false;
  unsigned long now = micros();

  for (int i = 0; i < 3; i++) {
    if (!axis[i].moving) continue;
    anyMoving = true;

    if ((now - axis[i].lastStepUs) >= axis[i].intervalUs) {
      pulseAxis(i, axis[i].direction);
      axis[i].currentSteps += axis[i].direction;
      axis[i].stepsRemaining--;
      axis[i].lastStepUs = now;

      if (axis[i].stepsRemaining <= 0) {
        axis[i].moving = false;
        axis[i].currentSteps = axis[i].targetSteps; // snap to exact target
      }
    }
  }

  if (!anyMoving) {
    moveInProgress = false;
    Serial.println("DONE");
  }
}

void stopMove() {
  for (int i = 0; i < 3; i++) {
    axis[i].moving = false;
  }
  moveInProgress = false;
  Serial.println("OK");
}

// ---------------- HOMING ----------------
// Simple example: drive each arm upward (toward the motor) until its
// limit switch triggers, then define that as step position 0.
// Adjust HOMING_DIR / HOMING_SPEED_US for your machine.
const int HOMING_DIR = 1;
const unsigned long HOMING_STEP_INTERVAL_US = 1500; // slower for safety

bool readLimit(int pin) {
  int val = digitalRead(pin);
  return LIMIT_ACTIVE_LOW ? (val == LOW) : (val == HIGH);
}

void homeAxis(int i, int stepPinDir) {
  bool triggered = false;
  int limitPin = (i == 0) ? LIMIT1_PIN : (i == 1) ? LIMIT2_PIN : LIMIT3_PIN;

  while (!triggered) {
    if (readLimit(limitPin)) {
      triggered = true;
      break;
    }
    pulseAxis(i, HOMING_DIR);
    delayMicroseconds(HOMING_STEP_INTERVAL_US);
  }
  axis[i].currentSteps = 0;
  axis[i].targetSteps = 0;
}

void runHoming() {
  setEnabled(true);
  for (int i = 0; i < 3; i++) {
    homeAxis(i, HOMING_DIR);
  }
  Serial.println("HOMED");
}

// ---------------- SERIAL COMMAND PARSER ----------------
String inputLine = "";

void handleCommand(String line) {
  line.trim();
  if (line.length() == 0) return;

  if (line.startsWith("M ")) {
    long s1, s2, s3;
    unsigned long t;
    int matched = sscanf(line.c_str(), "M %ld %ld %ld %lu", &s1, &s2, &s3, &t);
    if (matched == 4) {
      startMove(s1, s2, s3, t);
      Serial.println("OK");
    } else {
      Serial.println("ERR bad M command format");
    }

  } else if (line == "H") {
    runHoming();

  } else if (line.startsWith("EN")) {
    int val = 0;
    sscanf(line.c_str(), "EN %d", &val);
    setEnabled(val != 0);
    Serial.println("OK");

  } else if (line == "STOP") {
    stopMove();

  } else if (line == "P?") {
    Serial.print("POS ");
    Serial.print(axis[0].currentSteps); Serial.print(" ");
    Serial.print(axis[1].currentSteps); Serial.print(" ");
    Serial.println(axis[2].currentSteps);

  } else {
    Serial.println("ERR unknown command");
  }
}

// ---------------- ARDUINO ENTRY POINTS ----------------
void setup() {
  Serial.begin(115200);

  pinMode(M1_STEP_PIN, OUTPUT);
  pinMode(M1_DIR_PIN, OUTPUT);
  pinMode(M2_STEP_PIN, OUTPUT);
  pinMode(M2_DIR_PIN, OUTPUT);
  pinMode(M3_CW_PIN, OUTPUT);
  pinMode(M3_CCW_PIN, OUTPUT);
  pinMode(EN_PIN, OUTPUT);

  pinMode(LIMIT1_PIN, INPUT_PULLUP);
  pinMode(LIMIT2_PIN, INPUT_PULLUP);
  pinMode(LIMIT3_PIN, INPUT_PULLUP);

  setEnabled(false); // start disabled for safety until commanded

  Serial.println("READY");
}

void loop() {
  // Non-blocking serial line reading
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\n') {
      handleCommand(inputLine);
      inputLine = "";
    } else if (c != '\r') {
      inputLine += c;
    }
  }

  // Non-blocking multi-axis step generation
  serviceMove();
}
