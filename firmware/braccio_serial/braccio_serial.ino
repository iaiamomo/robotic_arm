/*
 * Arduino Braccio++ Firmware for Serial Command Handling
 * Source: A. M. Capparelli, Master's thesis (Sapienza), Listing 3.1
 *
 * Serial protocol (115200 baud, newline-terminated):
 *   HOME                 -> move to home configuration
 *   GRIP <g>             -> set gripper angle (clamped to [GRIP_MIN, GRIP_MAX])
 *   MOVE g r p e s b     -> gripper, wrist roll, wrist pitch, elbow, shoulder, base
 */
#include <Braccio++.h>

/* Joint and Gripper Operating Ranges (Mechanical Limits) */
static const float GRIP_MIN     = 120.0;
static const float GRIP_MAX     = 230.0;

static const float ROLL_MIN     = 0.0;
static const float ROLL_MAX     = 315.0;

static const float PITCH_MIN    = 80.0;
static const float PITCH_MAX    = 230.0;

static const float ELBOW_MIN    = 75.0;
static const float ELBOW_MAX    = 225.0;

static const float SHOULDER_MIN = 60.0;
static const float SHOULDER_MAX = 250.0;

static const float BASE_MIN     = 0.0;
static const float BASE_MAX     = 315.0;

/* Default Home Configuration */
static const float HOME_POS[6] = {
  160.0,  // GRIP
  150.0,  // ROLL
  157.5,  // PITCH
  157.5,  // ELBOW
  157.5,  // SHOULDER
  95.0,   // BASE
};

/* Motor Object Initialization */
static auto gripper    = Braccio.get(1);
static auto wristRoll  = Braccio.get(2);
static auto wristPitch = Braccio.get(3);
static auto elbow      = Braccio.get(4);
static auto shoulder   = Braccio.get(5);
static auto base       = Braccio.get(6);

String inputLine;

/** Moves the manipulator to the predefined Home position */
void goHome() {
  Braccio.moveTo(
    HOME_POS[0], HOME_POS[1], HOME_POS[2],
    HOME_POS[3], HOME_POS[4], HOME_POS[5]
  );
}

void setup() {
  Serial.begin(115200);  // High-speed serial communication
  while (!Serial);

  if (!Braccio.begin()) {
    Serial.println("ERROR: Braccio.begin() failed!");
    while (true) {}  // Halt execution on hardware failure
  }

  goHome();
  Serial.println("Braccio++ ready (Serial Control Mode active).");
  Serial.println("Commands: HOME | GRIP X | MOVE g r p e s b");
}

void loop() {
  /* Serial Stream Polling */
  while (Serial.available()) {
    char c = Serial.read();

    if (c == '\n' || c == '\r') {
      if (inputLine.length() > 0) {
        handleCommand(inputLine);
        inputLine = "";  // Reset buffer
      }
    } else {
      inputLine += c;  // Accumulate characters
    }
  }
}

/** Command Parser and Execution Dispatcher */
void handleCommand(String cmd) {
  cmd.trim();
  String upper = cmd;
  upper.toUpperCase();

  // HOME Command execution
  if (upper == "HOME") {
    goHome();
    Serial.println("OK: HOME");
    return;
  }

  // GRIP Command execution (End-effector control)
  if (upper.startsWith("GRIP")) {
    float g = cmd.substring(4).toFloat();
    g = constrain(g, GRIP_MIN, GRIP_MAX);
    gripper.move().to(g);
    Serial.print("OK: GRIP -> "); Serial.println(g);
    return;
  }

  // MOVE Command execution (Multi-joint coordination)
  if (upper.startsWith("MOVE")) {
    float a[6];
    if (!parseMove(cmd, a)) {
      Serial.println("ERROR: MOVE syntax error. Expected: MOVE g r p e s b");
      return;
    }
    applyLimits(a);  // Safety software constraints
    Braccio.moveTo(a[0], a[1], a[2], a[3], a[4], a[5]);
    Serial.println("OK: MOVE");
    return;
  }

  Serial.println("ERROR: Unknown command");
}

/** Parses a MOVE string into an array of float coordinates */
bool parseMove(String cmd, float out[6]) {
  cmd = cmd.substring(4);
  cmd.trim();

  int idx = 0;
  int start = 0;

  while (idx < 6) {
    int space = cmd.indexOf(' ', start);
    String token;

    if (space == -1) token = cmd.substring(start);
    else token = cmd.substring(start, space);

    token.trim();
    if (token.length() == 0) return false;

    out[idx++] = token.toFloat();
    if (space == -1) break;
    start = space + 1;
  }

  return idx == 6;
}

/** Enforces hardware safety limits on incoming joint angles */
void applyLimits(float a[6]) {
  a[0] = constrain(a[0], GRIP_MIN, GRIP_MAX);
  a[1] = constrain(a[1], ROLL_MIN, ROLL_MAX);
  a[2] = constrain(a[2], PITCH_MIN, PITCH_MAX);
  a[3] = constrain(a[3], ELBOW_MIN, ELBOW_MAX);
  a[4] = constrain(a[4], SHOULDER_MIN, SHOULDER_MAX);
  a[5] = constrain(a[5], BASE_MIN, BASE_MAX);
}
