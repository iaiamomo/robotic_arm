/*
 * Arduino Braccio++ serial command firmware.
 * Based on A. M. Capparelli, MSc thesis (Sapienza), Listing 3.1.
 *
 * Protocol (115200 baud, one command per line, case-insensitive):
 *   HOME                     -> home configuration
 *   GRIP <g>                 -> gripper angle
 *   MOVE <g> <r> <p> <e> <s> <b>
 *                            -> gripper, wrist roll, wrist pitch, elbow, shoulder, base
 * Replies: "OK: ..." (with " (clamped)" if a value was limited) or "ERROR: ...".
 *
 * Changes w.r.t. the thesis listing:
 *  - numeric arguments are validated: String::toFloat() returns 0 on garbage, which
 *    the original then clamped to the joint minimum and executed;
 *  - exact token counts (GRIP takes 1 value, MOVE exactly 6; "GRIPPER 150" is rejected);
 *  - bounded input buffer;
 *  - clamping is reported back to the host.
 */
#include <Braccio++.h>

/* Joint and gripper operating ranges (mechanical limits, degrees) */
static const float GRIP_MIN = 120.0, GRIP_MAX = 230.0;
static const float ROLL_MIN = 0.0, ROLL_MAX = 315.0;
static const float PITCH_MIN = 80.0, PITCH_MAX = 230.0;
static const float ELBOW_MIN = 75.0, ELBOW_MAX = 225.0;
static const float SHOULDER_MIN = 60.0, SHOULDER_MAX = 250.0;
static const float BASE_MIN = 0.0, BASE_MAX = 315.0;

static const float LIMITS[6][2] = {
  {GRIP_MIN, GRIP_MAX}, {ROLL_MIN, ROLL_MAX}, {PITCH_MIN, PITCH_MAX},
  {ELBOW_MIN, ELBOW_MAX}, {SHOULDER_MIN, SHOULDER_MAX}, {BASE_MIN, BASE_MAX},
};

/* Default home configuration: grip, roll, pitch, elbow, shoulder, base */
static const float HOME_POS[6] = {160.0, 150.0, 157.5, 157.5, 157.5, 95.0};

static const unsigned int MAX_LINE = 96;
static const int MAX_TOKENS = 8;

static auto gripper = Braccio.get(1);

String inputLine;
bool lineOverflow = false;

void goHome() {
  Braccio.moveTo(HOME_POS[0], HOME_POS[1], HOME_POS[2], HOME_POS[3], HOME_POS[4], HOME_POS[5]);
}

/* Strict decimal parser: [+-]digits[.digits] — rejects empty, garbage, "1.2.3". */
bool parseNumber(const String &tok, float &out) {
  unsigned int i = 0, n = tok.length();
  if (n == 0) return false;
  if (tok[0] == '+' || tok[0] == '-') i = 1;
  bool digits = false, dot = false;
  for (; i < n; i++) {
    char c = tok[i];
    if (c >= '0' && c <= '9') digits = true;
    else if (c == '.' && !dot) dot = true;
    else return false;
  }
  if (!digits) return false;
  out = tok.toFloat();
  return true;
}

/* Split on spaces/tabs; returns the number of tokens (MAX_TOKENS+1 if there are too many). */
int tokenize(const String &line, String tokens[]) {
  int count = 0;
  unsigned int i = 0, n = line.length();
  while (i < n) {
    while (i < n && (line[i] == ' ' || line[i] == '\t')) i++;
    if (i >= n) break;
    unsigned int start = i;
    while (i < n && line[i] != ' ' && line[i] != '\t') i++;
    if (count == MAX_TOKENS) return MAX_TOKENS + 1;
    tokens[count++] = line.substring(start, i);
  }
  return count;
}

bool clampValue(float &v, int joint) {
  float c = constrain(v, LIMITS[joint][0], LIMITS[joint][1]);
  bool changed = (c != v);
  v = c;
  return changed;
}

void handleCommand(const String &line) {
  String tok[MAX_TOKENS];
  int n = tokenize(line, tok);
  if (n == 0) return;
  String cmd = tok[0];
  cmd.toUpperCase();

  if (cmd == "HOME") {
    if (n != 1) { Serial.println("ERROR: HOME takes no arguments"); return; }
    goHome();
    Serial.println("OK: HOME");
    return;
  }

  if (cmd == "GRIP") {
    float g;
    if (n != 2 || !parseNumber(tok[1], g)) {
      Serial.println("ERROR: GRIP syntax. Expected: GRIP <angle>");
      return;
    }
    bool clamped = clampValue(g, 0);
    gripper.move().to(g);
    Serial.print("OK: GRIP -> ");
    Serial.print(g);
    Serial.println(clamped ? " (clamped)" : "");
    return;
  }

  if (cmd == "MOVE") {
    float a[6];
    if (n != 7) { Serial.println("ERROR: MOVE syntax. Expected: MOVE g r p e s b"); return; }
    for (int k = 0; k < 6; k++) {
      if (!parseNumber(tok[k + 1], a[k])) {
        Serial.print("ERROR: MOVE argument ");
        Serial.print(k + 1);
        Serial.println(" is not a number");
        return;
      }
    }
    bool clamped = false;
    for (int k = 0; k < 6; k++) clamped |= clampValue(a[k], k);
    Braccio.moveTo(a[0], a[1], a[2], a[3], a[4], a[5]);
    Serial.println(clamped ? "OK: MOVE (clamped)" : "OK: MOVE");
    return;
  }

  Serial.println("ERROR: Unknown command");
}

void setup() {
  Serial.begin(115200);
  while (!Serial);
  inputLine.reserve(MAX_LINE);

  if (!Braccio.begin()) {
    Serial.println("ERROR: Braccio.begin() failed!");
    while (true) {}
  }
  goHome();
  Serial.println("Braccio++ ready (Serial Control Mode active).");
  Serial.println("Commands: HOME | GRIP X | MOVE g r p e s b");
}

void loop() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (lineOverflow) {
        Serial.println("ERROR: line too long");
      } else if (inputLine.length() > 0) {
        handleCommand(inputLine);
      }
      inputLine = "";
      lineOverflow = false;
    } else if (inputLine.length() < MAX_LINE) {
      inputLine += c;
    } else {
      lineOverflow = true;
    }
  }
}
