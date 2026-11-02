#include "Braccio++.h"
SerialMock Serial;
BraccioMock Braccio;
#include "../../firmware/braccio_serial/braccio_serial.ino"
#include <cassert>
#include <iostream>

static std::string run(const std::string &line) {
  Serial.in = line; Serial.pos = 0; Serial.out.clear();
  loop();
  return Serial.out;
}

int main() {
  setup();
  size_t base = Braccio.moves.size();             // HOME in setup
  assert(run("MOVE 160 150 157.5 157.5 157.5 95\n") == "OK: MOVE\n");
  assert(Braccio.moves.size() == base + 1 && Braccio.moves.back().a[2] == 157.5f);
  assert(run("move 300 150 157.5 157.5 157.5 -10\n") == "OK: MOVE (clamped)\n");
  assert(Braccio.moves.back().a[0] == 230.0f && Braccio.moves.back().a[5] == 0.0f);
  size_t n = Braccio.moves.size();
  assert(run("MOVE 160 abc 157.5 157.5 157.5 95\n") == "ERROR: MOVE argument 2 is not a number\n");
  assert(run("MOVE 160 150 157.5\n").rfind("ERROR: MOVE syntax", 0) == 0);
  assert(run("MOVE 1 2 3 4 5 6 7\n").rfind("ERROR: MOVE syntax", 0) == 0);
  assert(run("MOVE 1.2.3 150 157.5 157.5 157.5 95\n").rfind("ERROR", 0) == 0);
  assert(Braccio.moves.size() == n);              // nothing moved on errors
  assert(run("GRIP 200\n") == "OK: GRIP -> 200.00\n" && Braccio.motors[1].last == 200.0f);
  assert(run("GRIP 50\n") == "OK: GRIP -> 120.00 (clamped)\n");
  assert(run("GRIPPER 150\n") == "ERROR: Unknown command\n");
  assert(run("GRIP\n").rfind("ERROR: GRIP syntax", 0) == 0);
  assert(run("  home  \r\n") == "OK: HOME\n");
  assert(run("HOME now\n").rfind("ERROR", 0) == 0);
  assert(run(std::string(200, 'x') + "\n") == "ERROR: line too long\n");
  assert(run("HOME\n") == "OK: HOME\n");          // recovers after overflow
  assert(run("MOVE 160 150 157.5 157.5\n  ").rfind("ERROR", 0) == 0);
  std::cout << "firmware parser: all tests passed\n";
  return 0;
}
