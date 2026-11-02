// Host-side stand-ins for the Arduino core and the Braccio++ library, used only to
// unit-test the command parser of firmware/braccio_serial on a PC (see run_tests.sh).
#pragma once
#include <cstdio>
#include <cstdlib>
#include <string>
#include <cctype>
#include <vector>

class String {
 public:
  std::string s;
  String() {}
  String(const char *c) : s(c) {}
  String(const std::string &x) : s(x) {}
  unsigned int length() const { return s.size(); }
  char operator[](unsigned int i) const { return s[i]; }
  String substring(unsigned int a) const { return String(s.substr(a)); }
  String substring(unsigned int a, unsigned int b) const { return String(s.substr(a, b - a)); }
  void toUpperCase() { for (auto &c : s) c = std::toupper(c); }
  float toFloat() const { return std::atof(s.c_str()); }
  bool operator==(const char *o) const { return s == o; }
  String &operator+=(char c) { s += c; return *this; }
  String &operator=(const char *c) { s = c; return *this; }
  void reserve(unsigned int) {}
};

template <class T> T constrain(T v, T lo, T hi) { return v < lo ? lo : (v > hi ? hi : v); }

struct SerialMock {
  std::string in; size_t pos = 0; std::string out;
  void begin(long) {}
  explicit operator bool() const { return true; }
  int available() { return pos < in.size(); }
  char read() { return in[pos++]; }
  void print(const char *x) { out += x; }
  void print(int x) { out += std::to_string(x); }
  void print(float x) { char b[32]; std::snprintf(b, sizeof b, "%.2f", x); out += b; }
  void println(const char *x = "") { out += x; out += "\n"; }
};
extern SerialMock Serial;

struct MoveCall { float a[6]; };
struct Motor { float last = -1; };
// Like the real library, get() returns a small handle by value.
struct MotorHandle {
  Motor *m;
  struct M { Motor *m; void to(float g) { m->last = g; } };
  M move() { return M{m}; }
};
struct BraccioMock {
  std::vector<MoveCall> moves; Motor motors[7];
  bool begin() { return true; }
  MotorHandle get(int i) { return MotorHandle{&motors[i]}; }
  void moveTo(float a, float b, float c, float d, float e, float f) { moves.push_back({{a, b, c, d, e, f}}); }
};
extern BraccioMock Braccio;
