#include "core/Input.h"

namespace {
  // Simple key state store (ASCII + small range)
  bool g_keys[512] = {false};
}

void Input::init() {
  for (bool &k : g_keys) k = false;
}

void Input::shutdown() {
  // nothing for now
}

bool Input::isKeyDown(int key) {
  if (key < 0 || key >= 512) return false;
  return g_keys[key];
}

void Input::setKey(int key, bool down) {
  if (key < 0 || key >= 512) return;

  g_keys[key] = down;

  // Mirror letter case so 'w' and 'W' behave the same.
  if (key >= 'A' && key <= 'Z') {
    const int lower = key - 'A' + 'a';
    if (lower >= 0 && lower < 512) g_keys[lower] = down;
  } else if (key >= 'a' && key <= 'z') {
    const int upper = key - 'a' + 'A';
    if (upper >= 0 && upper < 512) g_keys[upper] = down;
  }
}
