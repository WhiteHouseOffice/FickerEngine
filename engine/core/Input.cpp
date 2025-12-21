#include "core/Input.h"

namespace {
  bool  g_keys[Input::KEY_COUNT] = {false};
  float g_dx = 0.0f;
  float g_dy = 0.0f;
}

void Input::init() { resetAll(); }
void Input::shutdown() {}

void Input::resetAll() {
  for (int i = 0; i < KEY_COUNT; ++i) g_keys[i] = false;
  g_dx = g_dy = 0.0f;
}

bool Input::isKeyDown(Key key) {
  if (key < 0 || key >= KEY_COUNT) return false;
  return g_keys[key];
}

void Input::setKey(Key key, bool down) {
  if (key < 0 || key >= KEY_COUNT) return;
  g_keys[key] = down;
}

void Input::addMouseDelta(float dx, float dy) {
  g_dx += dx;
  g_dy += dy;
}

void Input::getMouseDelta(float& dx, float& dy) {
  dx = g_dx;
  dy = g_dy;
  g_dx = g_dy = 0.0f;
}
