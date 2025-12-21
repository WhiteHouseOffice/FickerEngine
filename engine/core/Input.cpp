#include "core/Input.h"

namespace {
  bool g_keys[Input::KEY_COUNT] = {false};

  // Mouse delta accumulator
  float  g_dx = 0.0f;
  float  g_dy = 0.0f;

  // Last absolute mouse position (used to compute deltas)
  bool   g_haveLast = false;
  double g_lastX = 0.0;
  double g_lastY = 0.0;
}

void Input::init() {
  resetAll();
}

void Input::shutdown() {}

void Input::resetMouse() {
  g_haveLast = false;
  g_lastX = 0.0;
  g_lastY = 0.0;
  g_dx = 0.0f;
  g_dy = 0.0f;
}

void Input::resetAll() {
  for (int i = 0; i < KEY_COUNT; ++i) g_keys[i] = false;
  resetMouse();
}

void Input::setKey(Key key, bool down) {
  if (key < 0 || key >= KEY_COUNT) return;
  g_keys[key] = down;
}

bool Input::isKeyDown(Key key) {
  if (key < 0 || key >= KEY_COUNT) return false;
  return g_keys[key];
}

void Input::onMouseMove(double x, double y) {
  if (!g_haveLast) {
    g_lastX = x;
    g_lastY = y;
    g_haveLast = true;
    return;
  }

  g_dx += static_cast<float>(x - g_lastX);
  g_dy += static_cast<float>(y - g_lastY);

  g_lastX = x;
  g_lastY = y;
}

void Input::getMouseDelta(float& dx, float& dy) {
  dx = g_dx;
  dy = g_dy;
  g_dx = 0.0f;
  g_dy = 0.0f;
}
