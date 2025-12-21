#include "core/Input.h"

namespace {
  bool  g_keys[Input::KEY_COUNT] = {false};

  bool  g_mouseHaveLast = false;
  double g_lastX = 0.0;
  double g_lastY = 0.0;

  float g_dx = 0.0f;
  float g_dy = 0.0f;
}

void Input::init() { resetAll(); }
void Input::shutdown() {}

void Input::resetAll() {
  for (int i = 0; i < KEY_COUNT; ++i) g_keys[i] = false;

  g_mouseHaveLast = false;
  g_lastX = g_lastY = 0.0;
  g_dx = g_dy = 0.0f;
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
  if (!g_mouseHaveLast) {
    g_lastX = x;
    g_lastY = y;
    g_mouseHaveLast = true;
    return;
  }

  // DELTAS = current - last (this is the “out of the box” FPS approach)
  g_dx += static_cast<float>(x - g_lastX);
  g_dy += static_cast<float>(y - g_lastY);

  g_lastX = x;
  g_lastY = y;
}

void Input::getMouseDelta(float& dx, float& dy) {
  dx = g_dx;
  dy = g_dy;
  g_dx = g_dy = 0.0f;
}
