#include "core/Input.h"
#include <cstring>

namespace {
  bool  g_keys[Input::KEY_COUNT];
  float g_dx = 0.0f;
  float g_dy = 0.0f;
}

void Input::init() {
  std::memset(g_keys, 0, sizeof(g_keys));
  g_dx = g_dy = 0.0f;
}

void Input::beginFrame() {
  // If later you want “pressed this frame” vs “held”, do it here.
}

void Input::setKey(Key k, bool down) {
  if (k >= 0 && k < KEY_COUNT) g_keys[k] = down;
}

bool Input::isKeyDown(Key k) {
  if (k >= 0 && k < KEY_COUNT) return g_keys[k];
  return false;
}

void Input::addMouseDelta(float dx, float dy) {
  g_dx += dx;
  g_dy += dy;
}

void Input::getMouseDelta(float& dx, float& dy) {
  dx = g_dx;
  dy = g_dy;
  g_dx = 0.0f;
  g_dy = 0.0f;
}

void Input::resetMouse() {
  g_dx = 0.0f;
  g_dy = 0.0f;
}

void Input::resetAll() {
  std::memset(g_keys, 0, sizeof(g_keys));
  resetMouse();
}
