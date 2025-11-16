#pragma once

namespace Input {

enum Key {
  KEY_W = 0,
  KEY_A,
  KEY_S,
  KEY_D,
  KEY_SHIFT,
  KEY_SPACE,
  KEY_CTRL,
  KEY_COUNT
};

enum MouseButton {
  MOUSE_LEFT = 0,
  MOUSE_RIGHT,
  MOUSE_MIDDLE,
  MOUSE_BUTTON_COUNT
};

// Call once at startup (Engine::init) to hook input.
void init();

// Query key / mouse button state.
bool isKeyDown(Key key);
bool isMouseButtonDown(MouseButton button);

// Get mouse movement since last call (in pixels).
// This returns the accumulated delta and resets it to zero.
void getMouseDelta(float& dx, float& dy);

} // namespace Input
