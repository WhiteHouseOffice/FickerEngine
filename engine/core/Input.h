#pragma once

namespace Input {

enum Key {
  KEY_W = 0,
  KEY_A,
  KEY_S,
  KEY_D,
  KEY_SHIFT,
  KEY_COUNT
};

// Call once at startup to hook into keyboard events (on web builds).
void init();

// Query current key state.
bool isKeyDown(Key key);

} // namespace Input
