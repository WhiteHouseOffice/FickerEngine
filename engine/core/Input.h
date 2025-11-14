#pragma once

namespace Input {

enum Key {
  KEY_W,
  KEY_A,
  KEY_S,
  KEY_D,
  KEY_SHIFT
};

// TODO: hook this up to real browser key events.
// For now, always return false so the game compiles and runs.
inline bool isKeyDown(Key) {
  return false;
}

} // namespace Input
