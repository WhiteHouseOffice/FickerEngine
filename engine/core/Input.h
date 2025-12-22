#pragma once

class Input {
public:
  enum Key {
    KEY_W = 0,
    KEY_A,
    KEY_S,
    KEY_D,
    KEY_SPACE,
    KEY_CTRL,
    KEY_SHIFT,
    KEY_F,
    KEY_COUNT
  };

  static void init();
  static void beginFrame();

  static void setKey(Key k, bool down);
  static bool isKeyDown(Key k);

  // Mouse deltas (accumulated by platform layer)
  static void addMouseDelta(float dx, float dy);
  static void getMouseDelta(float& dx, float& dy);
  static void resetMouse();

  static void resetAll();
};
