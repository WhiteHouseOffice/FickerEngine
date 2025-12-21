#pragma once

class Input {
public:
  enum Key {
    KEY_W,
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
  static void shutdown();

  static void resetAll();

  static void setKey(Key key, bool down);
  static bool isKeyDown(Key key);

  // Mouse (relative deltas)
  static void onMouseMove(double x, double y);
  static void getMouseDelta(float& dx, float& dy);

  // NEW: reset just mouse tracking (keep keys intact if you want)
  static void resetMouse();
};
