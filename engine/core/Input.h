#pragma once

class Input {
public:
  // Key codes used by the engine (Game.cpp expects these)
  enum Key {
    KEY_W,
    KEY_A,
    KEY_S,
    KEY_D,
    KEY_SPACE,
    KEY_CTRL,
    KEY_SHIFT,
    KEY_COUNT
  };

  static void init();
  static void shutdown();

  // Game.cpp API
  static bool isKeyDown(Key key);
  static void getMouseDelta(float& dx, float& dy);

  // Platform adapters (GLFW will call these)
  static void setKey(Key key, bool down);
  static void onMouseMove(double x, double y);
};
