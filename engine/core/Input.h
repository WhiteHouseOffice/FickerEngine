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

  static bool isKeyDown(Key key);
  static void getMouseDelta(float& dx, float& dy);

  static void setKey(Key key, bool down);

  // Called by platform layer (Main.cpp)
  static void addMouseDelta(float dx, float dy);

  static void resetAll();
};
