#pragma once

class Input {
public:
  // Initialize / shutdown key state
  static void init();
  static void shutdown();

  // Query key state (ASCII-based; 'W','A','S','D', ' ' etc.)
  static bool isKeyDown(int key);

  // Platform adapter (GLFW callback will call this)
  static void setKey(int key, bool down);
};
