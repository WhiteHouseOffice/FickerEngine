#pragma once

#include "game/Game.h"

class Engine {
public:
  // Global singleton instance
  static Engine& instance();

  // Called once from main/runtime
  void init();

  // Called once per frame from the main loop
  void stepOnce();

private:
  Engine();
  ~Engine();

  Engine(const Engine&) = delete;
  Engine& operator=(const Engine&) = delete;

  Game game;
};
