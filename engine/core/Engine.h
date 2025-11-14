#pragma once

#include <memory>

class Scene;
class Game;

class Engine {
public:
  // Singleton access used from runtime/main.cpp
  static Engine& instance();

  // Called each frame from the platform layer (emscripten main loop)
  void stepOnce();

private:
  Engine()  = default;
  ~Engine() = default;

  Engine(const Engine&)            = delete;
  Engine& operator=(const Engine&) = delete;

  // Internal lifecycle
  void init();
  void update();
  void render();
  void shutdown();

  bool initialized = false;

  std::unique_ptr<Scene> scene;
  std::unique_ptr<Game>  game;
};
