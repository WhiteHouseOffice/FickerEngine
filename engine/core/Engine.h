#pragma once
#include <memory>

class Game;
class Scene;

class Engine {
public:
  static Engine& instance();

  void stepOnce();   // called from main loop

  // Lifecycle – must be public because runtime/main.cpp calls them
  void init();
  void update();
  void render();
  void shutdown();

private:
  Engine()  = default;
  ~Engine() = default;

  Engine(const Engine&)            = delete;
  Engine& operator=(const Engine&) = delete;

  bool initialized = false;

  std::unique_ptr<Game>  game;
  std::unique_ptr<Scene> scene;
};
