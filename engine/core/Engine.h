// engine/core/Engine.h
#pragma once
#include <memory>

class Game;
class Scene;

class Engine {
public:
  static Engine& instance();

  void init();
  void update();
  void render();
  void stepOnce();  // convenience for main loop (update + render)
  void shutdown();

private:
  Engine() = default;

  bool initialized = false;
  std::unique_ptr<Game>  game;
  std::unique_ptr<Scene> scene;
};
