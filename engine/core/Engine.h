#pragma once

// Minimal core engine singleton interface used by runtime/main.cpp

class Engine {
public:
  // Singleton access (runtime/main.cpp calls Engine::instance())
  static Engine& instance();

  // Lifecycle
  void init();
  void update();
  void render();
  void shutdown();

  // One tick used by the browser loop
  void stepOnce();

private:
  Engine()  = default;
  ~Engine() = default;

  Engine(const Engine&)            = delete;
  Engine& operator=(const Engine&) = delete;
};
