#pragma once
#include <memory>

class Engine {
public:
  static Engine& instance();

  void init();
  // Advance the engine by one variable frame (will internally run 0..N fixed updates)
  void stepOnce();

  // Simple readback of state for debug/UI
  float angle() const;

  ~Engine();

private:
  Engine();
  Engine(const Engine&) = delete;
  Engine& operator=(const Engine&) = delete;

  struct Impl;
  std::unique_ptr<Impl> impl;
};
