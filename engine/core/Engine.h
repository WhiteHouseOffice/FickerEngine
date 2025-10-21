
#pragma once
#include <memory>
#include <functional>

struct IRenderer;

struct EngineConfig {
  int width = 800;
  int height = 600;
};

class Engine {
public:
  Engine(const EngineConfig& cfg, std::unique_ptr<IRenderer> r);
  int runOnce(double dt); // returns 0 to continue, non-zero to request exit
  void draw(double t);
private:
  struct Impl; std::unique_ptr<Impl> impl;
};
