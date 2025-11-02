
#pragma once
struct IRenderer {
  virtual ~IRenderer() = default;
  virtual bool init(int w, int h) = 0;
  virtual void beginFrame() = 0;
  virtual void drawTestScene(double t) = 0;
  virtual void endFrame() = 0;
};
