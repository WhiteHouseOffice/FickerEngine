class Game {
public:
  void init();
  void update(float dt);
  Mat4 view() const;

private:
  Vec3  camPos{0.f, 2.f, 5.f};
  float yaw{0.f};
  float logTimer{0.f};
};
