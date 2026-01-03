#include "game/GameObject.h"

void GameObject::update(float /*dt*/) {
  // Gameplay logic can go here later
}

void GameObject::enableBoxCollider(const Vec3& halfExtents) {
  m_hasBoxCollider = true;
  m_boxHalfExtents = halfExtents;
}

void GameObject::disableBoxCollider() {
  m_hasBoxCollider = false;
}
