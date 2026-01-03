#include "game/GameObject.h"

void GameObject::update(float /*dt*/) {
  // Gameplay logic can go here later
}

void GameObject::enablePhysics(float mass, bool gravity) {
  m_hasRigidBody = true;
  m_rigidBody.setMass(mass);
  m_rigidBody.useGravity = gravity;
}

void GameObject::disablePhysics() {
  m_hasRigidBody = false;
}

RigidBody* GameObject::rigidBody() {
  return m_hasRigidBody ? &m_rigidBody : nullptr;
}

const RigidBody* GameObject::rigidBody() const {
  return m_hasRigidBody ? &m_rigidBody : nullptr;
}

void GameObject::setVelocity(const Vec3& v) {
  if (RigidBody* rb = rigidBody()) rb->velocity = v;
}

void GameObject::addForce(const Vec3& f) {
  if (RigidBody* rb = rigidBody()) rb->addForce(f);
}

void GameObject::enableBoxCollider(const Vec3& halfExtents) {
  m_hasBoxCollider = true;
  m_boxHalfExtents = halfExtents;
}

void GameObject::disableBoxCollider() {
  m_hasBoxCollider = false;
}
