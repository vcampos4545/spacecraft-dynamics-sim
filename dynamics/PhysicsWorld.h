#pragma once
#include "RigidBody.h"
#include <vector>
#include <memory>

class PhysicsWorld
{
public:
  float fixedTimestep = 1.0f / 120.0f; // 120 Hz physics
  glm::vec3 gravity   = {0.0f, 0.0f, 0.0f}; // world gravity (m/s^2); set before stepping

  PhysicsWorld() = default;
  ~PhysicsWorld();

  // Body management
  RigidBody *createBody(RigidBodyShape shape,
                        const glm::vec3 &size,
                        float mass);

  void removeBody(RigidBody *body);
  void clear();

  // Simulation
  void step(float dt);

  // Accessors
  const std::vector<std::unique_ptr<RigidBody>> &getBodies() const { return bodies; }
  size_t getBodyCount() const { return bodies.size(); }

private:
  std::vector<std::unique_ptr<RigidBody>> bodies;
  float accumulator = 0.0f;

  void stepFixed(float dt);
  void integrateAll(float dt);
  void detectAndResolveCollisions();

  // Body-to-body collision
  void resolveBodyBodyCollision(RigidBody &a, RigidBody &b);
  void resolveSphereSphereCollision(RigidBody &a, RigidBody &b);
  void resolveSphereBoxCollision(RigidBody &sphere, RigidBody &box);
  void resolveBoxBoxCollision(RigidBody &a, RigidBody &b);

  // Ground collision (z = 0 plane)
  void resolveGroundCollisions();
};
