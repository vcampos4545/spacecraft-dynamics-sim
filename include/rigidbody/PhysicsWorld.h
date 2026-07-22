#pragma once
#include <rigidbody/RigidBody.h>
#include <rigidbody/Constraint.h>
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

  // Removing a body also removes any constraints that reference it (a
  // constraint left pointing at a freed body would dangle).
  void removeBody(RigidBody *body);
  void clear();

  // Constraint management — see Constraint.h for what each removes/allows.
  FixedConstraint  *addFixedConstraint(RigidBody *a, RigidBody *b, const glm::vec3 &worldPivot);
  PointConstraint  *addPointConstraint(RigidBody *a, RigidBody *b, const glm::vec3 &worldPivot);
  HingeConstraint  *addHingeConstraint(RigidBody *a, RigidBody *b,
                                       const glm::vec3 &worldPivot,
                                       const glm::vec3 &worldAxis);
  SliderConstraint *addSliderConstraint(RigidBody *a, RigidBody *b,
                                        const glm::vec3 &worldPivot,
                                        const glm::vec3 &worldAxis);
  DistanceConstraint *addDistanceConstraint(RigidBody *a, RigidBody *b,
                                            const glm::vec3 &worldPivotA,
                                            const glm::vec3 &worldPivotB,
                                            float restLength,
                                            bool unilateral = false);

  void removeConstraint(Constraint *constraint);

  // Simulation
  void step(float dt);

  // Accessors
  const std::vector<std::unique_ptr<RigidBody>> &getBodies() const { return bodies; }
  size_t getBodyCount() const { return bodies.size(); }

private:
  std::vector<std::unique_ptr<RigidBody>> bodies;
  std::vector<std::unique_ptr<Constraint>> constraints;
  float accumulator = 0.0f;

  void stepFixed(float dt);
  void integrateAll(float dt);
  void solveConstraints(float dt);
  void detectAndResolveCollisions();

  // Body-to-body collision
  void resolveBodyBodyCollision(RigidBody &a, RigidBody &b);
  void resolveSphereSphereCollision(RigidBody &a, RigidBody &b);
  void resolveSphereBoxCollision(RigidBody &sphere, RigidBody &box);
  void resolveBoxBoxCollision(RigidBody &a, RigidBody &b);

  // Ground collision (z = 0 plane)
  void resolveGroundCollisions();
};
