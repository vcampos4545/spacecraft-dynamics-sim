#pragma once

#include <rigidbody/ForceGenerator.h>
#include <glm/glm.hpp>

class RigidBody;

// A uniform constant-acceleration field (F = acceleration * body.mass),
// same everywhere and in every direction regardless of body position --
// e.g. "straight down" for a ground-based/atmospheric scenario. Applied to
// every body -- this is what PhysicsWorld used to do internally via a
// built-in `gravity` field. Now it's an ordinary global force generator:
// attach it via PhysicsWorld::addGlobalForceGenerator() if a scenario wants
// gravity at all, the same way a body opts into a Thruster or
// ReactionWheel. `acceleration` is public and readable after attaching
// (keep the raw pointer before the ownership-transferring move), for
// anything that needs the ambient field back -- e.g. IMU::sample()'s
// gravity parameter, to turn coordinate acceleration into specific force.
//
// See rigidbody/environment/central_body/CentralBodyGravity.h for the
// position-dependent inverse-square counterpart (orbital scenarios).
class UniformGravity : public ForceGenerator
{
public:
  glm::vec3 acceleration; // m/s^2

  explicit UniformGravity(glm::vec3 accelerationMS2) : acceleration(accelerationMS2) {}

  void apply(RigidBody &body, float dt) override;
};
