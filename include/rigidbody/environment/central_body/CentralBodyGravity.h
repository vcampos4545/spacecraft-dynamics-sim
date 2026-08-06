#pragma once

#include <rigidbody/ForceGenerator.h>
#include <glm/glm.hpp>

class RigidBody;

// Inverse-square gravity toward a single point mass: F = -mu * m * r_hat /
// |r|^2, where r = body.position - centralBodyPositionWorld. Unlike
// UniformGravity (same acceleration everywhere), this varies with the
// body's actual position -- give a body a circular-velocity initial
// condition at some radius and it will actually orbit, under this
// generator's own force alone, no separate orbit propagation needed.
//
// `centralBodyPositionWorld` and `mu` (standard gravitational parameter,
// GM) are both public and default to Earth at the world origin, but
// neither is hardcoded -- a lunar or Martian scenario just sets `mu` to
// that body's own GM. Registered globally via
// PhysicsWorld::addGlobalForceGenerator(), same as UniformGravity.
//
// Precision note: this operates on RigidBody's float32 position, which is
// adequate for demos, short-duration scenarios, and validating local
// effects (e.g. gravity-gradient torque) -- but a LEO-radius position
// (~6.9e6 m) only leaves float32 with meter-level precision, and that
// error compounds every physics step. For a mission-duration "truth"
// trajectory (weeks to years), use the double-precision rigidbody/orbit/
// module instead and bridge its result into a RigidBody's position each
// frame for local dynamics/rendering, rather than integrating the truth
// trajectory through this generator directly.
class CentralBodyGravity : public ForceGenerator
{
public:
  glm::vec3 centralBodyPositionWorld{0.0f};
  float mu = 3.986004418e14f; // m^3/s^2, Earth's standard gravitational parameter (GM)

  void apply(RigidBody &body, float dt) override;
};
