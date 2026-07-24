#pragma once

#include <rigidbody/ForceGenerator.h>
#include <glm/glm.hpp>

class RigidBody;

class Thruster : public ForceGenerator
{
public:
  glm::vec3 mountPositionBody;    // where thruster is mounted (body frame)
  glm::vec3 nominalDirectionBody; // default thrust direction (body frame)
  float maxThrust;

  // Commanded throttle [0, 1], persistent until next set -- same convention
  // as ReactionWheel::commandedTorque. Set this, then let RigidBody's normal
  // per-substep ForceGenerator dispatch call apply(); don't call apply()
  // directly (its dt parameter is a timestep, not a throttle).
  float throttle = 0.0f;

  // gimbal angles (radians)
  float pitch = 0.0f;
  float yaw = 0.0f;

  float gimbalLimit = glm::radians(15.0f);

  Thruster(
      glm::vec3 mountPosBody,
      glm::vec3 directionBody,
      float maxThrustForce);

  void apply(RigidBody &body, float dt) override;
  glm::vec3 getWorldMountPosition(const RigidBody &body) const;
  glm::vec3 getWorldThrustDirection(const RigidBody &body) const;
};
