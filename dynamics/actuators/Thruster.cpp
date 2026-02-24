#include "Thruster.h"
#include "RigidBody.h"

Thruster::Thruster(
    glm::vec3 mountPosBody,
    glm::vec3 directionBody,
    float maxThrustForce)
    : mountPositionBody(mountPosBody),
      nominalDirectionBody(glm::normalize(directionBody)),
      maxThrust(maxThrustForce)
{
}

void Thruster::apply(RigidBody &body, float throttle)
{
  if (throttle <= 0.0f)
    return;

  // Clamp gimbal
  pitch = glm::clamp(pitch, -gimbalLimit, gimbalLimit);
  yaw = glm::clamp(yaw, -gimbalLimit, gimbalLimit);

  // Build gimbal rotation in body frame
  glm::quat gimbalRot =
      glm::angleAxis(yaw, glm::vec3(0, 1, 0)) *
      glm::angleAxis(pitch, glm::vec3(1, 0, 0));

  glm::vec3 thrustDirBody =
      gimbalRot * nominalDirectionBody;

  glm::vec3 thrustBody = thrustDirBody * maxThrust * throttle;

  // Convert to world frame
  glm::vec3 thrustWorld =
      body.orientation * thrustBody;

  glm::vec3 mountWorld =
      body.position + body.orientation * mountPositionBody;

  body.applyForceAtPoint(thrustWorld, mountWorld);
}

glm::vec3 Thruster::getWorldMountPosition(const RigidBody &body) const
{
  return body.position + body.orientation * mountPositionBody;
}

glm::vec3 Thruster::getWorldThrustDirection(const RigidBody &body) const
{
  glm::quat gimbalRot =
      glm::angleAxis(yaw, glm::vec3(0, 1, 0)) *
      glm::angleAxis(pitch, glm::vec3(1, 0, 0));

  glm::vec3 dirBody =
      gimbalRot * nominalDirectionBody;

  return glm::normalize(body.orientation * dirBody);
}
