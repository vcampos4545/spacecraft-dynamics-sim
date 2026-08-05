#include <rigidbody/actuators/Magnetorquer.h>
#include <rigidbody/RigidBody.h>
#include <glm/gtc/quaternion.hpp>

Magnetorquer::Magnetorquer(
    glm::vec3 mountPosBody,
    glm::vec3 axisBodyIn,
    float maxDipoleMomentAm2)
    : mountPositionBody(mountPosBody),
      axisBody(glm::normalize(axisBodyIn)),
      maxDipoleMoment(maxDipoleMomentAm2),
      commandedDipoleMoment(0.0f)
{
}

void Magnetorquer::commandNormalized(float command)
{
  command = glm::clamp(command, -1.0f, 1.0f);
  commandedDipoleMoment = command * maxDipoleMoment;
}

void Magnetorquer::commandDipoleMoment(float momentAm2)
{
  commandedDipoleMoment = glm::clamp(momentAm2, -maxDipoleMoment, maxDipoleMoment);
}

void Magnetorquer::apply(RigidBody &body, float /*dt*/)
{
  float effectiveMoment = commandedDipoleMoment * glm::clamp(healthFactor, 0.0f, 1.0f);

  // m x B, both in world frame: m along the rod's current world-space axis,
  // scaled by the commanded moment.
  glm::vec3 mWorld = (body.orientation * axisBody) * effectiveMoment;
  glm::vec3 torqueWorld = glm::cross(mWorld, ambientFieldWorld);

  body.applyTorque(torqueWorld);
}

glm::vec3 Magnetorquer::getWorldMountPosition(const RigidBody &body) const
{
  return body.position + body.orientation * mountPositionBody;
}

glm::vec3 Magnetorquer::getWorldAxis(const RigidBody &body) const
{
  return body.orientation * axisBody;
}

glm::vec3 Magnetorquer::getWorldDipoleMoment(const RigidBody &body) const
{
  return getWorldAxis(body) * (commandedDipoleMoment * glm::clamp(healthFactor, 0.0f, 1.0f));
}

float Magnetorquer::getSaturationRatio() const
{
  return maxDipoleMoment > 0.0f ? (commandedDipoleMoment / maxDipoleMoment) : 0.0f;
}
