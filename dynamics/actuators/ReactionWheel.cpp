#include "ReactionWheel.h"
#include "RigidBody.h"
#include <glm/gtc/quaternion.hpp>
#include <cmath>

ReactionWheel::ReactionWheel(
    glm::vec3 mountPosBody,
    glm::vec3 axisBody,
    float maxTorqueNm,
    float maxSpeedRadS,
    float inertia)
    : mountPositionBody(mountPosBody),
      spinAxisBody(glm::normalize(axisBody)),
      maxTorque(maxTorqueNm),
      maxSpeed(maxSpeedRadS),
      wheelInertia(inertia),
      currentSpeed(0.0f),
      commandedTorque(0.0f)
{
}

void ReactionWheel::commandNormalized(float command)
{
  command = glm::clamp(command, -1.0f, 1.0f);
  commandedTorque = command * maxTorque;
}

void ReactionWheel::commandTorque(float torqueNm)
{
  commandedTorque = glm::clamp(torqueNm, -maxTorque, maxTorque);
}

void ReactionWheel::apply(RigidBody &body, float dt)
{
  // Check for saturation - can't apply torque if wheel is at max speed
  // in the direction we're trying to accelerate
  float wheelAccel = commandedTorque / wheelInertia;
  float newSpeed = currentSpeed + wheelAccel * dt;

  // Clamp wheel speed (saturation)
  if (std::abs(newSpeed) > maxSpeed)
  {
    // Wheel is saturated, can only slow down
    if (newSpeed > maxSpeed)
    {
      newSpeed = maxSpeed;
      // Only allow negative torque (slowing the wheel)
      if (commandedTorque > 0)
        commandedTorque = 0;
    }
    else if (newSpeed < -maxSpeed)
    {
      newSpeed = -maxSpeed;
      // Only allow positive torque (slowing the wheel)
      if (commandedTorque < 0)
        commandedTorque = 0;
    }
  }

  // Update wheel speed
  currentSpeed = newSpeed;

  // Apply reaction torque to spacecraft (Newton's 3rd law)
  // Torque on spacecraft is opposite to torque on wheel
  glm::vec3 reactionTorqueBody = -commandedTorque * spinAxisBody;

  // Transform to world frame and apply
  glm::vec3 reactionTorqueWorld = body.orientation * reactionTorqueBody;
  body.applyTorque(reactionTorqueWorld);
}

glm::vec3 ReactionWheel::getWorldMountPosition(const RigidBody &body) const
{
  return body.position + body.orientation * mountPositionBody;
}

glm::vec3 ReactionWheel::getWorldSpinAxis(const RigidBody &body) const
{
  return body.orientation * spinAxisBody;
}

bool ReactionWheel::isSaturated() const
{
  return std::abs(currentSpeed) >= maxSpeed * 0.99f;
}

float ReactionWheel::getSaturationRatio() const
{
  return currentSpeed / maxSpeed;
}
