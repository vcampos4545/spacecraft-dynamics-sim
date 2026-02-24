#pragma once

#include "ForceGenerator.h"
#include <glm/glm.hpp>

class RigidBody;

class ReactionWheel : public ForceGenerator
{
public:
  glm::vec3 mountPositionBody; // where wheel is mounted (body frame)
  glm::vec3 spinAxisBody;      // axis of rotation (body frame, normalized)

  float maxTorque;    // max torque output (Nm)
  float maxSpeed;     // max wheel speed before saturation (rad/s)
  float wheelInertia; // moment of inertia of the wheel (kg*m^2)

  float currentSpeed;    // current wheel angular velocity (rad/s)
  float commandedTorque; // current commanded torque (Nm)

  ReactionWheel(
      glm::vec3 mountPosBody,
      glm::vec3 axisBody,
      float maxTorqueNm,
      float maxSpeedRadS = 6000.0f * (2.0f * 3.14159f / 60.0f), // default 6000 RPM
      float inertia = 0.00001f);                                // small wheel

  // Command a torque output (-1 to 1 normalized, or raw Nm)
  void commandNormalized(float command); // -1 to 1
  void commandTorque(float torqueNm);    // direct Nm

  // Apply the reaction torque to the body (call each physics step)
  void apply(RigidBody &body, float dt);

  // Get world-space position and axis for visualization
  glm::vec3 getWorldMountPosition(const RigidBody &body) const;
  glm::vec3 getWorldSpinAxis(const RigidBody &body) const;

  // Check if wheel is saturated (at max speed)
  bool isSaturated() const;
  float getSaturationRatio() const; // -1 to 1
};
