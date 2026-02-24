#pragma once

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

class PIDController
{
public:
  // PID gains
  float Kp = 0.001f;
  float Ki = 0.0001f;
  float Kd = 0.002f;

  // Integral state
  glm::vec3 integralError{0.0f};
  float maxIntegral = 0.1f;

public:
  PIDController() = default;

  // Auto-tune gains based on spacecraft inertia and desired response
  void autoTune(glm::mat3 inertiaTensor,
                float settlingTime,
                float dampingRatio);

  // Compute control torque
  glm::vec3 computeControlTorque(
      glm::quat targetAttitude,
      const glm::quat &q_current,
      const glm::vec3 &omega_current,
      float dt);

  // Reset integral state
  void reset();
};
