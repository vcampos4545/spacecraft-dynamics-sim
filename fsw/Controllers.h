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

// Attitude regulator via LQR state feedback (u = -K x).
// Per-axis double-integrator model: theta_dot = omega, omega_dot = u / I_axis
// (small-angle attitude error, principal-axis inertia, cross-axis coupling ignored).
// The continuous-time algebraic Riccati equation for this 2-state/1-input system
// has a closed form, so no numerical Riccati solver is needed.
class LQRController
{
public:
  // LQR weights: state cost on [attitude error, angular rate], control cost on torque
  glm::vec3 Q_att{50.0f};
  glm::vec3 Q_rate{5.0f};
  glm::vec3 R{1.0f};

  // Gains solved by autoTune (torque = -Kp * error - Kd * omega)
  glm::vec3 Kp{0.0f};
  glm::vec3 Kd{0.0f};

public:
  LQRController() = default;

  // Solve the per-axis CARE for the given principal inertia and LQR weights
  void autoTune(glm::mat3 inertiaTensor,
                glm::vec3 Q_att = glm::vec3{50.0f},
                glm::vec3 Q_rate = glm::vec3{5.0f},
                glm::vec3 R = glm::vec3{1.0f});

  // Compute control torque
  glm::vec3 computeControlTorque(
      glm::quat targetAttitude,
      const glm::quat &q_current,
      const glm::vec3 &omega_current,
      float dt);

  // No persistent state, kept for interface parity with PIDController
  void reset() {}
};
