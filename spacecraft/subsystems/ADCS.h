#pragma once
#include "glm/glm.hpp"
#include "ReactionWheel.h"
#include "RigidBody.h"
#include "Controllers.h"

enum class ControlMode
{
  SAFE_MODE,       // Fault recovery / survival
  DETUMBLE,        // Rate damping after deployment
  SUN_POINTING,    // Maintain solar panel alignment
  ATTITUDE_HOLD,   // Fixed inertial orientation
  TARGET_POINTING, // Track external target
  SLEW,            // Rapid attitude change
  MOMENTUM_DUMP,   // Reaction wheel desaturation
  FINE_POINTING    // High precision science mode
};

class ADCS
{
public:
  ControlMode mode = ControlMode::TARGET_POINTING;
  glm::vec3 target = {0.5f, 0.5f, 0.5f};
  glm::quat targetAttitude;
  glm::vec3 torqueCommand;
  std::vector<float> wheelCommands; // wheelCommands[i] -> torque for wheels[i]

  // Rigid body references
  RigidBody *body = nullptr;
  std::vector<ReactionWheel *> wheels;

public:
  ADCS() = default;
  ADCS(RigidBody *body_, const std::vector<ReactionWheel *> &wheels_);
  ~ADCS();

  void run(float dt);

  void computeGuidance(float dt);
  void computeControl(glm::quat attitude, glm::vec3 rate, float dt);
  void allocateActuators();
  void sendCommands();

private:
  PIDController pid;
  float settlingTime = 5.0f; // Desired settling time (seconds)
  float dampingRatio = 1.0f; // 1.0 = critically damped, <1 = underdamped (oscillates), >1 = overdamped (slow)
};