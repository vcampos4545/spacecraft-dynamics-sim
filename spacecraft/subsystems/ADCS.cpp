#include "ADCS.h"
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/quaternion.hpp>

ADCS::ADCS(RigidBody *body_, const std::vector<ReactionWheel *> &wheels_)
{
  body = body_;
  wheels = wheels_;

  pid.autoTune(body->inertiaTensor, settlingTime, dampingRatio);
}

ADCS::~ADCS() = default;

void ADCS::run(float dt)
{
  // TODO: Get estimated attitude and rate from sensors
  auto attitude = body->orientation;
  auto rate = body->angularVelocity;

  computeGuidance(dt);
  computeControl(attitude, rate, dt);
  allocateActuators();
  sendCommands();
}

void ADCS::computeGuidance(float dt)
{
  glm::vec3 toTarget = glm::normalize(target - body->position);
  glm::vec3 bodyUp{0, 0, 1}; // Local +Z axis

  // Rotation from +Z to target direction
  float dot = glm::dot(bodyUp, toTarget);

  if (dot > 0.9999f)
  {
    targetAttitude = glm::quat{1, 0, 0, 0}; // Already aligned
  }
  else if (dot < -0.9999f)
  {
    // Opposite direction - rotate 180° around any perpendicular axis
    targetAttitude = glm::angleAxis(glm::pi<float>(), glm::vec3{1, 0, 0});
  }
  else
  {
    glm::vec3 axis = glm::normalize(glm::cross(bodyUp, toTarget));
    float angle = acos(dot);
    targetAttitude = glm::angleAxis(angle, axis);
  }
}

void ADCS::computeControl(glm::quat attitude, glm::vec3 rate, float dt)
{
  // TODO: Add switch statement for different controllers
  torqueCommand = pid.computeControlTorque(targetAttitude, attitude, rate, dt);
}

void ADCS::allocateActuators()
{
  int N = wheels.size();
  wheelCommands.assign(N, 0.0f);

  if (N == 0)
    return;

  if (N == 3)
  {
    // Build 3x3 effectiveness matrix A (column-major in GLM: A[col] = spin axis of wheel col)
    glm::mat3 A;
    for (int i = 0; i < 3; i++)
      A[i] = wheels[i]->spinAxisBody;

    // For a square effectiveness matrix the pseudoinverse is just the inverse
    glm::mat3 A_inv = glm::inverse(A);
    glm::vec3 wt = A_inv * torqueCommand;

    for (int i = 0; i < 3; i++)
      wheelCommands[i] = wt[i];
  }
  else
  {
    // General case: project desired torque onto each wheel's axis
    for (int i = 0; i < N; i++)
      wheelCommands[i] = glm::dot(wheels[i]->spinAxisBody, torqueCommand);
  }
}

void ADCS::sendCommands()
{
  for (int i = 0; i < (int)wheels.size(); i++)
    wheels[i]->commandTorque(wheelCommands[i]);
}
