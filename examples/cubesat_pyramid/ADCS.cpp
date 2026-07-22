#include "ADCS.h"
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/quaternion.hpp>

ADCS::ADCS(RigidBody *body_, const std::vector<ReactionWheel *> &wheels_)
{
  body = body_;
  wheels = wheels_;

  pid.autoTune(body->inertiaTensor, settlingTime, dampingRatio);
  lqr.autoTune(body->inertiaTensor, settlingTime, dampingRatio);
  cascaded.autoTune(body->inertiaTensor, settlingTime, dampingRatio);
}

ADCS::~ADCS() = default;

void ADCS::resetController()
{
  pid.reset();
  lqr.reset();
  cascaded.reset();
}

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
  switch (controllerType)
  {
  case ControllerType::LQR:
    torqueCommand = lqr.computeControlTorque(targetAttitude, attitude, rate, dt);
    break;
  case ControllerType::CASCADED:
    torqueCommand = cascaded.computeControlTorque(targetAttitude, attitude, rate, dt);
    break;
  case ControllerType::PID:
  default:
    torqueCommand = pid.computeControlTorque(targetAttitude, attitude, rate, dt);
    break;
  }
}

void ADCS::allocateActuators()
{
  int N = wheels.size();
  wheelCommands.assign(N, 0.0f);

  if (N == 0)
    return;

  // Minimum-norm allocation via the Moore-Penrose pseudoinverse
  // A+ = A^T (A A^T)^-1, where A's columns are each wheel's spin axis
  // (body frame). A A^T is only 3x3 regardless of wheel count, so this
  // handles any N (a 3-orthogonal-wheel cluster, a 4-wheel pyramid, ...)
  // as long as the wheels span all 3 axes. For N == 3 orthogonal wheels
  // A A^T == I, so this reduces to exactly the direct-inverse case.
  glm::mat3 AAt(0.0f);
  for (int i = 0; i < N; i++)
  {
    const glm::vec3 &a = wheels[i]->spinAxisBody;
    AAt += glm::mat3(a.x * a, a.y * a, a.z * a); // outer product a * a^T
  }

  glm::vec3 y = glm::inverse(AAt) * torqueCommand;

  for (int i = 0; i < N; i++)
    wheelCommands[i] = glm::dot(wheels[i]->spinAxisBody, y);
}

void ADCS::sendCommands()
{
  for (int i = 0; i < (int)wheels.size(); i++)
    wheels[i]->commandTorque(wheelCommands[i]);
}
