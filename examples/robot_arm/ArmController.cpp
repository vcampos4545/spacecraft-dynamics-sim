#include "ArmController.h"
#include <algorithm>
#include <cmath>
#include <glm/gtc/constants.hpp>

namespace
{
constexpr float APPROACH_HEIGHT_M = 0.10f; // above the cube, before descending
constexpr float LIFT_HEIGHT_M = 0.12f;     // above the cube's resting position, once grasped

// A P-only velocity servo has real steady-state error under a constant
// disturbance (gravity torque on the shoulder/elbow) -- there's no
// integral term to drive it to zero, so this is set to what the control
// loop actually achieves in practice, not an idealized value.
constexpr float ARRIVAL_TOLERANCE_M = 0.05f;
constexpr float PHASE_TIMEOUT_S = 6.0f; // don't get stuck forever if it never quite converges

// Small clearance above the cube's top face for DESCEND's target, not the
// cube's volumetric center -- the forearm is a solid box with real
// thickness, so driving its tip all the way to the cube's center would
// need the forearm to overlap both the cube and (for a cube this close to
// the floor) the ground plane.
constexpr float GRASP_CLEARANCE_M = 0.01f;

// Joint velocity-servo tuning: targetSpeed = JOINT_KP * angleError, clamped
// to MAX_JOINT_SPEED. A single P gain on top of HingeConstraint's own
// built-in rate servo behaves like a first-order lag (approaches the
// target without overshoot for reasonable gains), so no derivative term is
// needed here the way the gimbal-driven scenarios needed one.
constexpr float JOINT_KP = 3.0f;
constexpr float MAX_JOINT_SPEED_RAD_S = 1.5f;
constexpr float JOINT_MAX_TORQUE_NM = 4.0f;
} // namespace

ArmController::ArmController(PhysicsWorld &world,
                             HingeConstraint *yaw, HingeConstraint *shoulder, HingeConstraint *elbow,
                             RigidBody *forearm, RigidBody *cube,
                             glm::vec3 shoulderPivotWorld, float upperArmLength, float forearmLength,
                             float forearmTipLocalZ)
    : m_world(world), m_yaw(yaw), m_shoulder(shoulder), m_elbow(elbow),
      m_forearm(forearm), m_cube(cube),
      m_shoulderPivotWorld(shoulderPivotWorld),
      m_upperArmLength(upperArmLength), m_forearmLength(forearmLength),
      m_forearmTipLocalZ(forearmTipLocalZ)
{
  m_yaw->setMotorMaxTorque(JOINT_MAX_TORQUE_NM);
  m_shoulder->setMotorMaxTorque(JOINT_MAX_TORQUE_NM);
  m_elbow->setMotorMaxTorque(JOINT_MAX_TORQUE_NM);
  m_currentTarget = m_shoulderPivotWorld + glm::vec3(0, 0, upperArmLength + forearmLength);
}

const char *ArmController::phaseName() const
{
  switch (m_phase)
  {
  case ArmPhase::IDLE:     return "Idle";
  case ArmPhase::APPROACH: return "Approach";
  case ArmPhase::DESCEND:  return "Descend";
  case ArmPhase::GRASP:    return "Grasp";
  case ArmPhase::LIFT:     return "Lift";
  case ArmPhase::DONE:     return "Done";
  }
  return "Unknown";
}

glm::vec3 ArmController::endEffectorPosition() const
{
  return m_forearm->position + m_forearm->orientation * glm::vec3(0, 0, m_forearmTipLocalZ);
}

float ArmController::yawAngle() const { return m_yaw->getAngle(); }
float ArmController::shoulderAngle() const { return m_shoulder->getAngle(); }
float ArmController::elbowAngle() const { return m_elbow->getAngle(); }

void ArmController::enterPhase(ArmPhase next)
{
  m_phase = next;
  m_phaseTime = 0.0f;
}

// Closed-form 2-link planar IK, home pose = pointing straight up (+Z) -- see
// ArmController.h for the full derivation. Reachability is enforced by
// scaling the target vector (r, h) to fit the arm's actual reachable
// annulus [|L1-L2|, L1+L2] *before* the law-of-cosines step, so acos() is
// never handed an out-of-domain argument.
void ArmController::solveIK(const glm::vec3 &targetWorld, float &outYaw, float &outShoulder, float &outElbow) const
{
  glm::vec3 rel = targetWorld - m_shoulderPivotWorld;
  outYaw = std::atan2(rel.y, rel.x);

  float r = std::sqrt(rel.x * rel.x + rel.y * rel.y);
  float h = rel.z;

  float L1 = m_upperArmLength;
  float L2 = m_forearmLength;
  float d = std::sqrt(r * r + h * h);

  const float minReach = std::abs(L1 - L2) + 1e-4f;
  const float maxReach = L1 + L2 - 1e-4f;
  if (d > maxReach)
  {
    float scale = maxReach / d;
    r *= scale;
    h *= scale;
    d = maxReach;
  }
  else if (d < minReach)
  {
    float scale = (d > 1e-6f) ? (minReach / d) : 1.0f;
    r *= scale;
    h *= scale;
    d = minReach;
  }

  float cosElbow = (d * d - L1 * L1 - L2 * L2) / (2.0f * L1 * L2);
  cosElbow = glm::clamp(cosElbow, -1.0f, 1.0f);
  outElbow = std::acos(cosElbow); // "elbow bends forward" branch

  outShoulder = std::atan2(r, h) - std::atan2(L2 * std::sin(outElbow), L1 + L2 * std::cos(outElbow));
}

void ArmController::driveJoint(HingeConstraint *joint, float targetAngle)
{
  // getAngle() is atan2-based, so it's always in [-pi, pi] -- a naive
  // subtraction blows up whenever the target and the joint's current angle
  // straddle that branch cut (e.g. target=+179deg, current=-179deg is a
  // 2-degree physical error, but targetAngle-current computes 358deg).
  // Wrapping the error into [-pi, pi] is what actually fixed the wild
  // yaw oscillation seen in headless testing before this was added.
  float error = targetAngle - joint->getAngle();
  error = std::fmod(error + glm::pi<float>(), glm::two_pi<float>());
  if (error < 0.0f)
    error += glm::two_pi<float>();
  error -= glm::pi<float>();

  float targetSpeed = glm::clamp(JOINT_KP * error, -MAX_JOINT_SPEED_RAD_S, MAX_JOINT_SPEED_RAD_S);
  joint->enableMotor(true);
  joint->setMotorTargetSpeed(targetSpeed);
}

bool ArmController::arrivedAtTarget() const
{
  return glm::length(endEffectorPosition() - m_currentTarget) < ARRIVAL_TOLERANCE_M;
}

void ArmController::update(float dt, bool pickCommand)
{
  m_phaseTime += dt;

  switch (m_phase)
  {
  case ArmPhase::IDLE:
    m_currentTarget = m_shoulderPivotWorld + glm::vec3(0, 0, m_upperArmLength + m_forearmLength);
    if (pickCommand)
      enterPhase(ArmPhase::APPROACH);
    break;

  case ArmPhase::APPROACH:
    m_currentTarget = m_cube->position + glm::vec3(0, 0, APPROACH_HEIGHT_M);
    if (arrivedAtTarget() || m_phaseTime > PHASE_TIMEOUT_S)
      enterPhase(ArmPhase::DESCEND);
    break;

  case ArmPhase::DESCEND:
    m_currentTarget = m_cube->position + glm::vec3(0, 0, m_cube->size.z * 0.5f + GRASP_CLEARANCE_M);
    if (arrivedAtTarget() || m_phaseTime > PHASE_TIMEOUT_S)
      enterPhase(ArmPhase::GRASP);
    break;

  case ArmPhase::GRASP:
    // Weld the forearm to the cube at the cube's current position -- a
    // kinematic attach, not a simulated frictional grip (see class
    // comment). m_liftTargetXY is captured now so LIFT has a fixed point
    // to aim for instead of chasing the cube's position, which is
    // meaningless once the cube moves with the arm.
    if (!m_grasp)
    {
      m_grasp = m_world.addFixedConstraint(m_forearm, m_cube, m_cube->position);
      m_liftTargetXY = glm::vec2(m_cube->position.x, m_cube->position.y);
      m_liftTargetHeight = m_cube->position.z + LIFT_HEIGHT_M;
    }
    enterPhase(ArmPhase::LIFT);
    break;

  case ArmPhase::LIFT:
    m_currentTarget = glm::vec3(m_liftTargetXY.x, m_liftTargetXY.y, m_liftTargetHeight);
    if (arrivedAtTarget() || m_phaseTime > PHASE_TIMEOUT_S)
      enterPhase(ArmPhase::DONE);
    break;

  case ArmPhase::DONE:
    // Hold m_currentTarget from the end of LIFT.
    break;
  }

  float targetYaw, targetShoulder, targetElbow;
  solveIK(m_currentTarget, targetYaw, targetShoulder, targetElbow);
  driveJoint(m_yaw, targetYaw);
  driveJoint(m_shoulder, targetShoulder);
  driveJoint(m_elbow, targetElbow);
}
