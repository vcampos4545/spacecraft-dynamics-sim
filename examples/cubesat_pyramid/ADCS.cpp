#include "ADCS.h"
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/quaternion.hpp>

namespace
{
struct ModeTuning
{
  float settlingTime;
  float dampingRatio;
  float omega_max;
};

// Per-mode gain/rate-limit presets. SLEW trades precision for speed (short
// settling time, high rate cap); FINE_POINTING trades speed for precision
// (long settling time, overdamped, tight rate cap). Everything else uses
// the same moderate tuning the original single-mode cubesat used.
ModeTuning tuningForMode(PointingMode mode)
{
  switch (mode)
  {
  case PointingMode::SLEW:
    return {2.0f, 0.9f, 1.0f};
  case PointingMode::FINE_POINTING:
    return {12.0f, 1.3f, 0.08f};
  case PointingMode::NADIR:
  case PointingMode::SUN_POINTING:
  case PointingMode::TARGET:
  case PointingMode::DETUMBLE:
  default:
    return {5.0f, 1.0f, 0.5f};
  }
}
} // namespace

ADCS::ADCS(RigidBody *body_, const std::vector<ReactionWheel *> &wheels_, IMU *imu_)
{
  body = body_;
  wheels = wheels_;
  imu = imu_;

  // Bootstrap: there's no absolute-attitude sensor yet (sun sensor /
  // magnetometer / star tracker come later), so the estimator has no way
  // to acquire attitude on its own. Starting from truth is the one place
  // ground truth is used -- every update after this comes from the IMU.
  estimatedAttitude = body->orientation;

  lastTunedMode = mode;
  retuneForMode();

  // Detumble's rate-damping gain isn't part of the quaternion-error
  // controllers' tuning; derive it from inertia the same way autoTune
  // derives Kd, targeting a gentle ~8s damping settle.
  float I = (body->inertiaTensor[0][0] + body->inertiaTensor[1][1] + body->inertiaTensor[2][2]) / 3.0f;
  float detumbleSettlingTime = 8.0f;
  float omega_n = 4.0f / detumbleSettlingTime;
  detumbleKd = 2.0f * I * omega_n;
}

ADCS::~ADCS() = default;

void ADCS::resetController()
{
  pid.reset();
  lqr.reset();
  cascaded.reset();
}

void ADCS::retuneForMode()
{
  ModeTuning t = tuningForMode(mode);
  pid.autoTune(body->inertiaTensor, t.settlingTime, t.dampingRatio);
  lqr.autoTune(body->inertiaTensor, t.settlingTime, t.dampingRatio, t.omega_max);
  cascaded.autoTune(body->inertiaTensor, t.settlingTime, t.dampingRatio, t.omega_max);
  resetController(); // discard integral windup/state accumulated under the old tuning
}

void ADCS::run(float dt)
{
  if (mode != lastTunedMode)
  {
    retuneForMode();
    lastTunedMode = mode;
  }

  // Sensors: the IMU is the only source of attitude/rate information here.
  // Ground truth (body->orientation / body->angularVelocity) is never read
  // past the initial bootstrap above.
  IMU::Reading imuReading = imu->sample(*body, gravity, dt);

  // Rate feedback is the gyro reading directly (already body-frame -- see
  // IMU::sample's frame conversion).
  glm::vec3 rate = imuReading.gyro;

  // Attitude estimate: strapdown integration of the gyro,
  // q_dot = 0.5 * q (x) omega_body. There is no absolute attitude
  // reference yet (no sun sensor / magnetometer / star tracker -- those
  // come later), so this is open-loop dead reckoning: gyro bias and noise
  // accumulate into a slowly-growing attitude error with nothing to
  // correct it. That's a real, expected limitation of an IMU-only system,
  // not a bug to hide.
  glm::quat omegaBodyQuat(0.0f, imuReading.gyro.x, imuReading.gyro.y, imuReading.gyro.z);
  estimatedAttitude = estimatedAttitude + 0.5f * estimatedAttitude * omegaBodyQuat * dt;
  estimatedAttitude = glm::normalize(estimatedAttitude);

  computeGuidance(dt);
  computeControl(estimatedAttitude, rate, dt);
  allocateActuators();
  sendCommands();
}

void ADCS::computeGuidance(float dt)
{
  if (mode == PointingMode::DETUMBLE)
    return; // no attitude target; computeControl uses a rate-only law instead

  glm::vec3 pointDir;
  switch (mode)
  {
  case PointingMode::NADIR:
    // "Straight down" -- this sim has no orbital mechanics, so there's no
    // real nadir vector (Earth-center direction) to compute; a fixed world
    // direction is the honest equivalent here.
    pointDir = glm::vec3(0, 0, -1);
    break;
  case PointingMode::SUN_POINTING:
    pointDir = glm::normalize(sunPosition - body->position);
    break;
  case PointingMode::TARGET:
  case PointingMode::SLEW:
  case PointingMode::FINE_POINTING:
  default:
    pointDir = glm::normalize(target - body->position);
    break;
  }

  glm::vec3 bodyUp{0, 0, 1}; // Local +Z axis

  // Rotation from +Z to the pointing direction
  float dot = glm::dot(bodyUp, pointDir);

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
    glm::vec3 axis = glm::normalize(glm::cross(bodyUp, pointDir));
    float angle = acos(dot);
    targetAttitude = glm::angleAxis(angle, axis);
  }
}

void ADCS::computeControl(glm::quat attitude, glm::vec3 rate, float dt)
{
  if (mode == PointingMode::DETUMBLE)
  {
    torqueCommand = computeDetumbleTorque(rate);
    return;
  }

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

glm::vec3 ADCS::computeDetumbleTorque(const glm::vec3 &rate) const
{
  // Pure rate damping: right after deployment there's no attitude worth
  // chasing yet, only a rotation to kill. A wheel-only detumble can soak
  // up momentum but never dump it (that needs magnetorquers, added
  // later), so it will eventually saturate on a body with real residual
  // momentum -- that's expected, not a bug in this control law.
  //
  // Sign note: the physically-damping torque is -Kd*rate, but
  // allocateActuators()/ReactionWheel::apply() apply the *negative* of
  // whatever torqueCommand they're given (Newton's-third-law reaction
  // convention -- confirmed by direct measurement, not just derivation).
  // PIDController::computeControlTorque already returns its torque
  // pre-negated to compensate for that; this needs the same compensating
  // negation, so the value returned here is +Kd*rate.
  return detumbleKd * rate;
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
