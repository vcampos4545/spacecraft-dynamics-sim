#include "LandingSoftware.h"
#include <algorithm>
#include <cmath>

namespace
{
constexpr float GROUND_Z = 0.0f;
constexpr float GRAVITY_MS2 = 9.81f;

// Vertical guidance: track the standard suicide-burn reference velocity
// profile v(h) = -sqrt(2 * REFERENCE_DECEL_MS2 * h) -- the speed you'd have
// if you'd been decelerating at a constant REFERENCE_DECEL_MS2 the whole
// way down from rest at infinite altitude, so it reaches exactly zero at
// h=0. Throttle is a closed-loop velocity servo tracking that curve, not a
// one-shot "decel needed from current speed" calculation: dropped from
// rest, current speed starts at (almost) zero while the reference speed at
// 600m is ~60 m/s, so throttle clamps to 0 (free fall) until actual speed
// catches up to the curve, then continuously brakes to stay on it the rest
// of the way down. (An earlier version computed decel from current speed
// directly and would falsely converge to an indefinite near-hover instead
// of ever really descending, because that quantity is ~0 whenever current
// speed is small relative to remaining altitude -- exactly the case right
// after a drop from rest.)
constexpr float REFERENCE_DECEL_MS2 = 1.0f;
constexpr float VELOCITY_TRACKING_TIME_CONSTANT_S = 0.5f;

// Lateral guidance: PD on horizontal position/velocity error, parameterized
// the same settling-time/damping-ratio way as the attitude loop below, but
// directly in acceleration space (commanding a lean angle, not a torque).
constexpr float LATERAL_SETTLING_TIME_S = 15.0f;
constexpr float LATERAL_DAMPING_RATIO = 1.0f;
constexpr float MAX_TILT_DEG = 8.0f;
constexpr float FLARE_ALTITUDE_M = 30.0f; // lateral tilt authority fades to zero below this

// Attitude (gimbal) guidance: same parameterization as
// examples/starship/FlightSoftware.cpp's ATTITUDE_SETTLING_TIME_S/
// ATTITUDE_DAMPING_RATIO, just faster -- a landing burn is a much shorter,
// tighter-tolerance maneuver than an ascent pitch program.
constexpr float ATTITUDE_SETTLING_TIME_S = 3.0f;
constexpr float ATTITUDE_DAMPING_RATIO = 1.0f;

constexpr float TOUCHDOWN_ALTITUDE_M = 0.5f;
constexpr float SAFE_TOUCHDOWN_SPEED_MS = 3.0f;
constexpr float SAFE_TOUCHDOWN_TILT_DEG = 8.0f;
} // namespace

LandingSoftware::LandingSoftware(Starship &ship, glm::vec2 targetXY)
    : m_ship(ship), m_holdPosition(ship.body->position), m_holdOrientation(ship.body->orientation),
      m_targetXY(targetXY)
{
}

const char *LandingSoftware::phaseName() const
{
  switch (m_phase)
  {
  case LandingPhase::HOLDING:
    return "Holding";
  case LandingPhase::DESCENT:
    return "Descent";
  case LandingPhase::LANDED:
    return "Landed";
  case LandingPhase::CRASHED:
    return "Crashed";
  }
  return "Unknown";
}

float LandingSoftware::altitude() const
{
  return m_ship.body->position.z - Starship::HEIGHT_M * 0.5f - GROUND_Z;
}

// Keeps the ship motionless at its starting pose regardless of how long the
// drop is held off -- just zeroing velocity each frame still lets a tiny
// per-substep gravity drift accumulate linearly with wall-clock time, so
// position/orientation are pinned back too.
void LandingSoftware::holdPose()
{
  m_ship.body->position = m_holdPosition;
  m_ship.body->orientation = m_holdOrientation;
  m_ship.body->velocity = glm::vec3(0.0f);
  m_ship.body->angularVelocity = glm::vec3(0.0f);
}

// Vertical suicide-burn throttle + lateral tilt command, combined into one
// target attitude. Returns the target; throttle is written to m_throttle.
glm::quat LandingSoftware::computeAttitudeTarget(float altitudeM)
{
  RigidBody &body = *m_ship.body;

  // --- Vertical: velocity servo tracking the suicide-burn reference curve ---
  float referenceSpeed = std::sqrt(2.0f * REFERENCE_DECEL_MS2 * std::max(altitudeM, 0.0f));
  float referenceVz = -referenceSpeed;
  float velocityError = body.velocity.z - referenceVz; // >0: falling slower than the curve wants

  float kV = 1.0f / VELOCITY_TRACKING_TIME_CONSTANT_S;
  float netAccelCmd = -kV * velocityError; // desired net (thrust - gravity) world-Z acceleration

  float maxThrust = float(m_ship.centerEngines.size()) *
                    (m_ship.centerEngines.empty() ? 0.0f : m_ship.centerEngines[0]->maxThrust);
  float maxAccelFromEngines = maxThrust / body.mass;
  m_throttle = maxAccelFromEngines > 1e-3f
      ? glm::clamp((netAccelCmd + GRAVITY_MS2) / maxAccelFromEngines, 0.0f, 1.0f)
      : 0.0f;

  // --- Lateral: PD on horizontal position/velocity -> a small lean angle ---
  glm::vec2 posError = m_targetXY - glm::vec2(body.position.x, body.position.y);
  glm::vec2 velXY(body.velocity.x, body.velocity.y);

  float omega_n = 4.0f / (LATERAL_DAMPING_RATIO * LATERAL_SETTLING_TIME_S);
  float kP = omega_n * omega_n;
  float kD = 2.0f * LATERAL_DAMPING_RATIO * omega_n;
  glm::vec2 desiredAccel = kP * posError - kD * velXY; // m/s^2, world frame

  glm::vec2 rawTilt = desiredAccel / GRAVITY_MS2; // small-angle approx: accel ~ g * tan(tilt)
  float maxTiltRad = glm::radians(MAX_TILT_DEG);
  rawTilt = glm::clamp(rawTilt, glm::vec2(-maxTiltRad), glm::vec2(maxTiltRad));

  // Fade lateral authority to zero near the ground so the vehicle is
  // forced back upright before touchdown instead of landing on a lean.
  float fade = glm::clamp(altitudeM / FLARE_ALTITUDE_M, 0.0f, 1.0f);
  m_targetTilt = rawTilt * fade;

  // tiltX (toward +X) is a rotation about +Y; tiltY (toward +Y) is a
  // rotation about -X -- same convention as FlightSoftware.cpp's pitch
  // program, verified there via Rodrigues' formula.
  return glm::angleAxis(m_targetTilt.x, glm::vec3(0, 1, 0)) *
         glm::angleAxis(-m_targetTilt.y, glm::vec3(1, 0, 0));
}

// Inertia-adaptive PD -> torque -> gimbal control, same technique (and
// same reasoning for the minus signs / lever-arm derivation) as
// examples/starship/FlightSoftware.cpp::controlAttitude -- reimplemented
// here rather than shared, matching this project's convention that each
// scenario owns its own flight software.
void LandingSoftware::controlAttitude(const glm::quat &targetAttitude)
{
  RigidBody &body = *m_ship.body;
  const std::vector<Thruster *> &gimbalEngines = m_ship.centerEngines;
  if (gimbalEngines.empty() || m_throttle <= 0.0f)
    return;

  glm::quat qError = glm::inverse(body.orientation) * targetAttitude;
  if (qError.w < 0.0f)
    qError = -qError;
  glm::vec3 axisError(qError.x, qError.y, qError.z);
  axisError *= 2.0f;

  glm::vec3 omegaBody = glm::inverse(body.orientation) * body.angularVelocity;

  float I = body.inertiaTensor[0][0];
  float omega_n = 4.0f / (ATTITUDE_DAMPING_RATIO * ATTITUDE_SETTLING_TIME_S);
  float Kp = I * omega_n * omega_n;
  float Kd = 2.0f * ATTITUDE_DAMPING_RATIO * I * omega_n;

  float torqueX = Kp * axisError.x - Kd * omegaBody.x;
  float torqueY = Kp * axisError.y - Kd * omegaBody.y;

  float leverArm = body.size.y * 0.5f;
  float torquePerRadian = float(gimbalEngines.size()) * gimbalEngines[0]->maxThrust * m_throttle * leverArm;
  if (torquePerRadian < 1e-3f)
    return;

  float pitchCmd = -torqueX / torquePerRadian;
  float yawCmd = -torqueY / torquePerRadian;

  for (Thruster *engine : gimbalEngines)
  {
    engine->pitch = pitchCmd;
    engine->yaw = yawCmd;
  }
}

void LandingSoftware::evaluateTouchdown()
{
  RigidBody &body = *m_ship.body;
  float speed = glm::length(body.velocity);

  glm::vec3 up = body.orientation * glm::vec3(0, 0, 1);
  float tiltDeg = glm::degrees(std::acos(glm::clamp(up.z, -1.0f, 1.0f)));

  bool soft = speed <= SAFE_TOUCHDOWN_SPEED_MS && tiltDeg <= SAFE_TOUCHDOWN_TILT_DEG;
  m_phase = soft ? LandingPhase::LANDED : LandingPhase::CRASHED;
  // m_throttle is read back by the caller and passed into Starship::update()
  // right after this, which is what actually zeroes the engines' throttle
  // fields -- no need to touch them directly here too.
  m_throttle = 0.0f;
}

void LandingSoftware::update(float dt, bool dropCommand)
{
  switch (m_phase)
  {
  case LandingPhase::HOLDING:
    holdPose();
    if (dropCommand)
      m_phase = LandingPhase::DESCENT;
    break;

  case LandingPhase::DESCENT:
  {
    m_missionTime += dt;
    float altitudeM = altitude();
    if (altitudeM <= TOUCHDOWN_ALTITUDE_M)
    {
      evaluateTouchdown();
      break;
    }

    glm::quat target = computeAttitudeTarget(altitudeM);
    controlAttitude(target);
    break;
  }

  case LandingPhase::LANDED:
  case LandingPhase::CRASHED:
    m_missionTime += dt;
    break;
  }
}
