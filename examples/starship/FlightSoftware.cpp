#include "FlightSoftware.h"
#include <algorithm>

namespace
{
  // Sequencer timing (seconds) -- hand-picked for a "super simple" demo
  // profile, not derived from any real launch's actual timeline.
  constexpr float VERTICAL_RISE_TIME = 8.0f;  // clear the pad before tilting
  constexpr float PITCH_KICK_DURATION = 8.0f; // ramp to PITCH_KICK_DEG
  constexpr float PITCH_KICK_DEG = 3.0f;
  constexpr float GRAVITY_TURN_DURATION = 90.0f; // ramp from PITCH_KICK_DEG to MAX_PITCH_DEG
  constexpr float MAX_PITCH_DEG = 45.0f;

  constexpr float MECO_PROPELLANT_FRACTION = 0.01f; // "empty enough" to cut off
  constexpr float MECO_COAST_TIME = 2.0f;           // coast before separating
  constexpr float SHIP_IGNITION_DELAY = 1.5f;       // coast after separation before lighting
  constexpr float SECO_PROPELLANT_FRACTION = 0.01f;

  // Target closed-loop attitude response -- this is what actually stays
  // constant; the resulting Kp/Kd (computed from these plus the body's
  // *current* inertia in controlAttitude) do not. A fixed Kp/Kd in gimbal-
  // angle space was the earlier, broken approach: Booster/Starship shed the
  // large majority of their mass to propellant burn over one ascent, so a
  // gain that looks reasonable at liftoff is wrong by an order of magnitude
  // (in the wrong direction -- too weak) by MECO.
  constexpr float ATTITUDE_SETTLING_TIME_S = 4.0f;
  constexpr float ATTITUDE_DAMPING_RATIO = 1.0f;
} // namespace

FlightSoftware::FlightSoftware(PhysicsWorld &world, Booster &booster, Starship &ship, FixedConstraint *stackWeld)
    : m_world(world), m_booster(booster), m_ship(ship), m_stackWeld(stackWeld)
{
}

const char *FlightSoftware::phaseName() const
{
  switch (m_phase)
  {
  case MissionPhase::PRELAUNCH:
    return "Prelaunch";
  case MissionPhase::LIFTOFF:
    return "Liftoff";
  case MissionPhase::PITCH_KICK:
    return "Pitch Kick";
  case MissionPhase::GRAVITY_TURN:
    return "Gravity Turn (Booster Ascent)";
  case MissionPhase::MECO:
    return "MECO";
  case MissionPhase::STAGE_SEP:
    return "Stage Separation";
  case MissionPhase::SHIP_IGNITION:
    return "Ship Ignition";
  case MissionPhase::ASCENT:
    return "Ascent (Second Stage)";
  case MissionPhase::SECO:
    return "SECO";
  }
  return "Unknown";
}

void FlightSoftware::enterPhase(MissionPhase next)
{
  m_phase = next;
  m_phaseTime = 0.0f;
}

// Open-loop pitch-vs-time schedule: hold PITCH_KICK_DEG's ramp for the kick,
// then continue tilting toward MAX_PITCH_DEG over the gravity-turn duration.
// tSincePitchStart is continuous across staging, so the ship's ASCENT burn
// picks up the same schedule right where the booster's GRAVITY_TURN left it
// instead of snapping to a new target attitude.
float FlightSoftware::pitchProgramDeg(float tSincePitchStart) const
{
  if (tSincePitchStart < PITCH_KICK_DURATION)
  {
    float frac = tSincePitchStart / PITCH_KICK_DURATION;
    return PITCH_KICK_DEG * frac;
  }
  float turnT = tSincePitchStart - PITCH_KICK_DURATION;
  float turnFrac = glm::clamp(turnT / GRAVITY_TURN_DURATION, 0.0f, 1.0f);
  return PITCH_KICK_DEG + (MAX_PITCH_DEG - PITCH_KICK_DEG) * turnFrac;
}

void FlightSoftware::controlAttitude(RigidBody &body, const std::vector<Thruster *> &gimbalEngines,
                                     const glm::quat &targetAttitude, float throttle)
{
  if (gimbalEngines.empty() || throttle <= 0.0f)
    return; // no thrust means no gimbal authority regardless of what's commanded

  // --- Attitude PD -> torque command -------------------------------------
  // Quaternion attitude error in the body frame (same convention as
  // examples/cubesat_pyramid/Controllers.cpp's PIDController). Kp/Kd are
  // re-derived every call from the body's *current* transverse moment of
  // inertia (Ixx == Iyy for a cylinder -- see RigidBody::computeInertiaFromMass)
  // via the same settling-time/damping-ratio parameterization as
  // PIDController::autoTune, so the response stays consistent as propellant
  // burns off and the vehicle gets dramatically lighter.
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

  // --- Torque command -> collective gimbal angle --------------------------
  // Every gimbal-capable engine deflects the same amount (one effective TVC
  // vector, not a per-engine allocation). For a small gimbal angle theta,
  // each engine contributes a transverse force of maxThrust*throttle*theta
  // at a lever arm of half the stage's own height (engines mount at the
  // base, body.size.y/2 below the center of mass), so
  // torque = -(numEngines * maxThrust * throttle * leverArm) * theta --
  // negative because engines below the CoM must gimbal *away* from the
  // direction you want the nose to move to produce a righting torque (see
  // Booster.cpp/Starship.cpp's "engines mount at local -Z"). Solving for
  // theta gives the gimbal command below.
  float leverArm = body.size.y * 0.5f;
  float torquePerRadian = float(gimbalEngines.size()) * gimbalEngines[0]->maxThrust * throttle * leverArm;
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

void FlightSoftware::update(float dt, bool launchCommit)
{
  m_phaseTime += dt;

  glm::quat verticalAttitude(1.0f, 0.0f, 0.0f, 0.0f);

  switch (m_phase)
  {
  case MissionPhase::PRELAUNCH:
    m_boosterThrottle = 0.0f;
    m_shipThrottle = 0.0f;
    if (launchCommit)
      enterPhase(MissionPhase::LIFTOFF);
    break;

  case MissionPhase::LIFTOFF:
    m_missionTime += dt;
    m_boosterThrottle = 1.0f;
    m_targetPitchDeg = 0.0f;
    controlAttitude(*m_booster.body, m_booster.centerEngines, verticalAttitude, m_boosterThrottle);
    if (m_phaseTime >= VERTICAL_RISE_TIME)
      enterPhase(MissionPhase::PITCH_KICK);
    break;

  case MissionPhase::PITCH_KICK:
  case MissionPhase::GRAVITY_TURN:
  {
    m_missionTime += dt;
    m_pitchProgramTime += dt;
    m_boosterThrottle = 1.0f;
    m_targetPitchDeg = pitchProgramDeg(m_pitchProgramTime);
    glm::quat target = glm::angleAxis(glm::radians(m_targetPitchDeg), glm::vec3(0, 1, 0));
    controlAttitude(*m_booster.body, m_booster.centerEngines, target, m_boosterThrottle);

    if (m_phase == MissionPhase::PITCH_KICK && m_phaseTime >= PITCH_KICK_DURATION)
      enterPhase(MissionPhase::GRAVITY_TURN);
    else if (m_phase == MissionPhase::GRAVITY_TURN && m_booster.propellantFraction() <= MECO_PROPELLANT_FRACTION)
      enterPhase(MissionPhase::MECO);
    break;
  }

  case MissionPhase::MECO:
    m_missionTime += dt;
    m_boosterThrottle = 0.0f;
    if (m_phaseTime >= MECO_COAST_TIME)
      enterPhase(MissionPhase::STAGE_SEP);
    break;

  case MissionPhase::STAGE_SEP:
    m_missionTime += dt;
    m_boosterThrottle = 0.0f;
    if (m_stackWeld)
    {
      m_world.removeConstraint(m_stackWeld);
      m_stackWeld = nullptr;
    }
    if (m_phaseTime >= SHIP_IGNITION_DELAY)
      enterPhase(MissionPhase::SHIP_IGNITION);
    break;

  case MissionPhase::SHIP_IGNITION:
    m_missionTime += dt;
    m_shipThrottle = 1.0f;
    enterPhase(MissionPhase::ASCENT); // hands off into the ship burn next frame
    break;

  case MissionPhase::ASCENT:
  {
    m_missionTime += dt;
    m_pitchProgramTime += dt;
    m_shipThrottle = 1.0f;
    m_targetPitchDeg = pitchProgramDeg(m_pitchProgramTime);
    glm::quat target = glm::angleAxis(glm::radians(m_targetPitchDeg), glm::vec3(0, 1, 0));
    controlAttitude(*m_ship.body, m_ship.centerEngines, target, m_shipThrottle);

    if (m_ship.propellantFraction() <= SECO_PROPELLANT_FRACTION)
      enterPhase(MissionPhase::SECO);
    break;
  }

  case MissionPhase::SECO:
    m_missionTime += dt;
    m_shipThrottle = 0.0f;
    break;
  }
}
