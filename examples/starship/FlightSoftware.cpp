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

  // Attitude -> gimbal PD gains (radians of gimbal per radian of attitude
  // error / per rad-s^-1 of rate error). Hand-tuned for a plausible-looking
  // response, not derived from vehicle inertia -- Thruster::apply() clamps
  // the result to each engine's own gimbalLimit regardless.
  constexpr float GIMBAL_KP = 2.0f;
  constexpr float GIMBAL_KD = 1.7f;
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
                                     const glm::quat &targetAttitude)
{
  // Quaternion attitude error in the body frame (same convention as
  // examples/cubesat_pyramid/Controllers.cpp's PIDController).
  glm::quat qError = glm::inverse(body.orientation) * targetAttitude;
  if (qError.w < 0.0f)
    qError = -qError;
  glm::vec3 axisError(qError.x, qError.y, qError.z);
  axisError *= 2.0f;

  glm::vec3 omegaBody = glm::inverse(body.orientation) * body.angularVelocity;

  // Engines mount at the base (local -Z, see Booster.cpp/Starship.cpp), so
  // gimbaling *away* from the direction you want the nose to move is what
  // actually produces the righting torque -- hence the minus signs.
  float pitchCmd = -GIMBAL_KP * axisError.x - GIMBAL_KD * omegaBody.x;
  float yawCmd = -GIMBAL_KP * axisError.y - GIMBAL_KD * omegaBody.y;

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
    controlAttitude(*m_booster.body, m_booster.centerEngines, verticalAttitude);
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
    controlAttitude(*m_booster.body, m_booster.centerEngines, target);

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
    controlAttitude(*m_ship.body, m_ship.centerEngines, target);

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
