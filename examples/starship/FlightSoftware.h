#pragma once
#include <rigidbody/PhysicsWorld.h>
#include <rigidbody/Constraint.h>
#include <rigidbody/actuators/Thruster.h>
#include "Booster.h"
#include "Starship.h"
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

// Mission phases, named the way real ascent flight software / launch
// commentary names them. The sequencer below drives these transitions off
// time and propellant thresholds instead of a human pressing buttons --
// once LIFTOFF is commanded, everything through SECO runs on its own.
enum class MissionPhase
{
  PRELAUNCH,      // engines off, waiting for launch commit
  LIFTOFF,        // full throttle, vertical rise clear of the pad
  PITCH_KICK,     // brief open-loop tilt that starts the turn
  GRAVITY_TURN,   // booster burn, following the pitch-vs-time program
  MECO,           // booster engines cut off, brief coast before separation
  STAGE_SEP,      // weld released, coasting unpowered
  SHIP_IGNITION,  // ship engines light
  ASCENT,         // ship burn, continuing the pitch program
  SECO,           // ship engines cut off (propellant exhausted)
};

// A minimal ascent + staging sequencer, structured like a real ascent GNC
// (Guidance, Navigation, Control) stack but simplified for a rigid-body sim
// with no aerodynamics:
//
//  - Navigation: reads Booster/Starship's RigidBody state directly, as a
//    stand-in for a real navigation filter (no IMU is mounted on either
//    stage yet -- see examples/cubesat_pyramid for one that does).
//  - Guidance: an open-loop pitch-vs-time program. Real vehicles fly a
//    (mostly) closed-loop trajectory computed against live vehicle state
//    (e.g. Powered Explicit Guidance); a fixed schedule is the closest
//    equivalent here since this sim has no aerodynamic forces to fly a
//    true zero-angle-of-attack gravity turn against.
//  - Control: a PD law on attitude error produces a torque command, using
//    gains re-derived from the body's *current* transverse inertia every
//    call (same settling-time/damping-ratio parameterization as
//    examples/cubesat_pyramid/Controllers.cpp's PIDController::autoTune) --
//    Booster/Starship lose most of their mass to propellant burn over one
//    ascent, so fixed gains go badly wrong well before MECO. That torque
//    is then converted into a *collective* gimbal command applied
//    identically to every currently-firing gimbal-capable engine (treating
//    the cluster as one effective TVC vector rather than solving a
//    per-engine allocation), via the physical lever-arm relationship
//    between gimbal angle and torque. Only pitch/yaw are controlled --
//    roll is left free, same as this sim's other scenarios.
//  - Sequencer: a simple state machine drives MECO / stage separation /
//    second-stage ignition / SECO off time and propellant thresholds,
//    mirroring a real flight computer's mission-event sequencer.
class FlightSoftware
{
public:
  FlightSoftware(PhysicsWorld &world, Booster &booster, Starship &ship, FixedConstraint *stackWeld);

  // Advances the sequencer, computes guidance + control, and updates the
  // commanded throttles (read back via boosterThrottle()/shipThrottle() and
  // passed into Booster::update()/Starship::update()). Call once per frame,
  // before PhysicsWorld::step(). `launchCommit` is the only manual input --
  // everything after LIFTOFF is autonomous.
  void update(float dt, bool launchCommit);

  MissionPhase phase() const { return m_phase; }
  const char *phaseName() const;
  bool staged() const { return m_stackWeld == nullptr; }

  float missionTime() const { return m_missionTime; }
  float boosterThrottle() const { return m_boosterThrottle; }
  float shipThrottle() const { return m_shipThrottle; }
  float targetPitchDeg() const { return m_targetPitchDeg; }

private:
  void enterPhase(MissionPhase next);
  float pitchProgramDeg(float tSincePitchStart) const;
  void controlAttitude(RigidBody &body, const std::vector<Thruster *> &gimbalEngines,
                       const glm::quat &targetAttitude, float throttle);

  PhysicsWorld &m_world;
  Booster &m_booster;
  Starship &m_ship;
  FixedConstraint *m_stackWeld;

  MissionPhase m_phase = MissionPhase::PRELAUNCH;
  float m_missionTime = 0.0f;      // seconds since LIFTOFF (T+)
  float m_phaseTime = 0.0f;        // seconds since entering the current phase
  float m_pitchProgramTime = 0.0f; // seconds since PITCH_KICK began, continuous across staging

  float m_boosterThrottle = 0.0f;
  float m_shipThrottle = 0.0f;
  float m_targetPitchDeg = 0.0f;
};
