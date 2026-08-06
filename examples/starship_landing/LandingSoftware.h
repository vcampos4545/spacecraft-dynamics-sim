#pragma once
#include "Starship.h"
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

// Mission phases for a single Starship dropped from altitude and trying to
// land itself.
enum class LandingPhase
{
  HOLDING, // suspended at the start pose, waiting for the drop command
  DESCENT, // falling/burning, closed-loop guidance active
  LANDED,  // touched down softly and upright
  CRASHED, // touched down too hard or too tilted
};

// A minimal powered-descent guidance demo: drop a Starship from altitude
// (optionally tilted) and try to bring it to a soft, upright landing using
// only its own engines. This targets the "low-altitude landing burn" phase
// specifically -- aero is small enough there to ignore (Starship's
// UniformDrag force generator still applies, but there's no lift/moment
// aero model, and none is needed at this altitude/speed regime), which is
// what makes this the tractable, GNC-relevant slice of a full booster landing to
// actually build.
//
// This is NOT the real algorithm SpaceX uses -- their powered descent
// guidance solves a genuine trajectory optimization problem (convex
// optimization / "lossless convexification") every guidance cycle. This is
// a much simpler, closed-form approximation, structured in three pieces:
//
//  - Vertical (throttle): a closed-loop velocity servo tracking the
//    standard suicide-burn reference curve v(h) = -sqrt(2*a_ref*h) -- the
//    speed you'd have if you'd been decelerating at a constant a_ref the
//    whole way down, so it reaches exactly zero at the ground. Dropped
//    from rest, actual speed starts far below the curve, so throttle
//    clamps to zero (free fall) until it catches up, then continuously
//    brakes to stay on the curve the rest of the way down.
//  - Lateral (attitude tilt): a PD law on horizontal position/velocity
//    error produces a small commanded vehicle tilt -- steering by leaning
//    the whole thrust vector, the only way a single-gimbal-cluster vehicle
//    can translate -- faded to zero below a flare altitude so the vehicle
//    is forced back upright before touchdown.
//  - Attitude (gimbal): the same inertia-adaptive PD -> torque -> gimbal
//    control as examples/starship/FlightSoftware.cpp, tracking whatever
//    attitude the lateral guidance above is currently commanding.
//
// Only the 3 gimbal-capable, sea-level-optimized centerEngines are used --
// real vehicles don't light vacuum-optimized engines this low either, and
// Starship::update()'s independent per-group throttle makes that a direct
// choice here instead of firing everything.
class LandingSoftware
{
public:
  LandingSoftware(Starship &ship, glm::vec2 targetXY);

  // Advances guidance/control and updates the commanded throttle (read
  // back via throttle() and passed into Starship::update()). Call once per
  // frame, before PhysicsWorld::step(). `dropCommand` is the only manual
  // input -- everything after that is autonomous.
  void update(float dt, bool dropCommand);

  LandingPhase phase() const { return m_phase; }
  const char *phaseName() const;

  float missionTime() const { return m_missionTime; }
  float throttle() const { return m_throttle; }
  float altitude() const; // base height above ground
  glm::vec2 targetTiltDeg() const { return glm::degrees(m_targetTilt); }

private:
  void holdPose();
  void controlDescent(float dt);
  glm::quat computeAttitudeTarget(float altitudeM);
  void controlAttitude(const glm::quat &targetAttitude);
  void evaluateTouchdown();

  Starship &m_ship;
  glm::vec3 m_holdPosition;
  glm::quat m_holdOrientation;
  glm::vec2 m_targetXY;

  LandingPhase m_phase = LandingPhase::HOLDING;
  float m_missionTime = 0.0f;
  float m_throttle = 0.0f;
  glm::vec2 m_targetTilt{0.0f}; // radians: (tilt toward +X, tilt toward +Y)
};
