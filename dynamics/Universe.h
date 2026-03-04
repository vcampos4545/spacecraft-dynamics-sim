#pragma once
#include "CelestialBody.h"
#include "RigidBody.h"
#include <vector>
#include <memory>
#include <unordered_map>
#include <utility>
#include <string>

// ---------------------------------------------------------------------------
// Universe — reference-frame-agnostic physics engine for spacecraft dynamics.
//
// Design principles:
//   - CelestialBodies provide N-body gravity; rigid bodies feel all of them.
//   - Orbital translation is integrated with RK4 at (optionally warped) time.
//   - Attitude dynamics run at real-time (no warp) using the existing
//     ForceGenerator / integrateAngular pipeline from RigidBody.
//   - The frame is defined by which body you choose as "fixed" (anchor).
//     Earth-centered: add Earth as fixed, Moon/satellites as non-fixed.
//     Sun-centered: add Sun as fixed, planets as non-fixed, etc.
//   - Rigid-body masses are assumed negligible compared to celestials and do
//     not back-react on them (standard astrodynamics approximation).
//
// Non-fixed celestials integrate with a decoupled RK4 (each body uses other
// bodies' start-of-step positions for their acceleration evaluation), which
// introduces O(h²) splitting error — sufficient for our time-step sizes.
// ---------------------------------------------------------------------------
class Universe
{
public:
  static constexpr double G = 6.674e-11; // m³ kg⁻¹ s⁻²

  std::vector<CelestialBody> celestials;

  // ---- Celestial management -----------------------------------------------

  // Add a celestial body and return a reference to it.
  CelestialBody &addCelestial(const std::string &name,
                               double mass,
                               double radius,
                               glm::dvec3 position = {0, 0, 0},
                               glm::dvec3 velocity = {0, 0, 0},
                               bool fixed          = true);

  // ---- Rigid-body management ----------------------------------------------

  // Add a rigid body. Universe owns it; returns a non-owning raw pointer.
  RigidBody *addBody(RigidBodyShape shape,
                     const glm::vec3 &size,
                     float mass);

  // Set / get double-precision orbital state for a registered rigid body.
  // setOrbitalState also syncs body->position (vec3) immediately.
  void       setOrbitalState(RigidBody *body, glm::dvec3 pos, glm::dvec3 vel);
  glm::dvec3 getOrbitalPosition(RigidBody *body) const;
  glm::dvec3 getOrbitalVelocity(RigidBody *body) const;

  // ---- Simulation ---------------------------------------------------------

  // Advance the simulation.
  //   wallDt        : real elapsed time (seconds)
  //   timeWarp      : orbital integration multiplier (attitude always uses wallDt)
  //   maxOrbitalStep: max integrator sub-step in sim-seconds (clamps error at
  //                   high time warps)
  void step(double wallDt, int timeWarp = 1, double maxOrbitalStep = 60.0);

  // ---- Helpers ------------------------------------------------------------

  // Compute initial position + velocity for a prograde circular orbit
  // around a given celestial at the ascending node.
  //   altM    : altitude above the celestial's surface (m)
  //   incRad  : orbital inclination (rad)
  //   raanRad : right-ascension of ascending node (rad)
  static std::pair<glm::dvec3, glm::dvec3>
  circularOrbit(const CelestialBody &around,
                double altM,
                double incRad  = 0.0,
                double raanRad = 0.0);

  // Altitude above a celestial's surface (m).
  static double altitude(const CelestialBody &around, glm::dvec3 pos);

  // Orbital period of the current orbit around a celestial (seconds).
  // Returns 0 for hyperbolic / escape trajectories.
  static double orbitalPeriod(const CelestialBody &around,
                               glm::dvec3 pos,
                               glm::dvec3 vel);

  // Read-only access to owned bodies.
  const std::vector<std::unique_ptr<RigidBody>> &getBodies() const
  {
    return bodies_;
  }

private:
  struct OrbitalState
  {
    glm::dvec3 pos{0.0};
    glm::dvec3 vel{0.0};
  };

  std::vector<std::unique_ptr<RigidBody>> bodies_;
  std::vector<OrbitalState>               orbStates_;
  std::unordered_map<RigidBody *, size_t> bodyIndex_;

  // N-body gravitational acceleration at world position pos,
  // summing contributions from all celestials at their current positions.
  glm::dvec3 gravAccelAtPos(const glm::dvec3 &pos) const;

  // One RK4 sub-step for all non-fixed celestials and all rigid bodies.
  void stepRK4(double dt);
};
