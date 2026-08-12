#pragma once
#include <rigidbody/orbit/OrbitState.h>
#include <rigidbody/orbit/OrbitPropagator.h>
#include <glm/glm.hpp>
#include <string>
#include <functional>

// Physical/rotational parameters for one body in a CelestialSystem's
// hierarchy -- generalizes what OrbitFrames'/EclipseModel's/
// AtmosphericDrag's Earth-hardcoded constants used to assume, per body.
// Every field is opt-in: leaving a term at its zero default disables it
// (no J2, no atmosphere, no magnetic field) rather than requiring a
// scenario to model everything about a body it doesn't care about.
struct CelestialBodyParams
{
  double mu = 0.0;      // GM, m^3/s^2
  double radiusM = 0.0; // mean/equatorial radius, m
  double j2 = 0.0;      // zero disables J2 perturbation for this body

  // Constant-rate sidereal rotation -- a simplified stand-in for
  // OrbitFrames::gmstRad's Earth-specific IAU polynomial (no precession
  // terms), reusable for any body: rotationAngle(jd) = rotationAngle0Rad +
  // rotationRateRadS * (jd - jd0) * 86400.
  double rotationRateRadS = 0.0;
  double rotationAngle0Rad = 0.0;
  double jd0 = 2451545.0;
  glm::dvec3 rotationAxis{0.0, 0.0, 1.0};

  double seaLevelDensityKgM3 = 0.0; // zero disables atmosphere/drag
  double scaleHeightM = 0.0;

  double dipoleTiltDeg = 0.0;   // zero disables magnetic-field sampling
  double dipoleScaleTm3 = 0.0;
};

// One node in a CelestialSystem's body hierarchy. `parent == nullptr` means
// this body is the system's root (fixed at the system origin); every other
// body's position is expressed relative to its own parent, exactly one of
// two ways:
//
// - `analyticPositionFn`: a cheap closed-form ephemeris (e.g. wrapping
//   SunModel::positionEci/MoonModel::positionEci for real Sun/Moon-class
//   bodies with a known low-precision formula).
// - `orbitState`/`orbitPropagator`: numeric two-body(+perturbation)
//   propagation relative to the parent, for bodies with no analytic fit --
//   stepped by CelestialSystem::step() every cycle. The caller is
//   responsible for populating `orbitPropagator` with whatever force
//   models it wants (typically a single TwoBodyGravity toward the
//   parent's mu), the same "you wire up your own force list" convention
//   OrbitPropagator already uses everywhere else in this module.
//
// Leave `analyticPositionFn` unset to select the numeric path.
class CelestialBody
{
public:
  std::string name;
  CelestialBodyParams params;
  CelestialBody *parent = nullptr;

  std::function<glm::dvec3(double jd)> analyticPositionFn;
  OrbitState orbitState;
  OrbitPropagator orbitPropagator;
};
