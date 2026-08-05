#pragma once
#include <glm/glm.hpp>

// Simplified LEO magnetic field model: a tilted dipole (Earth's real field
// to first order) sampled along a kinematic circular orbit driven purely by
// simulated time -- not a ForceGenerator, since it doesn't act on a body
// directly, it's an ambient field a scenario samples and feeds to
// Magnetorquer::ambientFieldWorld / Magnetometer::sample(), the same role
// Gravity::acceleration plays for IMU::sample()'s specific-force
// calculation.
//
// This sim has no orbital-position dynamics for the cubesat body itself
// (see ADCS.cpp's NADIR comment -- pointing modes already use a fixed
// stand-in "down" direction for the same reason), so orbital motion here is
// a separate, purely kinematic phase angle: theta(t) = 2*pi*t/orbitalPeriodS,
// swept along an inclined circular orbit. That alone is enough to give a
// magnetorquer/B-dot controller a genuinely time-varying field to detumble
// against, matching what a real LEO ADCS sees as it moves through Earth's
// field -- it just isn't tied to where the rendered cubesat mesh actually
// sits in the scene.
//
// Field magnitude/geometry uses the standard dipole formula
//   B(r) = (K / r^3) * (3*(m . r_hat)*r_hat - m)
// with K = mu0*|m_earth|/(4*pi) ~= 7.94e15 T*m^3, which reproduces the
// right order of magnitude (~25-65 uT) for real LEO field strength.
class MagneticField
{
public:
  float altitudeKm = 500.0f;      // orbital altitude above Earth's surface
  float orbitalPeriodS = 5400.0f; // ~90 minutes, typical LEO
  float inclinationDeg = 51.6f;   // orbit inclination from the equatorial plane (ISS-like default)
  float dipoleTiltDeg = 11.0f;    // Earth's magnetic dipole tilt from the rotation axis

  // Direction the field vectors are drawn toward in the *scene* (world
  // frame) -- purely a visualization choice, decoupled from the physical
  // orbit radius (which would be thousands of km, meaningless at cubesat
  // scale). Defaults to the same "+Z is up" convention every other scenario
  // in this project uses.
  glm::vec3 sceneUp{0.0f, 0.0f, 1.0f};

  // World-frame ambient magnetic field (Tesla) at simulated mission time t.
  glm::vec3 sample(float simTime) const;

  // Unit vector from Earth's center toward the satellite's current orbital
  // position, in the same fixed inertial frame sample() uses -- exposed for
  // visualization (e.g. drawing an "orbit position" or field-line marker),
  // not itself a physical quantity a real sensor would read.
  glm::vec3 orbitPositionDirection(float simTime) const;

private:
  glm::vec3 dipoleMomentDirection() const;
};
