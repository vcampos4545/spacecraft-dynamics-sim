#pragma once
#include <glm/glm.hpp>

// Same tilted-dipole model as UniformMagneticField (Earth's real field to
// first order), but sampled at a real position instead of a fake kinematic
// orbit phase -- for a scenario that has an actual position to sample
// against (e.g. from rigidbody/orbit/'s double-precision propagator). Not
// a ForceGenerator, same as UniformMagneticField: it doesn't act on a body
// directly, it's an ambient field a scenario samples and feeds to
// Magnetorquer::ambientFieldWorld / Magnetometer::sample().
//
// Field magnitude/geometry uses the standard dipole formula
//   B(r) = (K / r^3) * (3*(m . r_hat)*r_hat - m)
// with K = mu0*|m_earth|/(4*pi) ~= 7.94e15 T*m^3 by default, reproducing
// the right order of magnitude (~25-65 uT) for real LEO field strength --
// same constant UniformMagneticField uses, exposed here since a non-Earth
// scenario needs a different one.
class CentralBodyMagneticField
{
public:
  float dipoleTiltDeg = 11.0f; // dipole axis tilt from the central body's rotation axis
  float dipoleScaleTm3 = 7.94e15f; // K, T*m^3 -- see header comment

  // Central body's rotation axis in world frame, unit length. The dipole
  // axis is tilted `dipoleTiltDeg` away from this. Defaults to the same
  // "+Z is up" convention every scenario in this project uses.
  glm::vec3 rotationAxisWorld{0.0f, 0.0f, 1.0f};

  // World-frame ambient magnetic field (Tesla) at a position given
  // relative to the central body's center (i.e. body position minus
  // central body position -- the caller does that subtraction, this class
  // has no notion of where the central body itself is).
  glm::vec3 sample(const glm::vec3 &positionRelativeToCentralBody) const;

private:
  glm::vec3 dipoleMomentDirection() const;
};
