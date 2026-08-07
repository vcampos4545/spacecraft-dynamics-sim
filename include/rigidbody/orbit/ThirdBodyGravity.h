#pragma once
#include <rigidbody/orbit/OrbitForceModel.h>

// Third-body gravitational perturbation (Sun or Moon) -- the dominant
// non-J2 perturbation for many orbits, responsible for real effects a
// two-body+J2 model can't reproduce (long-period eccentricity/inclination
// oscillations, resonances at some altitudes).
//
// Perturbation acceleration in ECI:
//   a = mu_b * ( (r_b - r) / |r_b - r|^3  -  r_b / |r_b|^3 )
// The first term is direct attraction toward the third body; the second
// removes the acceleration Earth *itself* experiences from that body, so
// the result is the *differential* (tidal) acceleration relative to
// Earth -- the correct perturbation to add in an Earth-centered (ECI)
// frame, not the third body's raw attraction.
//
// `epochJd` is public and mutable rather than fixed at construction: this
// project's mission epoch is editable live from the Simulation tab (see
// satellite_adcs_sim.cpp's EpochControls), so the harness updates this
// field each frame to stay in sync rather than needing to reconstruct the
// force model whenever the epoch changes.
enum class ThirdBodyType
{
  Sun,
  Moon
};

class ThirdBodyGravity : public OrbitForceModel
{
public:
  double epochJd = 2451545.0; // J2000 default; set by the harness each frame

  explicit ThirdBodyGravity(ThirdBodyType body);

  glm::dvec3 acceleration(const OrbitState &state, double t) const override;

private:
  ThirdBodyType body_;
  double mu_; // gravitational parameter of the third body (m^3/s^2)
};
