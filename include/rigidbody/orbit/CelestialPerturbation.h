#pragma once
#include <rigidbody/orbit/OrbitForceModel.h>
#include <rigidbody/orbit/CelestialSystem.h>

// Gravitational perturbation from any body in a CelestialSystem, using its
// live current position -- generalizes ThirdBodyGravity's closed Sun/Moon
// enum (which calls SunModel/MoonModel internally) to any body the system
// knows about.
//
// This is deliberately NOT used for a body's own primary/parent -- an
// OrbitState propagated relative to its primary already has that gravity
// as plain TwoBodyGravity(mu = primary.mu), no live-position query needed,
// since the primary sits at the origin of that frame by construction. This
// class is only for *perturbers*: other bodies whose gravity must be added
// as the differential (tidal) term, exactly as ThirdBodyGravity's own
// header comment already derives:
//   a = mu_p * ( (r_p - r) / |r_p - r|^3  -  r_p / |r_p|^3 )
// where r_p is the perturber's position *relative to the same primary-
// centered frame the propagated OrbitState lives in* (so the frame's own
// non-inertial acceleration toward the perturber, which the primary itself
// feels, is correctly subtracted out).
class CelestialPerturbation : public OrbitForceModel
{
public:
  CelestialPerturbation(const CelestialSystem &system, const CelestialBody &primary, const CelestialBody &perturber);

  // JD at t=0 of the current OrbitPropagator::step() call -- refreshed by
  // the caller each cycle, same role ThirdBodyGravity::epochJd plays.
  double jd = 2451545.0;

  glm::dvec3 acceleration(const OrbitState &state, double t) const override;

private:
  const CelestialSystem &system_;
  const CelestialBody &primary_;
  const CelestialBody &perturber_;
};
