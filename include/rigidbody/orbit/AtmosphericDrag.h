#pragma once
#include <rigidbody/orbit/OrbitForceModel.h>

// Atmospheric drag perturbation on the double-precision truth propagator
// (a distinct thing from CentralBodyDrag/UniformDrag, which apply to a
// float32 RigidBody in PhysicsWorld -- same underlying physics, different
// consumer/precision, matching this project's existing "orbit/ vs
// environment/" split):
//   a = -0.5 * rho(alt) * Cd * (A/m) * |v_rel| * v_rel
// where v_rel is velocity relative to the atmosphere, which co-rotates
// with the planet (not relative to inertial/ECI velocity directly).
//
// dragCoefficient/areaM2/massKg are public and tunable -- areaM2/massKg
// have no sensible generic default (they're specific to whatever
// spacecraft the scenario is actually flying), so the constructor
// requires them; dragCoefficient defaults to 2.2, a representative value
// for a convex body dominated by free-molecular flow (LEO altitudes),
// per standard practice absent a computed value for the specific shape.
class AtmosphericDrag : public OrbitForceModel
{
public:
  double dragCoefficient = 2.2; // Cd, dimensionless
  double areaM2;
  double massKg;
  double planetRadiusM = 6.378137e6;      // Earth equatorial radius (WGS84)
  double planetRotationRateRadS = 7.2921150e-5; // sidereal rotation rate, for atmosphere co-rotation

  AtmosphericDrag(double areaM2, double massKg);

  glm::dvec3 acceleration(const OrbitState &state, double t) const override;
};
