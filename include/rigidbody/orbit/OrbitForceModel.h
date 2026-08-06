#pragma once
#include <rigidbody/orbit/OrbitState.h>

// One term in the truth-propagation acceleration model -- OrbitPropagator
// sums every attached model's acceleration() each RK4 stage. Double
// precision throughout, independent of RigidBody/ForceGenerator (which
// operate on float32 RigidBody state for a different purpose -- see
// OrbitState.h).
class OrbitForceModel
{
public:
  virtual ~OrbitForceModel() = default;
  virtual glm::dvec3 acceleration(const OrbitState &state, double t) const = 0;
};

// Point-mass central-body gravity: a = -mu * r / |r|^3.
class TwoBodyGravity : public OrbitForceModel
{
public:
  double mu = 3.986004418e14; // m^3/s^2, Earth's standard gravitational parameter (GM)

  glm::dvec3 acceleration(const OrbitState &state, double t) const override;
};

// J2 (Earth oblateness) perturbation -- the dominant secular perturbation
// for a LEO orbit, responsible for real effects like nodal regression
// (the orbital plane precessing over time) that a pure point-mass model
// can't reproduce. Standard closed-form J2 acceleration (Vallado eq.
// 9-38); no higher-order zonal harmonics.
class J2Perturbation : public OrbitForceModel
{
public:
  double mu = 3.986004418e14;      // m^3/s^2
  double j2 = 1.08262668e-3;        // Earth's J2 coefficient, dimensionless
  double planetRadiusM = 6.378137e6; // Earth equatorial radius (WGS84)

  glm::dvec3 acceleration(const OrbitState &state, double t) const override;
};
