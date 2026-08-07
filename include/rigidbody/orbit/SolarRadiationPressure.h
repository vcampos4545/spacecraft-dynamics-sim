#pragma once
#include <rigidbody/orbit/OrbitForceModel.h>

// Solar radiation pressure perturbation:
//   a = -P(r_sun) * Cr * (A/m) * r_hat_(sun->sat)
// where P(r_sun) is the radiation pressure at the satellite's actual
// distance from the Sun (P_1AU * (AU/r_sun)^2 -- inverse-square, same as
// solar flux), and Cr is the reflectivity coefficient (1.0 = perfect
// absorber, ~2.0 = perfect reflector; ~1.2-1.5 is typical for a real
// spacecraft surface, a mix of both). Zeroed during eclipse (cylindrical
// shadow model, same as EclipseModel's other consumer, EPS generation).
//
// reflectivity/areaM2/massKg are public and tunable; `epochJd` is public
// and mutable rather than fixed at construction for the same reason as
// ThirdBodyGravity's -- this project's mission epoch is editable live.
class SolarRadiationPressure : public OrbitForceModel
{
public:
  double reflectivity = 1.3; // Cr, dimensionless
  double areaM2;
  double massKg;
  double epochJd = 2451545.0; // J2000 default; set by the harness each frame

  SolarRadiationPressure(double areaM2, double massKg);

  glm::dvec3 acceleration(const OrbitState &state, double t) const override;
};
