#include <rigidbody/orbit/OrbitForceModel.h>

glm::dvec3 TwoBodyGravity::acceleration(const OrbitState &state, double /*t*/) const
{
  double rMag = glm::length(state.position);
  return -mu / (rMag * rMag * rMag) * state.position;
}

glm::dvec3 J2Perturbation::acceleration(const OrbitState &state, double /*t*/) const
{
  const glm::dvec3 &r = state.position;
  double rMag = glm::length(r);
  double rMag2 = rMag * rMag;
  double rMag5 = rMag2 * rMag2 * rMag;

  // Standard closed-form J2 acceleration (Vallado eq. 9-38), z along the
  // planet's rotation/dipole axis (same ECI convention OrbitFrames.h
  // uses).
  double factor = -1.5 * j2 * mu * planetRadiusM * planetRadiusM / rMag5;
  double zRatioTerm = 5.0 * (r.z * r.z) / rMag2;

  return glm::dvec3(
      factor * r.x * (1.0 - zRatioTerm),
      factor * r.y * (1.0 - zRatioTerm),
      factor * r.z * (3.0 - zRatioTerm));
}
