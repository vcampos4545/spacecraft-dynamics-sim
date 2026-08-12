#include <rigidbody/orbit/CelestialPerturbation.h>

namespace
{
constexpr double SEC_PER_DAY = 86400.0;
}

CelestialPerturbation::CelestialPerturbation(const CelestialSystem &system, const CelestialBody &primary, const CelestialBody &perturber)
    : system_(system), primary_(primary), perturber_(perturber)
{
}

glm::dvec3 CelestialPerturbation::acceleration(const OrbitState &state, double t) const
{
  double thisJd = jd + t / SEC_PER_DAY;
  glm::dvec3 rPerturber = system_.absolutePosition(&perturber_, thisJd) - system_.absolutePosition(&primary_, thisJd);

  glm::dvec3 d = rPerturber - state.position; // propagated body -> perturber
  double dMag = glm::length(d);
  double rPerturberMag = glm::length(rPerturber);

  return perturber_.params.mu * (d / (dMag * dMag * dMag) - rPerturber / (rPerturberMag * rPerturberMag * rPerturberMag));
}
