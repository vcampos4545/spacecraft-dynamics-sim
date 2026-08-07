#include <rigidbody/orbit/OrbitalElements.h>
#include <cmath>
#include <algorithm>

namespace
{
constexpr double TWO_PI = 6.28318530717958647692;
} // namespace

OrbitalElements OrbitalElements::circular(double altitudeM, double inclinationRad,
                                          double planetRadiusM, double raanRad,
                                          double trueAnomalyRad)
{
  OrbitalElements elements;
  elements.semiMajorAxisM = planetRadiusM + altitudeM;
  elements.eccentricity = 0.0;
  elements.inclinationRad = inclinationRad;
  elements.raanRad = raanRad;
  elements.argPeriapsisRad = 0.0; // meaningless for e = 0; fixed at zero
  elements.trueAnomalyRad = trueAnomalyRad;
  return elements;
}

OrbitState OrbitalElements::toState(double mu, double missionTimeS) const
{
  double p = semiMajorAxisM * (1.0 - eccentricity * eccentricity); // semi-latus rectum
  double cosNu = std::cos(trueAnomalyRad);
  double sinNu = std::sin(trueAnomalyRad);
  double rMag = p / (1.0 + eccentricity * cosNu);

  // Position/velocity in the perifocal (PQW) frame -- P toward periapsis,
  // W along the orbit normal, Q completing the right-handed triad. Both
  // lie exactly in the orbital plane (z = 0 in this frame).
  glm::dvec2 rPQW(rMag * cosNu, rMag * sinNu);
  glm::dvec2 vPQW(-std::sin(trueAnomalyRad), eccentricity + cosNu);
  vPQW *= std::sqrt(mu / p);

  // PQW -> ECI basis vectors P, Q (Vallado, "Fundamentals of Astrodynamics
  // and Applications", the classical R3(-raan)*R1(-i)*R3(-argPeriapsis)
  // rotation, expanded directly into the two basis vectors actually
  // needed rather than composing 3x3 rotation matrices -- less room for a
  // row/column transpose mistake).
  double cosO = std::cos(raanRad), sinO = std::sin(raanRad);
  double cosw = std::cos(argPeriapsisRad), sinw = std::sin(argPeriapsisRad);
  double cosi = std::cos(inclinationRad), sini = std::sin(inclinationRad);

  glm::dvec3 P(cosO * cosw - sinO * sinw * cosi,
              sinO * cosw + cosO * sinw * cosi,
              sinw * sini);
  glm::dvec3 Q(-cosO * sinw - sinO * cosw * cosi,
              -sinO * sinw + cosO * cosw * cosi,
              cosw * sini);

  OrbitState state;
  state.position = rPQW.x * P + rPQW.y * Q;
  state.velocity = vPQW.x * P + vPQW.y * Q;
  state.missionTimeS = missionTimeS;
  return state;
}

// Standard RV2COE algorithm (Vallado): angular momentum + node + eccentricity
// vectors give inclination/RAAN/argument-of-periapsis/eccentricity directly
// as vector angles; semi-major axis from the vis-viva energy equation; true
// anomaly from the angle between the eccentricity vector and position (with
// a radial-velocity sign check to resolve the pre/post-periapsis ambiguity,
// since acos() alone can't tell those apart). Degenerate cases (near-zero
// eccentricity or inclination, where RAAN/argument-of-periapsis are not
// well-defined) fall back to a sane zero rather than propagating NaN --
// this project's orbits are only ever near-circular/near-equatorial around
// exact commanded values, not literally exactly circular/equatorial, so
// these branches are a safety net more than an expected code path.
OrbitalElements OrbitalElements::fromState(const OrbitState &state, double mu)
{
  const glm::dvec3 &r = state.position;
  const glm::dvec3 &v = state.velocity;
  double rMag = glm::length(r);
  double vMag = glm::length(v);

  glm::dvec3 h = glm::cross(r, v); // specific angular momentum
  double hMag = glm::length(h);

  glm::dvec3 k(0.0, 0.0, 1.0);
  glm::dvec3 n = glm::cross(k, h); // ascending-node direction
  double nMag = glm::length(n);

  glm::dvec3 eVec = (1.0 / mu) * ((vMag * vMag - mu / rMag) * r - glm::dot(r, v) * v);
  double e = glm::length(eVec);

  double energy = 0.5 * vMag * vMag - mu / rMag;
  double a = (std::abs(energy) > 1e-10) ? -mu / (2.0 * energy) : hMag * hMag / mu;

  double inc = std::acos(std::clamp(h.z / hMag, -1.0, 1.0));

  double raan = 0.0;
  if (nMag > 1e-10)
  {
    raan = std::acos(std::clamp(n.x / nMag, -1.0, 1.0));
    if (n.y < 0.0)
      raan = TWO_PI - raan;
  }

  double argPeriapsis = 0.0;
  if (nMag > 1e-10 && e > 1e-10)
  {
    argPeriapsis = std::acos(std::clamp(glm::dot(n, eVec) / (nMag * e), -1.0, 1.0));
    if (eVec.z < 0.0)
      argPeriapsis = TWO_PI - argPeriapsis;
  }

  double trueAnomaly = 0.0;
  if (e > 1e-10)
  {
    trueAnomaly = std::acos(std::clamp(glm::dot(eVec, r) / (e * rMag), -1.0, 1.0));
    if (glm::dot(r, v) < 0.0)
      trueAnomaly = TWO_PI - trueAnomaly;
  }
  else if (nMag > 1e-10)
  {
    // Circular orbit: true anomaly isn't well-defined against a periapsis
    // that doesn't exist, so measure from the ascending node instead.
    trueAnomaly = std::acos(std::clamp(glm::dot(n, r) / (nMag * rMag), -1.0, 1.0));
    if (r.z < 0.0)
      trueAnomaly = TWO_PI - trueAnomaly;
  }

  OrbitalElements elements;
  elements.semiMajorAxisM = a;
  elements.eccentricity = e;
  elements.inclinationRad = inc;
  elements.raanRad = raan;
  elements.argPeriapsisRad = argPeriapsis;
  elements.trueAnomalyRad = trueAnomaly;
  return elements;
}

double OrbitalElements::periodS(double mu) const
{
  return TWO_PI * std::sqrt(semiMajorAxisM * semiMajorAxisM * semiMajorAxisM / mu);
}

double OrbitalElements::meanMotionRadS(double mu) const
{
  return std::sqrt(mu / (semiMajorAxisM * semiMajorAxisM * semiMajorAxisM));
}
