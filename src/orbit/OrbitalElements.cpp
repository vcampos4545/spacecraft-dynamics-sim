#include <rigidbody/orbit/OrbitalElements.h>
#include <cmath>

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
