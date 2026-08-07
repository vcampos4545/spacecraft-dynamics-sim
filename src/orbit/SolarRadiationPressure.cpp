#include <rigidbody/orbit/SolarRadiationPressure.h>
#include <rigidbody/orbit/SunModel.h>
#include <rigidbody/orbit/EclipseModel.h>

namespace
{
constexpr double AU_M = 1.495978707e11;
constexpr double SOLAR_PRESSURE_PA_AT_1AU = 4.56e-6; // N/m^2
constexpr double SEC_PER_DAY = 86400.0;
} // namespace

SolarRadiationPressure::SolarRadiationPressure(double areaM2, double massKg) : areaM2(areaM2), massKg(massKg)
{
}

glm::dvec3 SolarRadiationPressure::acceleration(const OrbitState &state, double t) const
{
  double jd = epochJd + t / SEC_PER_DAY;
  glm::dvec3 sunPos = SunModel::positionEci(jd);
  glm::dvec3 sunDir = glm::normalize(sunPos);

  if (EclipseModel::inEclipse(state.position, sunDir))
    return glm::dvec3(0.0);

  double rSunSat = glm::length(sunPos - state.position);
  double pressure = SOLAR_PRESSURE_PA_AT_1AU * (AU_M / rSunSat) * (AU_M / rSunSat);

  glm::dvec3 satFromSun = glm::normalize(state.position - sunPos);
  double coeff = pressure * reflectivity * areaM2 / massKg;
  return coeff * satFromSun;
}
