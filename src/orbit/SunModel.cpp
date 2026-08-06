#include <rigidbody/orbit/SunModel.h>
#include <cmath>

namespace
{
constexpr double DEG2RAD = 3.14159265358979323846 / 180.0;
constexpr double AU_M = 1.495978707e11;
} // namespace

glm::dvec3 SunModel::positionEci(double jd)
{
  double n = jd - 2451545.0; // days since J2000

  double meanLongitudeDeg = 280.460 + 0.9856474 * n;
  double meanAnomalyDeg = 357.528 + 0.9856003 * n;
  double g = meanAnomalyDeg * DEG2RAD;

  double eclipticLonDeg = meanLongitudeDeg + 1.915 * std::sin(g) + 0.020 * std::sin(2.0 * g);
  double eclipticLon = eclipticLonDeg * DEG2RAD;

  double obliquityDeg = 23.439 - 0.0000004 * n;
  double obliquity = obliquityDeg * DEG2RAD;

  glm::dvec3 dirEci(std::cos(eclipticLon),
                    std::cos(obliquity) * std::sin(eclipticLon),
                    std::sin(obliquity) * std::sin(eclipticLon));

  double distanceAU = 1.00014 - 0.01671 * std::cos(g) - 0.00014 * std::cos(2.0 * g);
  return dirEci * (distanceAU * AU_M);
}

glm::dvec3 SunModel::directionEci(double jd)
{
  return glm::normalize(positionEci(jd));
}
