#include <rigidbody/orbit/MoonModel.h>
#include <cmath>

namespace
{
constexpr double DEG2RAD = 3.14159265358979323846 / 180.0;
constexpr double J2000_JD = 2451545.0;
} // namespace

glm::dvec3 MoonModel::positionEci(double jd)
{
  double T = (jd - J2000_JD) / 36525.0;

  // Fundamental arguments (degrees) -- Meeus ch. 47.
  double L0 = std::fmod(218.3164477 + 481267.88123421 * T, 360.0);
  double meanAnomalySun = std::fmod(357.5291092 + 35999.0502909 * T, 360.0);
  double meanAnomalyMoon = std::fmod(134.9633964 + 477198.8675055 * T, 360.0);
  double meanElongation = std::fmod(297.8501921 + 445267.1114034 * T, 360.0);
  double argLatitude = std::fmod(93.2720950 + 483202.0175233 * T, 360.0);

  double Mr = meanAnomalyMoon * DEG2RAD;
  double Dr = meanElongation * DEG2RAD;
  double Fr = argLatitude * DEG2RAD;
  double Msr = meanAnomalySun * DEG2RAD;

  // Ecliptic longitude (degrees) -- dominant periodic terms only.
  double lonDeg = L0
                + 6.288774 * std::sin(Mr)
                + 1.274027 * std::sin(2.0 * Dr - Mr)
                + 0.658314 * std::sin(2.0 * Dr)
                + 0.213618 * std::sin(2.0 * Mr)
                - 0.185116 * std::sin(Msr)
                - 0.114332 * std::sin(2.0 * Fr);

  // Ecliptic latitude (degrees).
  double latDeg = 5.128122 * std::sin(Fr)
                + 0.280602 * std::sin(Mr + Fr)
                + 0.277693 * std::sin(Mr - Fr)
                + 0.173237 * std::sin(2.0 * Dr - Fr);

  // Distance (km).
  double distKm = 385000.56
                + (-20905.355) * std::cos(Mr)
                + (-3699.111) * std::cos(2.0 * Dr - Mr)
                + (-2955.968) * std::cos(2.0 * Dr);

  double lon = lonDeg * DEG2RAD;
  double lat = latDeg * DEG2RAD;
  double distM = distKm * 1000.0;

  // Ecliptic -> equatorial (ECI), mean J2000 obliquity.
  double eps = 23.439291111 * DEG2RAD;
  return glm::dvec3(
      distM * std::cos(lat) * std::cos(lon),
      distM * (std::cos(eps) * std::cos(lat) * std::sin(lon) - std::sin(eps) * std::sin(lat)),
      distM * (std::sin(eps) * std::cos(lat) * std::sin(lon) + std::cos(eps) * std::sin(lat)));
}

glm::dvec3 MoonModel::directionEci(double jd)
{
  return glm::normalize(positionEci(jd));
}
