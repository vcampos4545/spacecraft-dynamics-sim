#pragma once
#include <glm/glm.hpp>
#include <cmath>
#include <algorithm>

// Earth geometry / frame conversion utilities: ECEF<->ECI, geodetic
// (lat/lon/alt) -> ECEF, Greenwich sidereal time, and ground-station
// elevation angle. Spherical Earth (not WGS84 ellipsoidal) -- adequate for
// this project's visualization/pointing accuracy needs; an ellipsoidal
// model is a documented future refinement if geodetic precision below
// ~0.1% of Earth's radius (a few km) ever matters here.
namespace OrbitFrames
{
constexpr double EARTH_RADIUS_M = 6.371e6;
constexpr double EARTH_ROTATION_RATE_RAD_S = 7.2921150e-5; // sidereal rotation rate
constexpr double DEG2RAD = 3.14159265358979323846 / 180.0;

// Geodetic latitude/longitude (degrees) + altitude (m) -> ECEF (m),
// spherical Earth.
inline glm::dvec3 geodeticToECEF(double latDeg, double lonDeg, double altM = 0.0)
{
  double lat = latDeg * DEG2RAD;
  double lon = lonDeg * DEG2RAD;
  double r = EARTH_RADIUS_M + altM;
  return glm::dvec3(r * std::cos(lat) * std::cos(lon),
                    r * std::cos(lat) * std::sin(lon),
                    r * std::sin(lat));
}

// Greenwich Mean Sidereal Time (radians) for Julian Date jd -- IAU 1982
// formula (Vallado, Alg 15), accurate to ~0.1 arcsec.
inline double gmstRad(double jd)
{
  double t = (jd - 2451545.0) / 36525.0; // Julian centuries since J2000
  double thetaDeg = 280.46061837
                   + 360.98564736629 * (jd - 2451545.0)
                   + 0.000387933 * t * t
                   - t * t * t / 38710000.0;
  thetaDeg = std::fmod(thetaDeg, 360.0);
  if (thetaDeg < 0.0)
    thetaDeg += 360.0;
  return thetaDeg * DEG2RAD;
}

// Rotates an ECEF vector into ECI by the Earth-rotation angle thetaGstRad
// (from gmstRad, for the current epoch).
inline glm::dvec3 ecefToECI(const glm::dvec3 &ecef, double thetaGstRad)
{
  double c = std::cos(thetaGstRad);
  double s = std::sin(thetaGstRad);
  return glm::dvec3(c * ecef.x - s * ecef.y,
                    s * ecef.x + c * ecef.y,
                    ecef.z);
}

// Inverse of ecefToECI.
inline glm::dvec3 eciToECEF(const glm::dvec3 &eci, double thetaGstRad)
{
  double c = std::cos(thetaGstRad);
  double s = std::sin(thetaGstRad);
  return glm::dvec3(c * eci.x + s * eci.y,
                    -s * eci.x + c * eci.y,
                    eci.z);
}

// Elevation angle (radians) of a satellite at rSatEci as seen from a
// ground point at rGroundEci -- positive means above the local horizon.
// Both vectors must be in the same frame (ECI here, but the formula is
// frame-agnostic as long as both share one).
inline double elevationAngleRad(const glm::dvec3 &rGroundEci, const glm::dvec3 &rSatEci)
{
  glm::dvec3 slant = rSatEci - rGroundEci;
  glm::dvec3 groundUp = glm::normalize(rGroundEci);
  glm::dvec3 slantDir = glm::normalize(slant);
  double sinElevation = std::clamp(glm::dot(groundUp, slantDir), -1.0, 1.0);
  return std::asin(sinElevation);
}
} // namespace OrbitFrames
