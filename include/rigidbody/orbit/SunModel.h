#pragma once
#include <glm/glm.hpp>

// Low-precision Sun position (the standard "Astronomical Almanac
// low-precision formula" -- a few arcminutes accurate near J2000,
// degrading slowly further from it), sufficient for eclipse checks and
// sun-pointing guidance; nowhere near the precision an actual ephemeris
// (e.g. JPL DE) would give, which this project has no need for.
namespace SunModel
{
// Sun position in ECI (meters) for Julian Date jd.
glm::dvec3 positionEci(double jd);

// Unit vector from Earth's center toward the Sun in ECI, for Julian Date
// jd.
glm::dvec3 directionEci(double jd);
} // namespace SunModel
