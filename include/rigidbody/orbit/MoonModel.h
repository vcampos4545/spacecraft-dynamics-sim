#pragma once
#include <glm/glm.hpp>

// Low-precision Moon position (Meeus, "Astronomical Algorithms" ch. 47,
// reduced to the dominant periodic terms -- ~0.3 deg accurate, adequate
// for third-body perturbation magnitude and visualization; nowhere near
// the precision a real lunar ephemeris would give, which this project has
// no need for). Same role/precision tier as SunModel.h.
namespace MoonModel
{
// Moon position in ECI (meters) for Julian Date jd.
glm::dvec3 positionEci(double jd);

// Unit vector from Earth's center toward the Moon in ECI, for Julian Date
// jd.
glm::dvec3 directionEci(double jd);
} // namespace MoonModel
