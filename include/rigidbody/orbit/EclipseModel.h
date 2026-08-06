#pragma once
#include <glm/glm.hpp>

// Cylindrical Earth-shadow check: is a satellite in Earth's shadow right
// now? Treats the shadow as an infinite cylinder extending away from the
// Sun (not the true, slightly-converging conical umbra/penumbra), which
// slightly overestimates eclipse duration -- adequate for gating solar
// panel generation; the conical model is a documented future refinement
// if penumbra transition time ever matters here.
namespace EclipseModel
{
// True if the satellite at satPosEci (ECI, meters) is in Earth's
// cylindrical shadow, given the unit direction from Earth to the Sun
// (sunDirEci).
bool inEclipse(const glm::dvec3 &satPosEci, const glm::dvec3 &sunDirEci);
} // namespace EclipseModel
