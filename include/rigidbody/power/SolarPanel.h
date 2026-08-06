#pragma once
#include <glm/glm.hpp>

class RigidBody;

// A single flat, body-fixed solar panel/cell array. Generation follows the
// standard cosine law for a flat photovoltaic surface: power scales with
// the incident flux, the panel's area and conversion efficiency, and the
// cosine of the angle between the panel's outward normal and the direction
// to the sun -- zero (not negative) once the sun is behind the panel.
//
// Deliberately body-fixed (no gimbal/deployment tracking) -- real cubesats
// commonly do exactly this (body-mounted panels on multiple faces) rather
// than a single sun-tracking array, and it keeps generation a direct,
// legible function of attitude: which face happens to be sunward decides
// how much power comes in, the same way a real body-mounted panel works.
class SolarPanel
{
public:
  glm::vec3 normalBody; // outward-facing panel normal, body frame (normalized in the constructor)
  float areaM2;
  float efficiency; // 0-1, cell conversion efficiency (fraction of incident flux converted to electrical power)

  SolarPanel(glm::vec3 normalBodyIn, float areaM2In, float efficiencyIn);

  struct Reading
  {
    float powerW = 0.0f;          // generated power, >= 0
    float incidenceAngleDeg = 90.0f; // angle between panel normal and sun direction; 0 = normal incidence
  };

  // sunDirWorld: direction FROM the spacecraft TOWARD the sun, world frame
  // (need not be normalized). solarFluxWm2 defaults to the solar constant
  // at 1 AU (~1361 W/m^2) -- this sim has no orbital eclipse model, so
  // there's no shadow term; a scenario that wants one can zero the flux
  // itself while in eclipse.
  Reading sample(const RigidBody &body, const glm::vec3 &sunDirWorld, float solarFluxWm2 = 1361.0f) const;
};
