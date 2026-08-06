#include <rigidbody/power/SolarPanel.h>
#include <rigidbody/RigidBody.h>
#include <glm/gtc/quaternion.hpp>
#include <cmath>
#include <algorithm>

SolarPanel::SolarPanel(glm::vec3 normalBodyIn, float areaM2In, float efficiencyIn)
    : normalBody(glm::normalize(normalBodyIn)),
      areaM2(areaM2In),
      efficiency(efficiencyIn)
{
}

SolarPanel::Reading SolarPanel::sample(const RigidBody &body, const glm::vec3 &sunDirWorld, float solarFluxWm2) const
{
  Reading r;

  float len = glm::length(sunDirWorld);
  if (len < 1e-9f)
    return r; // no usable sun direction -- report zero rather than divide by nothing

  glm::vec3 sunDir = sunDirWorld / len;
  glm::vec3 normalWorld = body.orientation * normalBody;

  float cosAngle = glm::dot(normalWorld, sunDir);
  r.incidenceAngleDeg = glm::degrees(std::acos(glm::clamp(cosAngle, -1.0f, 1.0f)));
  r.powerW = solarFluxWm2 * areaM2 * efficiency * std::max(0.0f, cosAngle);
  return r;
}
