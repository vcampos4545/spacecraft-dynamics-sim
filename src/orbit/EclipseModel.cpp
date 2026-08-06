#include <rigidbody/orbit/EclipseModel.h>
#include <rigidbody/orbit/OrbitFrames.h>

bool EclipseModel::inEclipse(const glm::dvec3 &satPosEci, const glm::dvec3 &sunDirEci)
{
  double alongSunAxis = glm::dot(satPosEci, sunDirEci);
  if (alongSunAxis > 0.0)
    return false; // on the sunward side of Earth's center -- can't be in its shadow

  glm::dvec3 perp = satPosEci - alongSunAxis * sunDirEci;
  return glm::length(perp) < OrbitFrames::EARTH_RADIUS_M;
}
