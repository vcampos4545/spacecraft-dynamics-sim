#include <rigidbody/orbit/EclipseModel.h>
#include <rigidbody/orbit/OrbitFrames.h>

bool EclipseModel::inEclipse(const glm::dvec3 &satPosEci, const glm::dvec3 &sunDirEci)
{
  return inShadow(satPosEci, sunDirEci, glm::dvec3(0.0), OrbitFrames::EARTH_RADIUS_M);
}

bool EclipseModel::inShadow(const glm::dvec3 &position, const glm::dvec3 &lightDirFromPosition,
                             const glm::dvec3 &occluderPosition, double occluderRadiusM)
{
  glm::dvec3 rel = position - occluderPosition;
  double alongLightAxis = glm::dot(rel, lightDirFromPosition);
  if (alongLightAxis > 0.0)
    return false; // on the light-facing side of the occluder's center -- can't be in its shadow

  glm::dvec3 perp = rel - alongLightAxis * lightDirFromPosition;
  return glm::length(perp) < occluderRadiusM;
}
