#include <rigidbody/environment/central_body/CentralBodyMagneticField.h>
#include <glm/gtc/constants.hpp>
#include <cmath>

namespace
{
// Builds a right-handed (x, y, up) orthonormal basis from an arbitrary up
// vector, so the dipole geometry below still makes sense if a scenario
// ever overrides rotationAxisWorld away from the default +Z.
void buildBasis(const glm::vec3 &up, glm::vec3 &x, glm::vec3 &y, glm::vec3 &z)
{
  z = glm::normalize(up);
  glm::vec3 helper = (std::abs(z.x) < 0.9f) ? glm::vec3(1, 0, 0) : glm::vec3(0, 1, 0);
  x = glm::normalize(glm::cross(helper, z));
  y = glm::cross(z, x);
}
} // namespace

glm::vec3 CentralBodyMagneticField::dipoleMomentDirection() const
{
  glm::vec3 x, y, z;
  buildBasis(rotationAxisWorld, x, y, z);
  float tilt = glm::radians(dipoleTiltDeg);
  // Fixed-inertial tilted dipole -- Earth's real dipole axis also slowly
  // precesses/rotates with the planet's spin, but that's a much slower
  // effect than orbital motion, so it's left out (a simplification, not
  // an oversight), same as UniformMagneticField.
  return glm::normalize(std::sin(tilt) * x + std::cos(tilt) * z);
}

glm::vec3 CentralBodyMagneticField::sample(const glm::vec3 &positionRelativeToCentralBody) const
{
  float r = glm::length(positionRelativeToCentralBody);
  if (r < 1.0f)
    return glm::vec3(0.0f); // at/inside the central body's center -- undefined direction

  glm::vec3 rHat = positionRelativeToCentralBody / r;
  glm::vec3 m = dipoleMomentDirection();

  float scale = dipoleScaleTm3 / (r * r * r);
  return scale * (3.0f * glm::dot(m, rHat) * rHat - m);
}
