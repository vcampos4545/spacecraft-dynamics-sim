#include <rigidbody/environment/MagneticField.h>
#include <glm/gtc/constants.hpp>
#include <cmath>

namespace
{
constexpr float EARTH_RADIUS_M = 6.371e6f;
// mu0 * |m_earth| / (4*pi), the standard dipole-field scale constant --
// reproduces the ~25-65 uT real-world LEO field magnitude at r ~ 6.9e6 m.
constexpr float DIPOLE_K = 7.94e15f; // T*m^3

// Builds a right-handed (x, y, up) orthonormal basis from an arbitrary up
// vector, so the orbit/dipole geometry below still makes sense if a
// scenario ever overrides sceneUp away from the default +Z.
void buildBasis(const glm::vec3 &up, glm::vec3 &x, glm::vec3 &y, glm::vec3 &z)
{
  z = glm::normalize(up);
  glm::vec3 helper = (std::abs(z.x) < 0.9f) ? glm::vec3(1, 0, 0) : glm::vec3(0, 1, 0);
  x = glm::normalize(glm::cross(helper, z));
  y = glm::cross(z, x);
}
} // namespace

glm::vec3 MagneticField::dipoleMomentDirection() const
{
  glm::vec3 x, y, z;
  buildBasis(sceneUp, x, y, z);
  float tilt = glm::radians(dipoleTiltDeg);
  // Fixed-inertial tilted dipole -- Earth's real dipole axis also slowly
  // precesses/rotates with the planet's spin, but that's a much slower
  // effect than the orbital motion this model is built to capture, so it's
  // left out (a simplification, not an oversight).
  return glm::normalize(std::sin(tilt) * x + std::cos(tilt) * z);
}

glm::vec3 MagneticField::orbitPositionDirection(float simTime) const
{
  glm::vec3 x, y, z;
  buildBasis(sceneUp, x, y, z);

  float period = std::max(orbitalPeriodS, 1.0f);
  float theta = glm::two_pi<float>() * simTime / period;
  float inc = glm::radians(inclinationDeg);

  // Circular inclined orbit, ascending node along x, inclination rotating
  // the orbital plane about x relative to the (x, y) equatorial plane.
  return glm::normalize(std::cos(theta) * x +
                        std::sin(theta) * (std::cos(inc) * y + std::sin(inc) * z));
}

glm::vec3 MagneticField::sample(float simTime) const
{
  glm::vec3 rHat = orbitPositionDirection(simTime);
  glm::vec3 m = dipoleMomentDirection();

  float r = EARTH_RADIUS_M + altitudeKm * 1000.0f;
  float scale = DIPOLE_K / (r * r * r);

  return scale * (3.0f * glm::dot(m, rHat) * rHat - m);
}
