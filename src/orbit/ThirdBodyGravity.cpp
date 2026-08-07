#include <rigidbody/orbit/ThirdBodyGravity.h>
#include <rigidbody/orbit/SunModel.h>
#include <rigidbody/orbit/MoonModel.h>

namespace
{
constexpr double GM_SUN = 1.32712440018e20;  // m^3/s^2
constexpr double GM_MOON = 4.9048695e12;     // m^3/s^2
constexpr double SEC_PER_DAY = 86400.0;
} // namespace

ThirdBodyGravity::ThirdBodyGravity(ThirdBodyType body)
    : body_(body), mu_(body == ThirdBodyType::Sun ? GM_SUN : GM_MOON)
{
}

glm::dvec3 ThirdBodyGravity::acceleration(const OrbitState &state, double t) const
{
  double jd = epochJd + t / SEC_PER_DAY;
  glm::dvec3 rBody = (body_ == ThirdBodyType::Sun) ? SunModel::positionEci(jd) : MoonModel::positionEci(jd);

  glm::dvec3 d = rBody - state.position; // satellite -> third body
  double dMag = glm::length(d);
  double rBodyMag = glm::length(rBody);

  return mu_ * (d / (dMag * dMag * dMag) - rBody / (rBodyMag * rBodyMag * rBodyMag));
}
