#include <rigidbody/sensors/SunSensor.h>
#include <rigidbody/RigidBody.h>

float SunSensor::gaussian(float stdDev)
{
  if (stdDev <= 0.0f)
    return 0.0f;
  std::normal_distribution<float> dist(0.0f, stdDev);
  return dist(rng);
}

SunSensor::Reading SunSensor::sample(const RigidBody &body, const glm::vec3 &sunDirWorld)
{
  Reading r;

  glm::vec3 dirWorld = sunDirWorld;
  float len = glm::length(dirWorld);
  if (len < 1e-9f)
    return r; // no usable sun direction -- leave invalid

  dirWorld /= len;
  glm::vec3 dirBodyTrue = glm::inverse(body.orientation) * dirWorld;

  glm::vec3 noise(gaussian(noiseStdRad), gaussian(noiseStdRad), gaussian(noiseStdRad));
  r.sunDirBody = glm::normalize(dirBodyTrue + noise);
  r.valid = true;
  return r;
}
