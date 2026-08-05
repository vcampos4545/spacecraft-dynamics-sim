#include <rigidbody/sensors/Magnetometer.h>
#include <rigidbody/RigidBody.h>
#include <cmath>
#include <algorithm>

Magnetometer::Magnetometer(glm::vec3 mountPosBody)
    : mountPositionBody(mountPosBody),
      rng(std::random_device{}())
{
  std::uniform_real_distribution<float> dist(-1.0f, 1.0f);
  bias = glm::vec3(dist(rng), dist(rng), dist(rng)) * biasRange;
}

float Magnetometer::gaussian(float stdDev)
{
  if (stdDev <= 0.0f)
    return 0.0f;
  std::normal_distribution<float> dist(0.0f, stdDev);
  return dist(rng);
}

Magnetometer::Reading Magnetometer::sample(const RigidBody &body, const glm::vec3 &ambientFieldWorld, float dt)
{
  Reading r;
  float sqrtDt = std::sqrt(std::max(dt, 0.0f));

  glm::vec3 trueFieldBody = glm::inverse(body.orientation) * ambientFieldWorld;

  bias += glm::vec3(gaussian(biasDriftStd.x), gaussian(biasDriftStd.y), gaussian(biasDriftStd.z)) * sqrtDt;
  glm::vec3 noise(gaussian(noiseStd.x), gaussian(noiseStd.y), gaussian(noiseStd.z));

  r.field = trueFieldBody + bias + noise;
  return r;
}
