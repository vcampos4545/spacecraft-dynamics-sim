#include <rigidbody/sensors/StarTracker.h>
#include <rigidbody/RigidBody.h>
#include <cmath>
#include <algorithm>

float StarTracker::gaussian(float stdDev)
{
  if (stdDev <= 0.0f)
    return 0.0f;
  std::normal_distribution<float> dist(0.0f, stdDev);
  return dist(rng);
}

StarTracker::Reading StarTracker::sample(const RigidBody &body, const glm::vec3 &sunDirWorld)
{
  Reading r;

  glm::vec3 boresightWorld = body.orientation * boresightBody;
  glm::vec3 sunDir = glm::normalize(sunDirWorld);
  float sunAngleDeg = glm::degrees(std::acos(glm::clamp(glm::dot(boresightWorld, sunDir), -1.0f, 1.0f)));
  if (sunAngleDeg < sunExclusionAngleDeg)
    return r; // blinded

  if (glm::length(body.angularVelocity) > maxSlewRateRadS)
    return r; // slewing too fast to centroid stars this exposure

  // Small-angle noise, same [1, 0.5*delta] quaternion approximation the
  // EKF itself uses for its multiplicative attitude corrections -- keeps
  // this reading's error model consistent with how the estimator
  // interprets it.
  glm::vec3 delta(gaussian(noiseStdRad), gaussian(noiseStdRad), gaussian(noiseStdRad));
  glm::quat deltaQ = glm::normalize(glm::quat(1.0f, 0.5f * delta.x, 0.5f * delta.y, 0.5f * delta.z));

  r.valid = true;
  r.attitude = glm::normalize(deltaQ * body.orientation);
  return r;
}
