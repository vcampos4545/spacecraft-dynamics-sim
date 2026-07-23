#include <rigidbody/sensors/IMU.h>
#include <rigidbody/RigidBody.h>
#include <cmath>
#include <algorithm>

IMU::IMU(glm::vec3 mountPosBody)
    : mountPositionBody(mountPosBody),
      rng(std::random_device{}())
{
  std::uniform_real_distribution<float> gyroDist(-1.0f, 1.0f);
  std::uniform_real_distribution<float> accelDist(-1.0f, 1.0f);
  gyroBias = glm::vec3(gyroDist(rng), gyroDist(rng), gyroDist(rng)) * gyroBiasRange;
  accelBias = glm::vec3(accelDist(rng), accelDist(rng), accelDist(rng)) * accelBiasRange;
}

float IMU::gaussian(float stdDev)
{
  if (stdDev <= 0.0f)
    return 0.0f;
  std::normal_distribution<float> dist(0.0f, stdDev);
  return dist(rng);
}

IMU::Reading IMU::sample(const RigidBody &body, const glm::vec3 &gravity, float dt)
{
  Reading r;
  float sqrtDt = std::sqrt(std::max(dt, 0.0f));

  // ---- Gyro: body-frame angular velocity ----
  glm::vec3 trueGyroBody = glm::inverse(body.orientation) * body.angularVelocity;

  gyroBias += glm::vec3(gaussian(gyroBiasDriftStd.x), gaussian(gyroBiasDriftStd.y), gaussian(gyroBiasDriftStd.z)) * sqrtDt;
  glm::vec3 gyroNoise(gaussian(gyroNoiseStd.x), gaussian(gyroNoiseStd.y), gaussian(gyroNoiseStd.z));
  r.gyro = trueGyroBody + gyroBias + gyroNoise;

  // ---- Accelerometer: specific force at the mount point, body frame ----
  glm::vec3 trueAccelBody(0.0f);
  if (hasPrevState && dt > 1e-6f)
  {
    glm::vec3 linearAccelWorld = (body.velocity - prevVelocity) / dt;
    glm::vec3 angularAccelWorld = (body.angularVelocity - prevAngularVelocity) / dt;
    glm::vec3 rWorld = body.orientation * mountPositionBody;

    // a_point = a_com + alpha x r + omega x (omega x r)
    glm::vec3 mountAccelWorld = linearAccelWorld
                              + glm::cross(angularAccelWorld, rWorld)
                              + glm::cross(body.angularVelocity, glm::cross(body.angularVelocity, rWorld));

    glm::vec3 specificForceWorld = mountAccelWorld - gravity;
    trueAccelBody = glm::inverse(body.orientation) * specificForceWorld;
  }

  accelBias += glm::vec3(gaussian(accelBiasDriftStd.x), gaussian(accelBiasDriftStd.y), gaussian(accelBiasDriftStd.z)) * sqrtDt;
  glm::vec3 accelNoise(gaussian(accelNoiseStd.x), gaussian(accelNoiseStd.y), gaussian(accelNoiseStd.z));
  r.accel = trueAccelBody + accelBias + accelNoise;

  prevVelocity = body.velocity;
  prevAngularVelocity = body.angularVelocity;
  hasPrevState = true;

  return r;
}
