#pragma once
#include <glm/glm.hpp>
#include <random>

class RigidBody;

// A 3-axis accelerometer + 3-axis gyro, modeled after commodity MEMS IMUs
// like the InvenSense MPU6050: every reading is corrupted by a per-run
// fixed turn-on bias (drawn once at construction), slow bias drift (a
// small random walk), and zero-mean Gaussian noise -- never exact ground
// truth. Default noise/bias magnitudes are order-of-magnitude MPU6050
// datasheet figures, not exact.
//
// Both channels report in the IMU's own (body-fixed) axes, matching real
// hardware -- a rate gyro can only sense rotation about its own axes, not
// some external frame. RigidBody::angularVelocity/velocity are stored in
// WORLD frame internally (see RigidBody::integrateAngular's left-multiply
// quaternion kinematics), so sample() rotates into the body frame before
// adding sensor error.
//
// The accelerometer measures specific force (proper acceleration minus
// gravity) at the IMU's mount point, which is generally not the body's
// center of mass, so it also picks up centripetal/tangential terms from
// angular velocity/acceleration -- exactly like a board bolted off-center
// inside a cubesat bus. RigidBody doesn't expose linear/angular
// acceleration directly, so both are estimated by finite-differencing
// velocity between samples (meaning the very first sample after
// construction has nothing to difference against and reads zero).
class IMU
{
public:
  glm::vec3 mountPositionBody{0.0f}; // IMU location, body frame

  // 1-sigma measurement noise, added every sample.
  glm::vec3 gyroNoiseStd{0.0008f};   // rad/s  (~0.05 deg/s)
  glm::vec3 accelNoiseStd{0.02f};    // m/s^2

  // 1-sigma bias random-walk rate, applied every sample (0 disables drift).
  glm::vec3 gyroBiasDriftStd{0.0003f};  // rad/s per sqrt(second)
  glm::vec3 accelBiasDriftStd{0.005f};  // m/s^2 per sqrt(second)

  // Turn-on bias range: at construction, each axis' bias is drawn uniformly
  // from [-range, range].
  glm::vec3 gyroBiasRange{0.01f};  // rad/s  (~0.6 deg/s)
  glm::vec3 accelBiasRange{0.2f};  // m/s^2

  explicit IMU(glm::vec3 mountPositionBody = glm::vec3(0.0f));

  struct Reading
  {
    glm::vec3 gyro{0.0f};  // rad/s, body frame
    glm::vec3 accel{0.0f}; // m/s^2, body frame (specific force)
  };

  // Samples the IMU at the current body state, given the ambient (uniform)
  // gravity the body is in -- an accelerometer reads proper acceleration,
  // not coordinate acceleration, so gravity must be subtracted out to get
  // what the sensor would actually report (zero in free fall). Call once
  // per control cycle with the elapsed time since the last call.
  Reading sample(const RigidBody &body, const glm::vec3 &gravity, float dt);

private:
  std::mt19937 rng;
  glm::vec3 gyroBias;
  glm::vec3 accelBias;

  bool hasPrevState = false;
  glm::vec3 prevVelocity{0.0f};
  glm::vec3 prevAngularVelocity{0.0f};

  float gaussian(float stdDev);
};
