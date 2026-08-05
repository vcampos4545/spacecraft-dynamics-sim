#pragma once
#include <glm/glm.hpp>
#include <random>

class RigidBody;

// A 3-axis magnetometer, modeled the same way IMU.h models a gyro/accel:
// turn-on bias (fixed per run), slow bias drift (random walk), and
// zero-mean Gaussian noise on top of the true field -- never exact ground
// truth. Default noise/bias magnitudes are order-of-magnitude figures for a
// commodity magnetoresistive sensor (e.g. Honeywell HMC5883L-class), not
// exact.
//
// Reports in the sensor's own (body-fixed) axes, matching real hardware --
// same reasoning as IMU::sample: a magnetometer can only sense the field
// projected onto its own axes, not some external frame. The ambient field
// itself is NOT computed here; it's sampled from a MagneticField model (see
// rigidbody/environment/MagneticField.h) and passed in by the scenario,
// since the field varies with orbital position/time, not something this
// sensor can know on its own.
class Magnetometer
{
public:
  glm::vec3 mountPositionBody{0.0f}; // sensor location, body frame (informational; field is ~uniform over a cubesat-sized body)

  // 1-sigma measurement noise, added every sample (Tesla).
  glm::vec3 noiseStd{50e-9f}; // ~50 nT, order-of-magnitude for a low-cost magnetometer

  // 1-sigma bias random-walk rate, applied every sample (Tesla per sqrt(second)).
  glm::vec3 biasDriftStd{5e-9f};

  // Turn-on bias range: at construction, each axis' bias is drawn uniformly
  // from [-range, range] (Tesla).
  glm::vec3 biasRange{200e-9f};

  explicit Magnetometer(glm::vec3 mountPositionBody = glm::vec3(0.0f));

  struct Reading
  {
    glm::vec3 field{0.0f}; // Tesla, body frame
  };

  // Samples the magnetometer given the ambient (world-frame) field at the
  // body's current location. Call once per control cycle with the elapsed
  // time since the last call.
  Reading sample(const RigidBody &body, const glm::vec3 &ambientFieldWorld, float dt);

private:
  std::mt19937 rng;
  glm::vec3 bias;

  float gaussian(float stdDev);
};
