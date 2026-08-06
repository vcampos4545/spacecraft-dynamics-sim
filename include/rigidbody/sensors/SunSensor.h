#pragma once
#include <glm/glm.hpp>
#include <random>

class RigidBody;

// A coarse sun sensor: reports the sun's direction in body frame, the way
// a simple analog/photodiode sun sensor array would, as opposed to
// StarTracker's arcsecond-class absolute attitude. Modeled the same way
// Magnetometer/StarTracker are -- the true body-frame direction plus a
// small angular perturbation, not an oracle reading.
//
// Unlike a magnetometer, a sun sensor has an obvious real failure mode
// (the sun isn't always in view -- eclipse, wrong side of the bus) that
// this simplified model doesn't capture; `valid` is always true here. A
// scenario/estimator that wants eclipse-aware dropouts needs to gate this
// externally for now.
class SunSensor
{
public:
  // 1-sigma per-axis angular noise (radians), added as a small additive
  // perturbation to the true direction -- ~0.5 deg is representative of a
  // simple coarse sun sensor (fine sun sensors get much closer to
  // arcminute-class, but aren't what this models).
  float noiseStdRad = glm::radians(0.5f);

  struct Reading
  {
    glm::vec3 sunDirBody{0.0f}; // unit vector, body frame
    bool valid = false;
  };

  // sunDirWorld: unit (or non-unit, normalized internally) vector from the
  // body toward the sun, world frame.
  Reading sample(const RigidBody &body, const glm::vec3 &sunDirWorld);

private:
  std::mt19937 rng{std::random_device{}()};
  float gaussian(float stdDev);
};
