#pragma once
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <random>

class RigidBody;

// A star tracker: images the star field along a fixed boresight and solves
// for absolute attitude by pattern-matching against an onboard catalog --
// modeled here as a direct (noisy) reading of the body's true orientation,
// not a simulated star field, since the catalog matching itself isn't what
// a flight-software-level sensor model needs to capture. Far more accurate
// than the coarse sun-sensor + magnetometer TRIAD this project also models
// (arcsecond-class here vs. the ~0.5 deg sun sensor), but with two real
// failure modes neither of those has: it goes blind if the sun is too
// close to the boresight, and it can't centroid stars while the body is
// slewing too fast. Both are modeled here since flight software has to
// treat "no valid reading this cycle" as a routine condition it estimates
// through, not an edge case -- see ADCS::propagateEstimator/
// correctEstimator, which fall back to the TRIAD measurement when this
// reports invalid.
class StarTracker
{
public:
  glm::vec3 boresightBody{0.0f, 0.0f, -1.0f}; // body-frame boresight direction (normalized)

  // 1-sigma per-axis attitude noise (radians) -- ~10 arcsec is a
  // representative figure for a small commercial cubesat-class tracker
  // (e.g. Blue Canyon NST-class hardware), not an exact spec. No bias
  // term the way IMU/Magnetometer have one: unlike a gyro or
  // magnetometer, a star tracker re-solves for *absolute* attitude from
  // scratch every frame, so there's nothing that accumulates.
  float noiseStdRad = glm::radians(10.0f / 3600.0f);

  float sunExclusionAngleDeg = 30.0f; // blinded if the boresight comes within this of the sun
  float maxSlewRateRadS = glm::radians(1.0f); // can't hold stars steady above this body rate

  struct Reading
  {
    bool valid = false;
    glm::quat attitude{1, 0, 0, 0}; // world frame; only meaningful if valid
  };

  // sunDirWorld: unit vector from the body toward the sun (world frame) --
  // the same quantity a scenario already derives from sunPosition/
  // body->position for SUN_POINTING guidance, just normalized first.
  Reading sample(const RigidBody &body, const glm::vec3 &sunDirWorld);

private:
  std::mt19937 rng{std::random_device{}()};
  float gaussian(float stdDev);
};
