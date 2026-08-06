#pragma once
#include <glm/glm.hpp>

// Double-precision Earth-Centered Inertial (ECI) state vector -- the
// "truth" state a scenario's orbit propagates through, independent of any
// RigidBody's own float32 state. Double precision matters here
// specifically: a LEO radius (~6.9e6 m) leaves float32 with only
// meter-level precision, and that error compounds every integration step
// over a mission that can run for months. See rigidbody/orbit/
// OrbitPropagator.h for what advances this state, and bridge the result
// into a RigidBody's position each frame (a single non-accumulating cast
// to float) for local dynamics/rendering -- don't integrate the truth
// trajectory through RigidBody's own float32 state directly.
struct OrbitState
{
  glm::dvec3 position{0.0}; // m, ECI
  glm::dvec3 velocity{0.0}; // m/s, ECI
  double missionTimeS = 0.0; // seconds elapsed since the scenario's mission epoch
};
