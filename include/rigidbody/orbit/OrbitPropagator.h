#pragma once
#include <rigidbody/orbit/OrbitForceModel.h>
#include <vector>
#include <memory>

// Double-precision RK4 propagator for OrbitState -- the mission-duration
// truth trajectory, completely independent of PhysicsWorld::step(). Force
// models are added once at startup (see OrbitForceModel.h) and summed at
// each RK4 stage; toggling a model's own parameters mid-run is fine, but
// adding/removing models isn't thread-safe without external
// synchronization (matches this engine's existing rigidbody Propagator
// convention).
class OrbitPropagator
{
public:
  void addForceModel(std::unique_ptr<OrbitForceModel> model);

  // Advances state by dt seconds (mission time), in place. state.missionTimeS
  // is advanced along with it.
  void step(OrbitState &state, double dt) const;

  const std::vector<std::unique_ptr<OrbitForceModel>> &forceModels() const { return forces_; }

private:
  std::vector<std::unique_ptr<OrbitForceModel>> forces_;

  glm::dvec3 totalAcceleration(const OrbitState &state, double t) const;
};
