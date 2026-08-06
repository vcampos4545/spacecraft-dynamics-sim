#include <rigidbody/orbit/OrbitPropagator.h>
#include <utility>

void OrbitPropagator::addForceModel(std::unique_ptr<OrbitForceModel> model)
{
  forces_.push_back(std::move(model));
}

glm::dvec3 OrbitPropagator::totalAcceleration(const OrbitState &state, double t) const
{
  glm::dvec3 total(0.0);
  for (const auto &force : forces_)
    total += force->acceleration(state, t);
  return total;
}

namespace
{
// One RK4 stage's derivative of the 6-element [position, velocity] state.
struct StateDerivative
{
  glm::dvec3 dPosition;
  glm::dvec3 dVelocity;
};
} // namespace

void OrbitPropagator::step(OrbitState &state, double dt) const
{
  double t0 = state.missionTimeS;

  auto derivative = [this](const OrbitState &s, double t) -> StateDerivative {
    return {s.velocity, totalAcceleration(s, t)};
  };

  StateDerivative k1 = derivative(state, t0);

  OrbitState s2 = state;
  s2.position = state.position + 0.5 * dt * k1.dPosition;
  s2.velocity = state.velocity + 0.5 * dt * k1.dVelocity;
  StateDerivative k2 = derivative(s2, t0 + 0.5 * dt);

  OrbitState s3 = state;
  s3.position = state.position + 0.5 * dt * k2.dPosition;
  s3.velocity = state.velocity + 0.5 * dt * k2.dVelocity;
  StateDerivative k3 = derivative(s3, t0 + 0.5 * dt);

  OrbitState s4 = state;
  s4.position = state.position + dt * k3.dPosition;
  s4.velocity = state.velocity + dt * k3.dVelocity;
  StateDerivative k4 = derivative(s4, t0 + dt);

  state.position += (dt / 6.0) * (k1.dPosition + 2.0 * k2.dPosition + 2.0 * k3.dPosition + k4.dPosition);
  state.velocity += (dt / 6.0) * (k1.dVelocity + 2.0 * k2.dVelocity + 2.0 * k3.dVelocity + k4.dVelocity);
  state.missionTimeS = t0 + dt;
}
