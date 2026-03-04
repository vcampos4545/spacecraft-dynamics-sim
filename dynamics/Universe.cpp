#include "Universe.h"
#include <cmath>
#include <stdexcept>

// ---------------------------------------------------------------------------
// Celestial management
// ---------------------------------------------------------------------------

CelestialBody &Universe::addCelestial(const std::string &name,
                                       double mass,
                                       double radius,
                                       glm::dvec3 position,
                                       glm::dvec3 velocity,
                                       bool fixed)
{
  celestials.push_back({name, mass, radius, position, velocity, fixed});
  return celestials.back();
}

// ---------------------------------------------------------------------------
// Rigid-body management
// ---------------------------------------------------------------------------

RigidBody *Universe::addBody(RigidBodyShape shape,
                              const glm::vec3 &size,
                              float mass)
{
  bodies_.push_back(RigidBody::fromMass(shape, size, mass));
  RigidBody *ptr = bodies_.back().get();
  bodyIndex_[ptr] = orbStates_.size();
  orbStates_.push_back({});
  return ptr;
}

void Universe::setOrbitalState(RigidBody *body, glm::dvec3 pos, glm::dvec3 vel)
{
  size_t idx       = bodyIndex_.at(body);
  orbStates_[idx]  = {pos, vel};
  body->position   = glm::vec3(pos); // keep vec3 in sync for ADCS / rendering
}

glm::dvec3 Universe::getOrbitalPosition(RigidBody *body) const
{
  return orbStates_.at(bodyIndex_.at(body)).pos;
}

glm::dvec3 Universe::getOrbitalVelocity(RigidBody *body) const
{
  return orbStates_.at(bodyIndex_.at(body)).vel;
}

// ---------------------------------------------------------------------------
// N-body gravity
// ---------------------------------------------------------------------------

glm::dvec3 Universe::gravAccelAtPos(const glm::dvec3 &pos) const
{
  glm::dvec3 acc{0.0};
  for (const auto &cb : celestials)
  {
    glm::dvec3 r  = cb.position - pos;
    double     r2 = glm::dot(r, r);
    if (r2 < 1e6) continue; // skip if < 1 km (prevents singularity on impact)
    double r3 = r2 * std::sqrt(r2);
    acc += (G * cb.mass / r3) * r;
  }
  return acc;
}

// ---------------------------------------------------------------------------
// RK4 sub-step
// ---------------------------------------------------------------------------

void Universe::stepRK4(double dt)
{
  const size_t nc = celestials.size();

  // --- Integrate non-fixed celestials (decoupled RK4) ----------------------
  // Each celestial evaluates gravity using all other celestials at their
  // start-of-step positions.  The O(h²) splitting error is negligible at the
  // sub-step sizes we use (≤ 60 s).
  std::vector<glm::dvec3> dPos(nc, glm::dvec3{0.0});
  std::vector<glm::dvec3> dVel(nc, glm::dvec3{0.0});

  for (size_t i = 0; i < nc; ++i)
  {
    if (celestials[i].fixed) continue;

    const glm::dvec3 p = celestials[i].position;
    const glm::dvec3 v = celestials[i].velocity;

    glm::dvec3 k1p = v;
    glm::dvec3 k1v = gravAccelAtPos(p);

    glm::dvec3 k2p = v + k1v * (dt * 0.5);
    glm::dvec3 k2v = gravAccelAtPos(p + k1p * (dt * 0.5));

    glm::dvec3 k3p = v + k2v * (dt * 0.5);
    glm::dvec3 k3v = gravAccelAtPos(p + k2p * (dt * 0.5));

    glm::dvec3 k4p = v + k3v * dt;
    glm::dvec3 k4v = gravAccelAtPos(p + k3p * dt);

    dPos[i] = (k1p + 2.0 * k2p + 2.0 * k3p + k4p) * (dt / 6.0);
    dVel[i] = (k1v + 2.0 * k2v + 2.0 * k3v + k4v) * (dt / 6.0);
  }

  // Apply celestial deltas (all at once to avoid ordering bias).
  for (size_t i = 0; i < nc; ++i)
  {
    celestials[i].position += dPos[i];
    celestials[i].velocity += dVel[i];
  }

  // --- Integrate rigid bodies (decoupled RK4) ------------------------------
  // Bodies use the now-updated celestial positions (end of this sub-step).
  // Rigid-body masses are negligible; they don't perturb celestials.
  for (auto &state : orbStates_)
  {
    const glm::dvec3 p = state.pos;
    const glm::dvec3 v = state.vel;

    glm::dvec3 k1p = v;
    glm::dvec3 k1v = gravAccelAtPos(p);

    glm::dvec3 k2p = v + k1v * (dt * 0.5);
    glm::dvec3 k2v = gravAccelAtPos(p + k1p * (dt * 0.5));

    glm::dvec3 k3p = v + k2v * (dt * 0.5);
    glm::dvec3 k3v = gravAccelAtPos(p + k2p * (dt * 0.5));

    glm::dvec3 k4p = v + k3v * dt;
    glm::dvec3 k4v = gravAccelAtPos(p + k3p * dt);

    state.pos += (k1p + 2.0 * k2p + 2.0 * k3p + k4p) * (dt / 6.0);
    state.vel += (k1v + 2.0 * k2v + 2.0 * k3v + k4v) * (dt / 6.0);
  }
}

// ---------------------------------------------------------------------------
// Main step
// ---------------------------------------------------------------------------

void Universe::step(double wallDt, int timeWarp, double maxOrbitalStep)
{
  // --- Orbital integration (time-warped, sub-stepped RK4) ------------------
  double orbDt = wallDt * timeWarp;
  while (orbDt > 0.0)
  {
    double h = std::min(orbDt, maxOrbitalStep);
    stepRK4(h);
    orbDt -= h;
  }

  // Sync body->position (vec3) from the high-precision orbital state.
  // This is used by ADCS, ForceGenerators, and rendering.
  for (auto &[ptr, idx] : bodyIndex_)
    ptr->position = glm::vec3(orbStates_[idx].pos);

  // --- Attitude integration (real-time, no warp) ---------------------------
  for (auto &body : bodies_)
    body->stepAttitude((float)wallDt);
}

// ---------------------------------------------------------------------------
// Static helpers
// ---------------------------------------------------------------------------

std::pair<glm::dvec3, glm::dvec3>
Universe::circularOrbit(const CelestialBody &around,
                         double altM,
                         double incRad,
                         double raanRad)
{
  double mu = G * around.mass;
  double r  = around.radius + altM;
  double v  = std::sqrt(mu / r);

  double cr = std::cos(raanRad), sr = std::sin(raanRad);
  double ci = std::cos(incRad),  si = std::sin(incRad);

  // Start at ascending node (argument of perigee = 0, true anomaly = 0).
  glm::dvec3 pos = around.position + glm::dvec3{r * cr, r * sr, 0.0};
  glm::dvec3 vel = {v * (-sr * ci), v * (cr * ci), v * si};

  return {pos, vel};
}

double Universe::altitude(const CelestialBody &around, glm::dvec3 pos)
{
  return glm::length(pos - around.position) - around.radius;
}

double Universe::orbitalPeriod(const CelestialBody &around,
                                glm::dvec3 pos,
                                glm::dvec3 vel)
{
  double mu = G * around.mass;
  glm::dvec3 relPos = pos - around.position;
  double r  = glm::length(relPos);
  double v2 = glm::dot(vel, vel);

  double energy = 0.5 * v2 - mu / r;
  if (energy >= 0.0) return 0.0; // hyperbolic / escape

  double a = -mu / (2.0 * energy);
  return 2.0 * M_PI * std::sqrt(a * a * a / mu);
}
