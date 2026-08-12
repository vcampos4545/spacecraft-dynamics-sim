#include <rigidbody/orbit/CelestialSystem.h>

CelestialBody *CelestialSystem::addBody(const std::string &name, const CelestialBodyParams &params, CelestialBody *parent)
{
  auto body = std::make_unique<CelestialBody>();
  body->name = name;
  body->params = params;
  body->parent = parent;
  CelestialBody *raw = body.get();
  bodies_.push_back(std::move(body));
  return raw;
}

void CelestialSystem::step(double dt)
{
  for (auto &body : bodies_)
    if (!body->analyticPositionFn)
      body->orbitPropagator.step(body->orbitState, dt);
}

glm::dvec3 CelestialSystem::absolutePosition(const CelestialBody *body, double jd) const
{
  glm::dvec3 pos(0.0);
  for (const CelestialBody *b = body; b != nullptr; b = b->parent)
    pos += b->analyticPositionFn ? b->analyticPositionFn(jd) : b->orbitState.position;
  return pos;
}
