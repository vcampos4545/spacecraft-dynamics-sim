#include "Booster.h"
#include <rigidbody/environment/Drag.h>
#include <glm/gtc/constants.hpp>
#include <algorithm>
#include <cmath>
#include <memory>

namespace
{
constexpr float G0 = 9.80665f;
constexpr float TF_TO_N = 1000.0f * G0; // 1 tonne-force -> newtons

constexpr float RAPTOR_THRUST_TF = 250.0f;
constexpr float RAPTOR_THRUST_N = RAPTOR_THRUST_TF * TF_TO_N;

// Isp isn't part of the supplied figures; 327s is the commonly cited
// Raptor 2 sea-level value, used only to convert thrust into a propellant
// mass-flow rate (mdot = thrust / (Isp * g0)). Super Heavy uses sea-level
// Raptors exclusively (no vacuum-optimized variant on the booster).
constexpr float RAPTOR_ISP_S = 327.0f;
constexpr float MASS_FLOW_PER_ENGINE_KGPS = RAPTOR_THRUST_N / (RAPTOR_ISP_S * G0);

constexpr float PROPELLANT_CAPACITY_KG = 3650.0f * 1000.0f; // 3650 t

// Dry mass isn't part of the supplied figures; ~200 t is a commonly cited
// estimate for Super Heavy.
constexpr float DRY_MASS_KG = 200.0f * 1000.0f;
} // namespace

Booster::Booster(PhysicsWorld &world, const glm::vec3 &position)
    : dryMassKg(DRY_MASS_KG),
      propellantCapacityKg(PROPELLANT_CAPACITY_KG),
      propellantMassKg(PROPELLANT_CAPACITY_KG)
{
  body = world.createBody(
      RigidBodyShape::CYLINDER,
      glm::vec3(DIAMETER_M * 0.5f, HEIGHT_M, 0.0f),
      dryMassKg + propellantMassKg);
  body->position = position;

  // Engines mount at the base (local -Z) and push along local +Z.
  const float baseZ = -HEIGHT_M * 0.5f;

  // SpaceX doesn't publish exact engine coordinates; these are
  // representative rings at the given counts, not the literal layout.
  const int numCenter = 13;
  const float centerRadius = 1.4f;
  for (int i = 0; i < numCenter; i++)
  {
    float angle = (2.0f * glm::pi<float>() * i) / numCenter;
    glm::vec3 mount(centerRadius * std::cos(angle), centerRadius * std::sin(angle), baseZ);

    auto engine = std::make_unique<Thruster>(mount, glm::vec3(0, 0, 1), RAPTOR_THRUST_N);
    centerEngines.push_back(engine.get());
    body->addForceGenerator(std::move(engine));
  }

  const int numOuter = 20;
  const float outerRadius = 3.4f;
  for (int i = 0; i < numOuter; i++)
  {
    float angle = (2.0f * glm::pi<float>() * i) / numOuter;
    glm::vec3 mount(outerRadius * std::cos(angle), outerRadius * std::sin(angle), baseZ);

    auto engine = std::make_unique<Thruster>(mount, glm::vec3(0, 0, 1), RAPTOR_THRUST_N);
    engine->gimbalLimit = 0.0f; // fixed, not gimbal-capable
    outerEngines.push_back(engine.get());
    body->addForceGenerator(std::move(engine));
  }

  // Simple atmospheric drag, frontal area = the booster's own cross-section.
  // Needs no driving from update() -- it reads body state automatically.
  float frontalAreaM2 = glm::pi<float>() * (DIAMETER_M * 0.5f) * (DIAMETER_M * 0.5f);
  body->addForceGenerator(std::make_unique<Drag>(frontalAreaM2));
}

void Booster::update(float throttle, float dt)
{
  if (propellantMassKg <= 0.0f)
  {
    propellantMassKg = 0.0f;
    throttle = 0.0f; // no propellant left: force engines to zero throttle below
  }
  else
  {
    throttle = glm::clamp(throttle, 0.0f, 1.0f);

    float engineCount = float(centerEngines.size() + outerEngines.size());
    float massFlow = engineCount * MASS_FLOW_PER_ENGINE_KGPS * throttle;
    propellantMassKg = std::max(0.0f, propellantMassKg - massFlow * dt);
  }

  // Set the commanded throttle; RigidBody's normal per-substep
  // ForceGenerator dispatch (see RigidBody::integrate) applies it every
  // substep regardless of what this function does, so throttle must always
  // be (re)set here -- including to zero once propellant runs out -- rather
  // than skipped via an early return. Calling Thruster::apply() directly
  // would double-fire it on top of that automatic dispatch.
  for (auto *engine : centerEngines)
    engine->throttle = throttle;
  for (auto *engine : outerEngines)
    engine->throttle = throttle;

  refreshMass();
}

void Booster::refreshMass()
{
  body->setMass(dryMassKg + propellantMassKg);
}
