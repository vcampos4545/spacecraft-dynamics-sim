#include "Starship.h"
#include <rigidbody/environment/uniform/UniformDrag.h>
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

// Isp isn't part of the supplied figures; these are commonly cited Raptor 2
// values for the sea-level and vacuum-optimized (RVac) variants, used only
// to convert thrust into a propellant mass-flow rate
// (mdot = thrust / (Isp * g0)).
constexpr float RAPTOR_ISP_SEALEVEL_S = 327.0f;
constexpr float RAPTOR_ISP_VACUUM_S = 378.0f;
constexpr float MASS_FLOW_SEALEVEL_KGPS = RAPTOR_THRUST_N / (RAPTOR_ISP_SEALEVEL_S * G0);
constexpr float MASS_FLOW_VACUUM_KGPS = RAPTOR_THRUST_N / (RAPTOR_ISP_VACUUM_S * G0);

constexpr float PROPELLANT_CAPACITY_KG = 1600.0f * 1000.0f; // 1600 t

// Dry mass isn't part of the supplied figures; ~120 t is a commonly cited
// estimate for Starship.
constexpr float DRY_MASS_KG = 120.0f * 1000.0f;
} // namespace

Starship::Starship(PhysicsWorld &world, const glm::vec3 &position)
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
  const int numCenter = 3;
  const float centerRadius = 0.9f;
  for (int i = 0; i < numCenter; i++)
  {
    float angle = (2.0f * glm::pi<float>() * i) / numCenter;
    glm::vec3 mount(centerRadius * std::cos(angle), centerRadius * std::sin(angle), baseZ);

    auto engine = std::make_unique<Thruster>(mount, glm::vec3(0, 0, 1), RAPTOR_THRUST_N);
    centerEngines.push_back(engine.get());
    body->addForceGenerator(std::move(engine));
  }

  const int numOuter = 3;
  const float outerRadius = 2.0f;
  for (int i = 0; i < numOuter; i++)
  {
    // Offset from the center engines so the two rings don't visually
    // overlap radially.
    float angle = glm::pi<float>() / 3.0f + (2.0f * glm::pi<float>() * i) / numOuter;
    glm::vec3 mount(outerRadius * std::cos(angle), outerRadius * std::sin(angle), baseZ);

    auto engine = std::make_unique<Thruster>(mount, glm::vec3(0, 0, 1), RAPTOR_THRUST_N);
    engine->gimbalLimit = 0.0f; // vacuum-optimized, fixed
    outerEngines.push_back(engine.get());
    body->addForceGenerator(std::move(engine));
  }

  // Simple atmospheric drag, frontal area = the ship's own cross-section.
  // Needs no driving from update() -- it reads body state automatically.
  float frontalAreaM2 = glm::pi<float>() * (DIAMETER_M * 0.5f) * (DIAMETER_M * 0.5f);
  body->addForceGenerator(std::make_unique<UniformDrag>(frontalAreaM2));
}

void Starship::update(float centerThrottle, float outerThrottle, float dt)
{
  if (propellantMassKg <= 0.0f)
  {
    propellantMassKg = 0.0f;
    centerThrottle = 0.0f; // no propellant left: force engines to zero throttle below
    outerThrottle = 0.0f;
  }
  else
  {
    centerThrottle = glm::clamp(centerThrottle, 0.0f, 1.0f);
    outerThrottle = glm::clamp(outerThrottle, 0.0f, 1.0f);

    float massFlow = float(centerEngines.size()) * MASS_FLOW_SEALEVEL_KGPS * centerThrottle
                    + float(outerEngines.size()) * MASS_FLOW_VACUUM_KGPS * outerThrottle;
    propellantMassKg = std::max(0.0f, propellantMassKg - massFlow * dt);
  }

  // Set the commanded throttle; RigidBody's normal per-substep
  // ForceGenerator dispatch (see RigidBody::integrate) applies it every
  // substep regardless of what this function does, so throttle must always
  // be (re)set here -- including to zero once propellant runs out -- rather
  // than skipped via an early return. Calling Thruster::apply() directly
  // would double-fire it on top of that automatic dispatch.
  for (auto *engine : centerEngines)
    engine->throttle = centerThrottle;
  for (auto *engine : outerEngines)
    engine->throttle = outerThrottle;

  refreshMass();
}

void Starship::refreshMass()
{
  body->setMass(dryMassKg + propellantMassKg);
}
