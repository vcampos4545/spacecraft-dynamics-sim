#pragma once
#include <rigidbody/RigidBody.h>
#include <rigidbody/PhysicsWorld.h>
#include <rigidbody/actuators/Thruster.h>
#include <vector>
#include <glm/glm.hpp>

// Starship upper stage: 6 Raptor engines (3 gimbal-capable sea-level
// engines in the center, 3 fixed vacuum-optimized engines in the outer
// ring). Propellant depletes as engines fire, based on thrust and specific
// impulse; the body's mass/inertia are kept in sync via
// RigidBody::setMass every update().
//
// Dimensions/propellant/thrust are the figures given for Starship; dry
// mass and engine Isp aren't part of that data and are commonly-cited
// estimates (see Starship.cpp) used only to make mass depletion physical.
class Starship
{
public:
  static constexpr float HEIGHT_M = 52.0f;
  static constexpr float DIAMETER_M = 9.0f;
  static constexpr float ENGINE_DIAMETER_M = 1.3f;

  RigidBody *body = nullptr;
  std::vector<Thruster *> centerEngines; // 3, gimbal-capable, sea-level
  std::vector<Thruster *> outerEngines;  // 3, fixed, vacuum-optimized

  float dryMassKg;
  float propellantCapacityKg;
  float propellantMassKg;

  Starship(PhysicsWorld &world, const glm::vec3 &position);

  // Fires each engine group at its own throttle [0,1] (no-op once
  // propellant is exhausted), depletes propellant by the resulting mass
  // flow, and updates the body's mass/inertia to match. Call once per
  // frame before stepping the PhysicsWorld. Real vehicles command these
  // groups independently too -- e.g. a landing burn only lights a handful
  // of sea-level-optimized (centerEngines) engines, never the
  // vacuum-optimized outerEngines. Ascent scenarios that want every engine
  // firing together can just pass the same value for both.
  void update(float centerThrottle, float outerThrottle, float dt);

  float propellantFraction() const { return propellantMassKg / propellantCapacityKg; }

private:
  void refreshMass();
};
