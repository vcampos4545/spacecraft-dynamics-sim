#pragma once
#include <rigidbody/RigidBody.h>
#include <rigidbody/PhysicsWorld.h>
#include <rigidbody/actuators/Thruster.h>
#include <vector>
#include <glm/glm.hpp>

// Super Heavy booster: 33 Raptor engines (13 gimbal-capable in the center,
// 20 fixed in the outer ring). Propellant depletes as engines fire, based
// on thrust and specific impulse; the body's mass/inertia are kept in sync
// via RigidBody::setMass every update().
//
// Dimensions/propellant/thrust are the figures given for Super Heavy; dry
// mass and engine Isp aren't part of that data and are commonly-cited
// estimates (see Booster.cpp) used only to make mass depletion physical.
class Booster
{
public:
  static constexpr float HEIGHT_M = 72.0f;
  static constexpr float DIAMETER_M = 9.0f;
  static constexpr float ENGINE_DIAMETER_M = 1.3f;

  RigidBody *body = nullptr;
  std::vector<Thruster *> centerEngines; // 13, gimbal-capable
  std::vector<Thruster *> outerEngines;  // 20, fixed

  float dryMassKg;
  float propellantCapacityKg;
  float propellantMassKg;

  Booster(PhysicsWorld &world, const glm::vec3 &position);

  // Fires every engine at the given throttle [0,1] (no-op once propellant
  // is exhausted), depletes propellant by the resulting mass flow, and
  // updates the body's mass/inertia to match. Call once per frame before
  // stepping the PhysicsWorld.
  void update(float throttle, float dt);

  float propellantFraction() const { return propellantMassKg / propellantCapacityKg; }

private:
  void refreshMass();
};
