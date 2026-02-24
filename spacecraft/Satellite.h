#pragma once
#include <vector>
#include "ADCS.h"

class PhysicsWorld;
class RigidBody;
class ReactionWheel;
class Thruster;

class Satellite
{
public:
  Satellite(PhysicsWorld *world);

  void update(float dt);

  RigidBody &getBody() { return *body; }
  ADCS adcs;

  const std::vector<ReactionWheel *> &getWheels() const { return wheels; };

private:
  // Physics component references
  RigidBody *body = nullptr;
  std::vector<ReactionWheel *> wheels; // non-owning; RigidBody owns the ForceGenerators

  // Subsystems
  float adcsTimer = 0;
};