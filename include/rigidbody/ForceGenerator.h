#pragma once

class RigidBody;

class ForceGenerator
{
public:
  virtual void apply(RigidBody &body, float dt) = 0;
  virtual ~ForceGenerator() = default;
};