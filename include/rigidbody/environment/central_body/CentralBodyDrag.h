#pragma once

#include <rigidbody/ForceGenerator.h>
#include <glm/glm.hpp>

class RigidBody;

// Same quadratic drag law as UniformDrag (F = -0.5 * rho(h) * Cd * A * |v|
// * v, exponential density falloff), but altitude is measured as true
// distance from a central body's center minus its radius, instead of a
// flat world-Z offset -- the right notion of "altitude" once a body isn't
// confined to a fixed local "up" (an orbital or reentry trajectory that
// isn't aligned with any single world axis).
//
// Per-body (needs frontalAreaM2, like UniformDrag), added via
// RigidBody::addForceGenerator, not a global generator.
//
// Known simplification: same as UniformDrag -- frontal area is a fixed
// constant, not recomputed from orientation relative to velocity, and
// atmospheric co-rotation (the atmosphere itself moving with the planet's
// rotation) is not modeled.
class CentralBodyDrag : public ForceGenerator
{
public:
  float dragCoefficient = 0.5f; // Cd, dimensionless
  float frontalAreaM2;          // cross-sectional area (m^2)

  glm::vec3 centralBodyPositionWorld{0.0f};
  float planetRadiusM = 6.371e6f;         // Earth mean radius
  float seaLevelDensityKgM3 = 1.225f;
  float scaleHeightM = 8500.0f;           // atmosphere e-folding height (Earth, ~isothermal approx)

  explicit CentralBodyDrag(float frontalAreaM2);

  void apply(RigidBody &body, float dt) override;
};
