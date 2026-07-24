#pragma once

#include <rigidbody/ForceGenerator.h>
#include <glm/glm.hpp>

class RigidBody;

// Simple quadratic aerodynamic drag: F = -0.5 * rho(h) * Cd * A * |v| * v,
// opposing the body's world-frame velocity. Needs no external driving --
// unlike Thruster/ReactionWheel there's nothing to command, it just reads
// the body's own position/velocity each substep (matches the base
// ForceGenerator contract directly: register it once via
// RigidBody::addForceGenerator and it runs on its own).
//
// Unlike Gravity, this is a per-body generator, not a global one -- each
// body has its own frontalAreaM2, so it can't be shared across bodies the
// way a single uniform Gravity field can.
//
// Air density falls off exponentially with altitude above groundZ
// (rho = rho0 * exp(-h / scaleHeight)), so drag fades out to ~nothing once
// a vehicle is high enough for that to matter, without needing a full
// atmosphere model.
//
// Known simplification: frontal area is a fixed constant, not recomputed
// from the body's orientation relative to its velocity -- this ignores
// angle of attack (a tumbling or sideways-moving body gets the same drag
// area as one flying nose-first).
class Drag : public ForceGenerator
{
public:
  float dragCoefficient = 0.5f; // Cd, dimensionless
  float frontalAreaM2;          // cross-sectional area (m^2)

  float seaLevelDensityKgM3 = 1.225f;
  float scaleHeightM = 8500.0f; // atmosphere e-folding height (Earth, ~isothermal approx)
  float groundZ = 0.0f;         // world Z treated as sea level / altitude zero

  explicit Drag(float frontalAreaM2);

  void apply(RigidBody &body, float dt) override;
};
