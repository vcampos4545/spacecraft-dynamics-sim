#pragma once
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/quaternion.hpp>
#include "RigidBody.h"
#include <utility>

// -----------------------------------------------------------------------
// Abstract constraint base
// -----------------------------------------------------------------------
class Constraint
{
public:
  virtual ~Constraint() = default;

  // Apply one Sequential-Impulse iteration to satisfy this constraint.
  virtual void solve(float dt) = 0;

  // Returns true if this constraint connects the given pair of bodies.
  // Used to suppress GJK collision between constrained pairs.
  virtual bool connects(const RigidBody *a, const RigidBody *b) const
  {
    return false;
  }
};

// -----------------------------------------------------------------------
// FixedJoint — locks all 6 DOF between bodyA and bodyB.
//
// At construction the joint captures:
//   - localAnchorA / localAnchorB : world-space pivot transformed into each
//     body's local frame (the pivot is the connection point in world space).
//   - relativeOrientation         : q_A^-1 * q_B  (desired relative rotation).
//
// solve() runs one Sequential-Impulse iteration with Baumgarte
// position-error correction for both translation and rotation.
// -----------------------------------------------------------------------
class FixedJoint : public Constraint
{
public:
  RigidBody *bodyA = nullptr;
  RigidBody *bodyB = nullptr;

  // Baumgarte stabilisation coefficient [0..1].
  // Higher = snappier correction but can overshoot at low iteration counts.
  float beta = 0.2f;

  // worldPivot: the shared attachment point in world space at the moment
  // the two bodies are first connected (e.g. the top of the rocket cylinder
  // for the nose cone, or the base-rim point for a landing leg).
  FixedJoint(RigidBody *a, RigidBody *b, const glm::vec3 &worldPivot);

  void solve(float dt) override;
  bool connects(const RigidBody *a, const RigidBody *b) const override;

private:
  glm::vec3 localAnchorA;        // worldPivot in bodyA local frame
  glm::vec3 localAnchorB;        // worldPivot in bodyB local frame
  glm::quat relativeOrientation; // q_A^-1 * q_B at construction

  // World-space inverse inertia tensor for a body.
  static glm::mat3 worldInvI(const RigidBody &b);

  // Cross-product (skew-symmetric) matrix: skew(v)*u == cross(v, u).
  static glm::mat3 skew(const glm::vec3 &v);

  // Solve 3x3 linear system A*x = b via Cramer's rule.
  // Returns zero if A is (near-)singular.
  static glm::vec3 solve3(const glm::mat3 &A, const glm::vec3 &b);
};

// -----------------------------------------------------------------------
// DistanceConstraint — constrains the distance between two anchor points
// to restLength. Either body pointer may be nullptr for a fixed world anchor.
// unilateral = true makes it act as a string (max-distance only).
// -----------------------------------------------------------------------
class DistanceConstraint : public Constraint
{
public:
  RigidBody *bodyA = nullptr;
  RigidBody *bodyB = nullptr;

  float restLength = 0.0f;
  bool  unilateral = false;

  // Baumgarte stabilisation coefficient.
  float beta = 0.1f;

  // worldPivotA: attachment point on bodyA in world space at construction
  //              (or the fixed world position if bodyA = nullptr).
  // worldPivotB: same for bodyB.
  DistanceConstraint(RigidBody *a, RigidBody *b,
                     const glm::vec3 &worldPivotA,
                     const glm::vec3 &worldPivotB,
                     float restLength,
                     bool unilateral = false);

  void solve(float dt) override;

  // Returns true for the connected pair (skips GJK between them).
  bool connects(const RigidBody *a, const RigidBody *b) const override;

  // Returns the current world-space positions of both anchor points.
  std::pair<glm::vec3, glm::vec3> getWorldAnchors() const;

private:
  // Attachment point stored in each body's local frame
  // (or the raw world position when the corresponding body is nullptr).
  glm::vec3 localAnchorA;
  glm::vec3 localAnchorB;

  static glm::mat3 worldInvI(const RigidBody &b);
};
