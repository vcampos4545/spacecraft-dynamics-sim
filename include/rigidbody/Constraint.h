#pragma once
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/quaternion.hpp>
#include <rigidbody/RigidBody.h>
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

  // Returns true if this constraint references the given body at all.
  // Used by PhysicsWorld::removeBody to drop constraints that would
  // otherwise be left holding a dangling pointer.
  virtual bool involves(const RigidBody *body) const
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
  bool involves(const RigidBody *body) const override;

private:
  glm::vec3 localAnchorA;        // worldPivot in bodyA local frame
  glm::vec3 localAnchorB;        // worldPivot in bodyB local frame
  glm::quat relativeOrientation; // q_A^-1 * q_B at construction
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
  bool involves(const RigidBody *body) const override;

  // Returns the current world-space positions of both anchor points.
  std::pair<glm::vec3, glm::vec3> getWorldAnchors() const;

private:
  // Attachment point stored in each body's local frame
  // (or the raw world position when the corresponding body is nullptr).
  glm::vec3 localAnchorA;
  glm::vec3 localAnchorB;
};

// -----------------------------------------------------------------------
// HingeConstraint — a 1-DOF revolute joint. bodyA and bodyB share a pivot
// point and rotate freely relative to each other about a common axis;
// translation and the two rotational DOF perpendicular to the axis are
// locked (like FixedJoint, but leaving one rotational degree of freedom
// open). Useful for anything that swings on a hinge — a deployable solar
// panel, an antenna boom, a landing leg with a folding joint.
//
// The hinge angle is bodyB's rotation relative to bodyA about the axis,
// zero at construction, and is available via getAngle(). Two optional
// mechanisms build on top of the bare joint:
//   - Angle limits (setLimits): a hard mechanical stop, e.g. a panel that
//     can swing from 0 (stowed) to pi/2 (deployed) and no further.
//   - A motor (enableMotor): drives the hinge toward a target angular rate,
//     capped by a max torque — e.g. a deployment spring/motor. Combined
//     with a limit, the motor drives the hinge open until it reaches the
//     stop and holds it there.
// -----------------------------------------------------------------------
class HingeConstraint : public Constraint
{
public:
  RigidBody *bodyA = nullptr;
  RigidBody *bodyB = nullptr;

  // Baumgarte stabilisation coefficients.
  float beta = 0.2f;      // pivot coincidence + axis alignment
  float limitBeta = 0.2f; // angle limit

  // worldPivot: shared attachment point in world space at construction.
  // worldAxis: hinge axis in world space at construction (need not be
  // unit length). Rotation about this axis is left free.
  HingeConstraint(RigidBody *a, RigidBody *b,
                  const glm::vec3 &worldPivot,
                  const glm::vec3 &worldAxis);

  void solve(float dt) override;
  bool connects(const RigidBody *a, const RigidBody *b) const override;
  bool involves(const RigidBody *body) const override;

  // Current hinge angle (radians): bodyB's rotation relative to bodyA
  // about the hinge axis, zero at construction, signed by the right-hand
  // rule around the axis (in bodyA's current frame).
  float getAngle() const;

  // Hard mechanical stop on the angle. Disabled (free rotation) until
  // this is called.
  void setLimits(float lowerRad, float upperRad);
  void clearLimits();

  // Simple velocity-servo motor (à la Bullet's btHingeConstraint motor):
  // drives the hinge's angular rate toward targetSpeed, limited by
  // maxTorque. Disabled by default.
  void enableMotor(bool enabled);
  void setMotorTargetSpeed(float speedRadPerSec);
  void setMotorMaxTorque(float torqueNm);

private:
  glm::vec3 localAnchorA;
  glm::vec3 localAnchorB;
  glm::vec3 localAxisA;
  glm::vec3 localAxisB;
  glm::vec3 localPerpA; // reference vector, perpendicular to the axis, used to measure the angle
  glm::vec3 localPerpB;

  bool  limitsEnabled = false;
  float lowerLimit = 0.0f;
  float upperLimit = 0.0f;

  bool  motorEnabled = false;
  float motorTargetSpeed = 0.0f;
  float motorMaxTorque = 0.0f;

  // Current hinge axis in world space (from bodyA's current orientation).
  glm::vec3 axisWorld() const;

  // Current axis (world space, from bodyA) and signed angle in one pass.
  void computeAxisAndAngle(glm::vec3 &axisOut, float &angleOut) const;
};
