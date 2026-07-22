#pragma once
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/quaternion.hpp>
#include <rigidbody/RigidBody.h>
#include <utility>

// -----------------------------------------------------------------------
// Constraint types
//
// Four primitives, each locking a specific set of the 6 relative degrees
// of freedom (3 translational + 3 rotational) between two bodies:
//
//   FixedConstraint  — 0 DOF free  (6 removed): point lock + orientation lock
//   PointConstraint  — 3 DOF free  (3 removed): point lock only
//   HingeConstraint  — 1 DOF free  (5 removed): point lock + axis alignment
//   SliderConstraint — 1 DOF free  (5 removed): orientation lock + perpendicular lock
//
// They share their underlying math (see the anonymous-namespace solvers in
// Constraint.cpp): FixedConstraint = PointConstraint + full orientation
// lock; HingeConstraint = PointConstraint + a 2-DOF axis-alignment lock;
// SliderConstraint = full orientation lock + a 2-DOF perpendicular
// translation lock. More elaborate joints can be built the same way two
// of these primitives combine — e.g. a universal joint is two
// HingeConstraints sharing a pivot with perpendicular axes, and a
// cylindrical joint is a Hinge and a Slider sharing an axis — or simply by
// adding more than one constraint between the same pair of bodies.
//
// DistanceConstraint (a rope/strut: anchors held at a target *distance*
// rather than coincident) is a fifth, distinct constraint kept alongside
// these four — it isn't expressible as a combination of them.
// -----------------------------------------------------------------------

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
// FixedConstraint (Weld) — locks all 6 relative DOF between bodyA and
// bodyB: relative position (3) and relative orientation (3). Removes all
// relative motion, as if the two bodies were a single rigid body.
//
// Uses: rocket body <-> nose cone, multi-part static assemblies,
// spacecraft bus components.
// -----------------------------------------------------------------------
class FixedConstraint : public Constraint
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
  FixedConstraint(RigidBody *a, RigidBody *b, const glm::vec3 &worldPivot);

  void solve(float dt) override;
  bool connects(const RigidBody *a, const RigidBody *b) const override;
  bool involves(const RigidBody *body) const override;

private:
  glm::vec3 localAnchorA;        // worldPivot in bodyA local frame
  glm::vec3 localAnchorB;        // worldPivot in bodyB local frame
  glm::quat relativeOrientation; // q_A^-1 * q_B at construction
};

// -----------------------------------------------------------------------
// PointConstraint (Point-to-Point / Ball Socket) — constrains a pivot
// point on bodyA to coincide with a pivot point on bodyB (3 DOF removed).
// Free rotation about the pivot in every direction. One of the most useful
// primitive constraints — a building block for ropes, chains, ragdolls,
// and (paired up) universal joints.
//
// Uses: rope/chain links, ragdolls, universal-joint building block.
// -----------------------------------------------------------------------
class PointConstraint : public Constraint
{
public:
  RigidBody *bodyA = nullptr;
  RigidBody *bodyB = nullptr;

  float beta = 0.2f;

  // worldPivot: the shared attachment point in world space at construction.
  PointConstraint(RigidBody *a, RigidBody *b, const glm::vec3 &worldPivot);

  void solve(float dt) override;
  bool connects(const RigidBody *a, const RigidBody *b) const override;
  bool involves(const RigidBody *body) const override;

private:
  glm::vec3 localAnchorA;
  glm::vec3 localAnchorB;
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
// HingeConstraint (Revolute) — a 1-DOF joint. bodyA and bodyB share a
// pivot point and rotate freely relative to each other about a common
// axis; the pivot (3 DOF) and the two rotational DOF perpendicular to the
// axis are locked, removing 5 of the 6 relative DOF.
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
//
// Uses: doors, solar panels, wheels, bearings, robot joints.
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

// -----------------------------------------------------------------------
// SliderConstraint (Prismatic) — a 1-DOF joint allowing translation along
// a single shared axis and nothing else: relative orientation (3 DOF) and
// the two translational DOF perpendicular to the axis are locked,
// removing 5 of the 6 relative DOF.
//
// The slide position is bodyB's anchor displaced from bodyA's anchor along
// the axis, zero at construction, available via getPosition(). As with
// HingeConstraint, optional limits (setLimits, a hard travel stop) and a
// motor (enableMotor, a velocity-servo linear actuator) build on the bare
// joint.
//
// Uses: pistons, telescoping antennas/booms, linear actuators.
// -----------------------------------------------------------------------
class SliderConstraint : public Constraint
{
public:
  RigidBody *bodyA = nullptr;
  RigidBody *bodyB = nullptr;

  float beta = 0.2f;      // orientation lock + perpendicular-translation lock
  float limitBeta = 0.2f; // position limit

  // worldPivot: shared reference point in world space at construction.
  // worldAxis: slide axis in world space at construction (need not be
  // unit length). Translation along this axis is left free; relative
  // orientation is locked to whatever it is at construction.
  SliderConstraint(RigidBody *a, RigidBody *b,
                   const glm::vec3 &worldPivot,
                   const glm::vec3 &worldAxis);

  void solve(float dt) override;
  bool connects(const RigidBody *a, const RigidBody *b) const override;
  bool involves(const RigidBody *body) const override;

  // Current slide position (meters): bodyB's anchor displaced from
  // bodyA's anchor along the axis, zero at construction.
  float getPosition() const;

  // Hard mechanical stop on the slide position (travel limits). Disabled
  // (free sliding) until this is called.
  void setLimits(float lowerM, float upperM);
  void clearLimits();

  // Simple velocity-servo motor: drives the slide rate toward targetSpeed,
  // limited by maxForce. Disabled by default.
  void enableMotor(bool enabled);
  void setMotorTargetSpeed(float speedMPerSec);
  void setMotorMaxForce(float forceN);

private:
  glm::vec3 localAnchorA;
  glm::vec3 localAnchorB;
  glm::vec3 localAxisA;
  glm::quat relativeOrientation;

  bool  limitsEnabled = false;
  float lowerLimit = 0.0f;
  float upperLimit = 0.0f;

  bool  motorEnabled = false;
  float motorTargetSpeed = 0.0f;
  float motorMaxForce = 0.0f;

  glm::vec3 axisWorld() const;
};
