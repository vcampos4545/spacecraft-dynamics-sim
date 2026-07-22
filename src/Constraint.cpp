#include <rigidbody/Constraint.h>
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/quaternion.hpp>
#include <cmath>
#include <algorithm>

// -----------------------------------------------------------------------
// Shared helpers (used by FixedJoint, DistanceConstraint, HingeConstraint)
// -----------------------------------------------------------------------
namespace {

glm::mat3 worldInvI(const RigidBody &b)
{
  glm::mat3 R = glm::toMat3(b.orientation);
  return R * b.invInertiaTensor * glm::transpose(R);
}

// Returns skew-symmetric matrix S such that S*u == cross(v, u).
// GLM mat3 is column-major: mat3(col0, col1, col2).
glm::mat3 skew(const glm::vec3 &v)
{
  return glm::mat3(
      glm::vec3( 0.0f,  v.z, -v.y),  // col 0
      glm::vec3(-v.z,  0.0f,  v.x),  // col 1
      glm::vec3( v.y, -v.x,  0.0f)); // col 2
}

// Solve 3x3 linear system A*x = b via Cramer's rule.
// Returns zero if A is (near-)singular.
glm::vec3 solve3(const glm::mat3 &A, const glm::vec3 &b)
{
  float det = glm::determinant(A);
  if (std::abs(det) < 1e-10f) return glm::vec3(0.0f);

  glm::mat3 Ax = A; Ax[0] = b; // replace column 0
  glm::mat3 Ay = A; Ay[1] = b; // replace column 1
  glm::mat3 Az = A; Az[2] = b; // replace column 2

  return {glm::determinant(Ax) / det,
          glm::determinant(Ay) / det,
          glm::determinant(Az) / det};
}

// A vector perpendicular to `axis` (which need not be normalized).
// Picks whichever world axis is least aligned with `axis` to cross against,
// avoiding a near-zero cross product.
glm::vec3 anyPerpendicular(const glm::vec3 &axis)
{
  glm::vec3 a = glm::normalize(axis);
  glm::vec3 ref = (std::abs(a.z) < 0.99f) ? glm::vec3(0, 0, 1) : glm::vec3(1, 0, 0);
  return glm::normalize(glm::cross(a, ref));
}

} // anonymous namespace

// -----------------------------------------------------------------------
// FixedJoint
// -----------------------------------------------------------------------

FixedJoint::FixedJoint(RigidBody *a, RigidBody *b, const glm::vec3 &worldPivot)
    : bodyA(a), bodyB(b)
{
  glm::mat3 RA = glm::toMat3(a->orientation);
  glm::mat3 RB = glm::toMat3(b->orientation);

  // Store pivot in each body's local frame
  localAnchorA = glm::transpose(RA) * (worldPivot - a->position);
  localAnchorB = glm::transpose(RB) * (worldPivot - b->position);

  // Capture relative orientation: q_A^-1 * q_B
  relativeOrientation = glm::inverse(a->orientation) * b->orientation;
}

void FixedJoint::solve(float dt)
{
  if (!bodyA || !bodyB) return;

  glm::mat3 RA    = glm::toMat3(bodyA->orientation);
  glm::mat3 RB    = glm::toMat3(bodyB->orientation);
  glm::mat3 invIA = worldInvI(*bodyA);
  glm::mat3 invIB = worldInvI(*bodyB);

  // World-space vectors from body CoM to the pivot
  glm::vec3 rA = RA * localAnchorA;
  glm::vec3 rB = RB * localAnchorB;

  // ---- Translational constraint (3 DOF) --------------------------------

  // Positional error: target anchorA_world == anchorB_world
  glm::vec3 posError = (bodyB->position + rB) - (bodyA->position + rA);

  // Relative velocity at pivot
  glm::vec3 velA   = bodyA->velocity + glm::cross(bodyA->angularVelocity, rA);
  glm::vec3 velB   = bodyB->velocity + glm::cross(bodyB->angularVelocity, rB);
  glm::vec3 relVel = velA - velB;

  // 3×3 effective mass matrix:
  //   K = (mA_inv + mB_inv)*I3 - skew(rA)*invIA*skew(rA)^T
  //                             - skew(rB)*invIB*skew(rB)^T
  // skew(r)^T = -skew(r), so  -skew(r)*invI*skew(r)^T = skew(r)*invI*skew(r)
  glm::mat3 sA = skew(rA);
  glm::mat3 sB = skew(rB);
  glm::mat3 K  = (bodyA->invMass + bodyB->invMass) * glm::mat3(1.0f)
               + sA * invIA * glm::transpose(sA)
               + sB * invIB * glm::transpose(sB);

  // RHS: -(relVel + baumgarte bias)
  glm::vec3 rhs    = -(relVel - (beta / dt) * posError);
  glm::vec3 lambda = solve3(K, rhs);

  bodyA->velocity        += bodyA->invMass * lambda;
  bodyB->velocity        -= bodyB->invMass * lambda;
  bodyA->angularVelocity += invIA * glm::cross(rA, lambda);
  bodyB->angularVelocity -= invIB * glm::cross(rB, lambda);

  // ---- Rotational constraint (3 DOF) -----------------------------------

  // Orientation error: q_err = (q_A^-1 * q_B) * relativeOrientation^-1
  glm::quat q_err = glm::inverse(bodyA->orientation)
                  * bodyB->orientation
                  * glm::inverse(relativeOrientation);

  if (q_err.w < 0.0f) q_err = -q_err; // shortest-path convention

  // Angular positional error in world space
  glm::vec3 angPosError = 2.0f * RA * glm::vec3(q_err.x, q_err.y, q_err.z);

  // Relative angular velocity
  glm::vec3 relAngVel = bodyA->angularVelocity - bodyB->angularVelocity;

  // Angular effective mass: K_ang = invIA_world + invIB_world
  glm::mat3 K_ang   = invIA + invIB;
  glm::vec3 rhs_ang = -(relAngVel - (beta / dt) * angPosError);
  glm::vec3 lam_ang = solve3(K_ang, rhs_ang);

  bodyA->angularVelocity += invIA * lam_ang;
  bodyB->angularVelocity -= invIB * lam_ang;
}

bool FixedJoint::connects(const RigidBody *a, const RigidBody *b) const
{
  return (bodyA == a && bodyB == b) || (bodyA == b && bodyB == a);
}

bool FixedJoint::involves(const RigidBody *body) const
{
  return bodyA == body || bodyB == body;
}

// -----------------------------------------------------------------------
// DistanceConstraint
// -----------------------------------------------------------------------

DistanceConstraint::DistanceConstraint(RigidBody *a, RigidBody *b,
                                       const glm::vec3 &worldPivotA,
                                       const glm::vec3 &worldPivotB,
                                       float rest, bool uni)
    : bodyA(a), bodyB(b), restLength(rest), unilateral(uni)
{
  if (bodyA)
  {
    glm::mat3 RA = glm::toMat3(bodyA->orientation);
    localAnchorA = glm::transpose(RA) * (worldPivotA - bodyA->position);
  }
  else
  {
    localAnchorA = worldPivotA; // fixed world position
  }

  if (bodyB)
  {
    glm::mat3 RB = glm::toMat3(bodyB->orientation);
    localAnchorB = glm::transpose(RB) * (worldPivotB - bodyB->position);
  }
  else
  {
    localAnchorB = worldPivotB; // fixed world position
  }
}

std::pair<glm::vec3, glm::vec3> DistanceConstraint::getWorldAnchors() const
{
  glm::vec3 pA = bodyA
                     ? (bodyA->position + glm::toMat3(bodyA->orientation) * localAnchorA)
                     : localAnchorA;
  glm::vec3 pB = bodyB
                     ? (bodyB->position + glm::toMat3(bodyB->orientation) * localAnchorB)
                     : localAnchorB;
  return {pA, pB};
}

bool DistanceConstraint::connects(const RigidBody *a, const RigidBody *b) const
{
  if (!bodyA || !bodyB) return false;
  return (bodyA == a && bodyB == b) || (bodyA == b && bodyB == a);
}

bool DistanceConstraint::involves(const RigidBody *body) const
{
  return (bodyA && bodyA == body) || (bodyB && bodyB == body);
}

void DistanceConstraint::solve(float dt)
{
  auto [pA, pB] = getWorldAnchors();

  glm::vec3 delta = pA - pB;
  float dist = glm::length(delta);
  if (dist < 1e-6f) return;

  float C = dist - restLength;

  // Unilateral (string): only act when stretched
  if (unilateral && C <= 0.0f) return;

  glm::vec3 n = delta / dist; // unit vector from pB toward pA

  // Relative velocity at anchor points
  glm::vec3 velA = bodyA
                       ? (bodyA->velocity + glm::cross(bodyA->angularVelocity, pA - bodyA->position))
                       : glm::vec3(0.0f);
  glm::vec3 velB = bodyB
                       ? (bodyB->velocity + glm::cross(bodyB->angularVelocity, pB - bodyB->position))
                       : glm::vec3(0.0f);

  float Cdot = glm::dot(n, velA - velB);

  // Effective mass
  float K = 0.0f;
  if (bodyA)
  {
    glm::vec3 rA    = pA - bodyA->position;
    glm::vec3 rAxN  = glm::cross(rA, n);
    glm::mat3 invIA = worldInvI(*bodyA);
    K += bodyA->invMass + glm::dot(rAxN, invIA * rAxN);
  }
  if (bodyB)
  {
    glm::vec3 rB    = pB - bodyB->position;
    glm::vec3 rBxN  = glm::cross(rB, n);
    glm::mat3 invIB = worldInvI(*bodyB);
    K += bodyB->invMass + glm::dot(rBxN, invIB * rBxN);
  }

  if (K < 1e-10f) return;

  float lambda = -(Cdot + (beta / dt) * C) / K;

  // Unilateral: string can only pull (lambda < 0 pulls A toward B)
  if (unilateral) lambda = std::min(0.0f, lambda);

  glm::vec3 impulse = lambda * n;

  if (bodyA)
  {
    glm::vec3 rA    = pA - bodyA->position;
    glm::mat3 invIA = worldInvI(*bodyA);
    bodyA->velocity        += bodyA->invMass * impulse;
    bodyA->angularVelocity += invIA * glm::cross(rA, impulse);
  }
  if (bodyB)
  {
    glm::vec3 rB    = pB - bodyB->position;
    glm::mat3 invIB = worldInvI(*bodyB);
    bodyB->velocity        -= bodyB->invMass * impulse;
    bodyB->angularVelocity -= invIB * glm::cross(rB, impulse);
  }
}

// -----------------------------------------------------------------------
// HingeConstraint
// -----------------------------------------------------------------------

HingeConstraint::HingeConstraint(RigidBody *a, RigidBody *b,
                                 const glm::vec3 &worldPivot,
                                 const glm::vec3 &worldAxis)
    : bodyA(a), bodyB(b)
{
  glm::mat3 RA = glm::toMat3(a->orientation);
  glm::mat3 RB = glm::toMat3(b->orientation);
  glm::mat3 RAt = glm::transpose(RA);
  glm::mat3 RBt = glm::transpose(RB);

  localAnchorA = RAt * (worldPivot - a->position);
  localAnchorB = RBt * (worldPivot - b->position);

  glm::vec3 axis = glm::normalize(worldAxis);
  localAxisA = RAt * axis;
  localAxisB = RBt * axis;

  // Reference perpendicular vector, coincident in both bodies' local frames
  // at construction (so getAngle() reads exactly zero here).
  glm::vec3 perp = anyPerpendicular(axis);
  localPerpA = RAt * perp;
  localPerpB = RBt * perp;
}

bool HingeConstraint::connects(const RigidBody *a, const RigidBody *b) const
{
  return (bodyA == a && bodyB == b) || (bodyA == b && bodyB == a);
}

bool HingeConstraint::involves(const RigidBody *body) const
{
  return bodyA == body || bodyB == body;
}

glm::vec3 HingeConstraint::axisWorld() const
{
  return glm::normalize(glm::toMat3(bodyA->orientation) * localAxisA);
}

void HingeConstraint::computeAxisAndAngle(glm::vec3 &axisOut, float &angleOut) const
{
  glm::mat3 RA = glm::toMat3(bodyA->orientation);
  glm::mat3 RB = glm::toMat3(bodyB->orientation);

  glm::vec3 axis = glm::normalize(RA * localAxisA);

  // Project each body's reference perpendicular onto the plane orthogonal
  // to the (shared, once the constraint is satisfied) axis, then measure
  // the signed angle between them around that axis.
  glm::vec3 perpA = RA * localPerpA;
  perpA = glm::normalize(perpA - axis * glm::dot(perpA, axis));

  glm::vec3 perpB = RB * localPerpB;
  perpB = glm::normalize(perpB - axis * glm::dot(perpB, axis));

  float angle = std::atan2(glm::dot(glm::cross(perpA, perpB), axis), glm::dot(perpA, perpB));

  axisOut = axis;
  angleOut = angle;
}

float HingeConstraint::getAngle() const
{
  glm::vec3 axis;
  float angle;
  computeAxisAndAngle(axis, angle);
  return angle;
}

void HingeConstraint::setLimits(float lowerRad, float upperRad)
{
  limitsEnabled = true;
  lowerLimit = lowerRad;
  upperLimit = upperRad;
}

void HingeConstraint::clearLimits()
{
  limitsEnabled = false;
}

void HingeConstraint::enableMotor(bool enabled)
{
  motorEnabled = enabled;
}

void HingeConstraint::setMotorTargetSpeed(float speedRadPerSec)
{
  motorTargetSpeed = speedRadPerSec;
}

void HingeConstraint::setMotorMaxTorque(float torqueNm)
{
  motorMaxTorque = torqueNm;
}

void HingeConstraint::solve(float dt)
{
  if (!bodyA || !bodyB) return;

  glm::mat3 RA    = glm::toMat3(bodyA->orientation);
  glm::mat3 RB    = glm::toMat3(bodyB->orientation);
  glm::mat3 invIA = worldInvI(*bodyA);
  glm::mat3 invIB = worldInvI(*bodyB);

  glm::vec3 rA = RA * localAnchorA;
  glm::vec3 rB = RB * localAnchorB;

  // ---- Point constraint (3 DOF): pivots coincide ------------------------
  // Identical in form to FixedJoint's translational solve.
  {
    glm::vec3 posError = (bodyB->position + rB) - (bodyA->position + rA);

    glm::vec3 velA   = bodyA->velocity + glm::cross(bodyA->angularVelocity, rA);
    glm::vec3 velB   = bodyB->velocity + glm::cross(bodyB->angularVelocity, rB);
    glm::vec3 relVel = velA - velB;

    glm::mat3 sA = skew(rA);
    glm::mat3 sB = skew(rB);
    glm::mat3 K  = (bodyA->invMass + bodyB->invMass) * glm::mat3(1.0f)
                 + sA * invIA * glm::transpose(sA)
                 + sB * invIB * glm::transpose(sB);

    glm::vec3 rhs    = -(relVel - (beta / dt) * posError);
    glm::vec3 lambda = solve3(K, rhs);

    bodyA->velocity        += bodyA->invMass * lambda;
    bodyB->velocity        -= bodyB->invMass * lambda;
    bodyA->angularVelocity += invIA * glm::cross(rA, lambda);
    bodyB->angularVelocity -= invIB * glm::cross(rB, lambda);
  }

  // ---- Axis-alignment constraint (2 DOF): leaves rotation about the ----
  // ---- hinge axis free ---------------------------------------------------
  glm::vec3 axis = glm::normalize(RA * localAxisA);
  {
    glm::vec3 axisB = glm::normalize(RB * localAxisB);

    // Small-angle axis-misalignment vector: zero when axisB == axis, and
    // (to first order) lies entirely in the plane perpendicular to the
    // axis — so, unlike FixedJoint's full quaternion error, this never
    // has a component that resists spin about the hinge axis itself.
    glm::vec3 angPosError = glm::cross(axis, axisB);

    glm::vec3 relAngVel     = bodyA->angularVelocity - bodyB->angularVelocity;
    glm::vec3 relAngVelPerp = relAngVel - axis * glm::dot(relAngVel, axis);

    glm::mat3 K_ang   = invIA + invIB;
    glm::vec3 rhs_ang = -(relAngVelPerp - (beta / dt) * angPosError);
    glm::vec3 lam_ang = solve3(K_ang, rhs_ang);

    // Re-project the solved impulse: guarantees no torque is ever applied
    // along the hinge axis, even with a non-isotropic inertia tensor.
    lam_ang -= axis * glm::dot(lam_ang, axis);

    bodyA->angularVelocity += invIA * lam_ang;
    bodyB->angularVelocity -= invIB * lam_ang;
  }

  float Kscalar = glm::dot(axis, invIA * axis) + glm::dot(axis, invIB * axis);
  if (Kscalar < 1e-10f) return;

  // ---- Angle limit (hard stop, 1 DOF along the axis) ---------------------
  if (limitsEnabled)
  {
    float angle = getAngle();
    float relAngVelAxis = glm::dot(bodyA->angularVelocity - bodyB->angularVelocity, axis);

    float s = 0.0f;
    if (angle > upperLimit)
    {
      float C = angle - upperLimit;
      s = (-relAngVelAxis + (limitBeta / dt) * C) / Kscalar;
    }
    else if (angle < lowerLimit)
    {
      float C = lowerLimit - angle;
      s = -(relAngVelAxis + (limitBeta / dt) * C) / Kscalar;
    }

    if (s != 0.0f)
    {
      glm::vec3 impulse = s * axis;
      bodyA->angularVelocity += invIA * impulse;
      bodyB->angularVelocity -= invIB * impulse;
    }
  }

  // ---- Motor (velocity servo, 1 DOF along the axis) ----------------------
  if (motorEnabled)
  {
    float relAngVelAxis = glm::dot(bodyA->angularVelocity - bodyB->angularVelocity, axis);
    float angleRateNow  = -relAngVelAxis;

    float s = (angleRateNow - motorTargetSpeed) / Kscalar;
    float maxImpulse = motorMaxTorque * dt;
    s = glm::clamp(s, -maxImpulse, maxImpulse);

    glm::vec3 impulse = s * axis;
    bodyA->angularVelocity += invIA * impulse;
    bodyB->angularVelocity -= invIB * impulse;
  }
}
