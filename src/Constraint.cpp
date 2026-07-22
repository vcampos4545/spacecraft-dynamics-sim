#include <rigidbody/Constraint.h>
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/quaternion.hpp>
#include <cmath>
#include <algorithm>

// -----------------------------------------------------------------------
// Shared math + constraint building blocks.
//
// Each of FixedConstraint / PointConstraint / HingeConstraint /
// SliderConstraint is assembled from these. They operate directly on a
// pair of RigidBody velocities (one Sequential-Impulse iteration each),
// so composing several of them in a single solve() composes the DOF locks
// they each remove.
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
//
// The singularity check is scale-relative, not absolute: these A matrices
// are sums of invMass/invInertia terms, whose *absolute* magnitude depends
// entirely on the masses involved. A rocket's inverse inertia is ~1e-6 in
// SI units, not because anything is degenerate but because it's a 500-tonne
// object — an absolute epsilon like 1e-10 would (and did) misfire on a
// perfectly well-conditioned matrix at that scale, silently zeroing out an
// entire constraint. Comparing det against the cube of A's own largest
// diagonal entry keeps the check meaningful across mass scales.
glm::vec3 solve3(const glm::mat3 &A, const glm::vec3 &b)
{
  float det = glm::determinant(A);

  float scale = std::max({std::abs(A[0][0]), std::abs(A[1][1]), std::abs(A[2][2])});
  if (scale < 1e-30f) return glm::vec3(0.0f); // A is (numerically) the zero matrix
  if (std::abs(det) < 1e-9f * scale * scale * scale) return glm::vec3(0.0f);

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

// Baumgarte position-error caps (matches Box2D's b2_maxLinearCorrection
// convention). A Baumgarte bias scales the *raw* position error by
// beta/dt (tens, with the defaults) to compute a corrective velocity —
// fine for the small errors these constraints normally run at, but with
// no cap, a large transient error (e.g. under a big torque, or between two
// bodies whose masses/inertias differ by orders of magnitude, as a rocket
// and its nose cone do) turns into a corrective velocity large enough to
// overshoot, producing a *bigger* error next step: an unstable feedback
// loop, not a convergence. Clamping the error before scaling means a big
// error still gets corrected — just gradually, over multiple steps,
// instead of in one (impossible) instant.
constexpr float MAX_LINEAR_CORRECTION  = 0.2f; // meters
constexpr float MAX_ANGULAR_CORRECTION = 0.2f; // radians (~11 degrees)

glm::vec3 clampLength(const glm::vec3 &v, float maxLen)
{
  float len = glm::length(v);
  return (len > maxLen && len > 1e-9f) ? v * (maxLen / len) : v;
}

// ---- 3-DOF blocks -------------------------------------------------------

// Locks two anchor points to coincide (3 DOF). rA/rB are the world-space
// offsets from each body's center of mass to its anchor point. Used by
// FixedConstraint, PointConstraint, HingeConstraint.
void solvePointConstraint(RigidBody &a, RigidBody &b,
                          const glm::vec3 &rA, const glm::vec3 &rB,
                          const glm::mat3 &invIA, const glm::mat3 &invIB,
                          float beta, float dt)
{
  glm::vec3 posError = (b.position + rB) - (a.position + rA);

  glm::vec3 velA   = a.velocity + glm::cross(a.angularVelocity, rA);
  glm::vec3 velB   = b.velocity + glm::cross(b.angularVelocity, rB);
  glm::vec3 relVel = velA - velB;

  // K = (mA_inv + mB_inv)*I3 - skew(rA)*invIA*skew(rA)^T
  //                           - skew(rB)*invIB*skew(rB)^T
  // skew(r)^T = -skew(r), so  -skew(r)*invI*skew(r)^T = skew(r)*invI*skew(r)
  glm::mat3 sA = skew(rA);
  glm::mat3 sB = skew(rB);
  glm::mat3 K  = (a.invMass + b.invMass) * glm::mat3(1.0f)
               + sA * invIA * glm::transpose(sA)
               + sB * invIB * glm::transpose(sB);

  glm::vec3 rhs    = -(relVel - (beta / dt) * clampLength(posError, MAX_LINEAR_CORRECTION));
  glm::vec3 lambda = solve3(K, rhs);

  a.velocity        += a.invMass * lambda;
  b.velocity        -= b.invMass * lambda;
  a.angularVelocity += invIA * glm::cross(rA, lambda);
  b.angularVelocity -= invIB * glm::cross(rB, lambda);
}

// Locks relative orientation to `relativeOrientation` (q_A^-1*q_B target at
// construction), fully removing all 3 rotational DOF. Used by
// FixedConstraint, SliderConstraint.
void solveOrientationConstraint(RigidBody &a, RigidBody &b,
                                const glm::quat &relativeOrientation,
                                const glm::mat3 &invIA, const glm::mat3 &invIB,
                                float beta, float dt)
{
  glm::mat3 RA = glm::toMat3(a.orientation);

  glm::quat q_err = glm::inverse(a.orientation) * b.orientation * glm::inverse(relativeOrientation);
  if (q_err.w < 0.0f) q_err = -q_err; // shortest-path convention

  glm::vec3 angPosError = 2.0f * RA * glm::vec3(q_err.x, q_err.y, q_err.z);
  glm::vec3 relAngVel   = a.angularVelocity - b.angularVelocity;

  glm::mat3 K_ang   = invIA + invIB;
  glm::vec3 rhs_ang = -(relAngVel - (beta / dt) * clampLength(angPosError, MAX_ANGULAR_CORRECTION));
  glm::vec3 lam_ang = solve3(K_ang, rhs_ang);

  a.angularVelocity += invIA * lam_ang;
  b.angularVelocity -= invIB * lam_ang;
}

// ---- 2-DOF blocks (a 3-DOF block above, relaxed along one axis) --------

// Locks 2 rotational DOF by aligning `axis` (from bodyA) with `axisB`
// (from bodyB), leaving rotation about the shared axis free. To first
// order this axis-misalignment vector already lies in the plane
// perpendicular to the axis, and the solved impulse is re-projected onto
// that plane too, so this never resists spin about the axis itself even
// with a non-isotropic inertia tensor. Used by HingeConstraint.
void solveAxisAlignmentConstraint(RigidBody &a, RigidBody &b,
                                  const glm::vec3 &axis, const glm::vec3 &axisB,
                                  const glm::mat3 &invIA, const glm::mat3 &invIB,
                                  float beta, float dt)
{
  glm::vec3 angPosError = glm::cross(axis, axisB);

  glm::vec3 relAngVel     = a.angularVelocity - b.angularVelocity;
  glm::vec3 relAngVelPerp = relAngVel - axis * glm::dot(relAngVel, axis);

  glm::mat3 K_ang   = invIA + invIB;
  glm::vec3 rhs_ang = -(relAngVelPerp - (beta / dt) * clampLength(angPosError, MAX_ANGULAR_CORRECTION));
  glm::vec3 lam_ang = solve3(K_ang, rhs_ang);

  lam_ang -= axis * glm::dot(lam_ang, axis);

  a.angularVelocity += invIA * lam_ang;
  b.angularVelocity -= invIB * lam_ang;
}

// Locks 2 translational DOF perpendicular to `axis`, leaving translation
// along the axis free (same relax-along-one-axis technique as the axis
// alignment block above, applied to the point constraint instead). Used
// by SliderConstraint.
void solvePerpendicularConstraint(RigidBody &a, RigidBody &b,
                                  const glm::vec3 &rA, const glm::vec3 &rB,
                                  const glm::vec3 &axis,
                                  const glm::mat3 &invIA, const glm::mat3 &invIB,
                                  float beta, float dt)
{
  glm::vec3 posErrorFull = (b.position + rB) - (a.position + rA);
  glm::vec3 posError     = posErrorFull - axis * glm::dot(posErrorFull, axis);

  glm::vec3 velA        = a.velocity + glm::cross(a.angularVelocity, rA);
  glm::vec3 velB        = b.velocity + glm::cross(b.angularVelocity, rB);
  glm::vec3 relVelFull  = velA - velB;
  glm::vec3 relVel      = relVelFull - axis * glm::dot(relVelFull, axis);

  glm::mat3 sA = skew(rA);
  glm::mat3 sB = skew(rB);
  glm::mat3 K  = (a.invMass + b.invMass) * glm::mat3(1.0f)
               + sA * invIA * glm::transpose(sA)
               + sB * invIB * glm::transpose(sB);

  glm::vec3 rhs    = -(relVel - (beta / dt) * clampLength(posError, MAX_LINEAR_CORRECTION));
  glm::vec3 lambda = solve3(K, rhs);
  lambda -= axis * glm::dot(lambda, axis); // keep along-axis translation free

  a.velocity        += a.invMass * lambda;
  b.velocity        -= b.invMass * lambda;
  a.angularVelocity += invIA * glm::cross(rA, lambda);
  b.angularVelocity -= invIB * glm::cross(rB, lambda);
}

// ---- 1-DOF limit/motor blocks -------------------------------------------
//
// Both pairs below share the same hard-stop / velocity-servo formulas —
// the angular pair applies a pure torque impulse along `axis`; the linear
// pair applies a force impulse at the anchor offsets (which also couples
// into angular velocity via the lever arm, like solvePointConstraint's
// lambda does). Used by HingeConstraint (angular) and SliderConstraint
// (linear).

void solveAngularLimit(RigidBody &a, RigidBody &b, const glm::vec3 &axis,
                       const glm::mat3 &invIA, const glm::mat3 &invIB,
                       float value, float lower, float upper,
                       float beta, float dt)
{
  float Kscalar = glm::dot(axis, invIA * axis) + glm::dot(axis, invIB * axis);
  if (Kscalar < 1e-10f) return;

  float relAxisVel = glm::dot(a.angularVelocity - b.angularVelocity, axis);

  float s;
  if (value > upper)
  {
    float C = std::min(value - upper, MAX_ANGULAR_CORRECTION);
    s = (-relAxisVel + (beta / dt) * C) / Kscalar;
  }
  else if (value < lower)
  {
    float C = std::min(lower - value, MAX_ANGULAR_CORRECTION);
    s = -(relAxisVel + (beta / dt) * C) / Kscalar;
  }
  else
  {
    return;
  }

  glm::vec3 impulse = s * axis;
  a.angularVelocity += invIA * impulse;
  b.angularVelocity -= invIB * impulse;
}

void solveAngularMotor(RigidBody &a, RigidBody &b, const glm::vec3 &axis,
                       const glm::mat3 &invIA, const glm::mat3 &invIB,
                       float targetRate, float maxTorque, float dt)
{
  float Kscalar = glm::dot(axis, invIA * axis) + glm::dot(axis, invIB * axis);
  if (Kscalar < 1e-10f) return;

  float relAxisVel = glm::dot(a.angularVelocity - b.angularVelocity, axis);
  float rateNow     = -relAxisVel;

  float s = (rateNow - targetRate) / Kscalar;
  float maxImpulse = maxTorque * dt;
  s = glm::clamp(s, -maxImpulse, maxImpulse);

  glm::vec3 impulse = s * axis;
  a.angularVelocity += invIA * impulse;
  b.angularVelocity -= invIB * impulse;
}

void solveLinearLimit(RigidBody &a, RigidBody &b,
                      const glm::vec3 &rA, const glm::vec3 &rB, const glm::vec3 &axis,
                      const glm::mat3 &invIA, const glm::mat3 &invIB,
                      float value, float lower, float upper,
                      float beta, float dt)
{
  glm::vec3 rAxN = glm::cross(rA, axis);
  glm::vec3 rBxN = glm::cross(rB, axis);
  float Kscalar  = a.invMass + b.invMass
                 + glm::dot(rAxN, invIA * rAxN)
                 + glm::dot(rBxN, invIB * rBxN);
  if (Kscalar < 1e-10f) return;

  glm::vec3 velA = a.velocity + glm::cross(a.angularVelocity, rA);
  glm::vec3 velB = b.velocity + glm::cross(b.angularVelocity, rB);
  float relAxisVel = glm::dot(velA - velB, axis);

  float s;
  if (value > upper)
  {
    float C = std::min(value - upper, MAX_LINEAR_CORRECTION);
    s = (-relAxisVel + (beta / dt) * C) / Kscalar;
  }
  else if (value < lower)
  {
    float C = std::min(lower - value, MAX_LINEAR_CORRECTION);
    s = -(relAxisVel + (beta / dt) * C) / Kscalar;
  }
  else
  {
    return;
  }

  glm::vec3 impulse = s * axis;
  a.velocity        += a.invMass * impulse;
  b.velocity        -= b.invMass * impulse;
  a.angularVelocity += invIA * glm::cross(rA, impulse);
  b.angularVelocity -= invIB * glm::cross(rB, impulse);
}

void solveLinearMotor(RigidBody &a, RigidBody &b,
                      const glm::vec3 &rA, const glm::vec3 &rB, const glm::vec3 &axis,
                      const glm::mat3 &invIA, const glm::mat3 &invIB,
                      float targetRate, float maxForce, float dt)
{
  glm::vec3 rAxN = glm::cross(rA, axis);
  glm::vec3 rBxN = glm::cross(rB, axis);
  float Kscalar  = a.invMass + b.invMass
                 + glm::dot(rAxN, invIA * rAxN)
                 + glm::dot(rBxN, invIB * rBxN);
  if (Kscalar < 1e-10f) return;

  glm::vec3 velA = a.velocity + glm::cross(a.angularVelocity, rA);
  glm::vec3 velB = b.velocity + glm::cross(b.angularVelocity, rB);
  float relAxisVel = glm::dot(velA - velB, axis);
  float rateNow     = -relAxisVel;

  float s = (rateNow - targetRate) / Kscalar;
  float maxImpulse = maxForce * dt;
  s = glm::clamp(s, -maxImpulse, maxImpulse);

  glm::vec3 impulse = s * axis;
  a.velocity        += a.invMass * impulse;
  b.velocity        -= b.invMass * impulse;
  a.angularVelocity += invIA * glm::cross(rA, impulse);
  b.angularVelocity -= invIB * glm::cross(rB, impulse);
}

} // anonymous namespace

// -----------------------------------------------------------------------
// FixedConstraint
// -----------------------------------------------------------------------

FixedConstraint::FixedConstraint(RigidBody *a, RigidBody *b, const glm::vec3 &worldPivot)
    : bodyA(a), bodyB(b)
{
  glm::mat3 RA = glm::toMat3(a->orientation);
  glm::mat3 RB = glm::toMat3(b->orientation);

  localAnchorA = glm::transpose(RA) * (worldPivot - a->position);
  localAnchorB = glm::transpose(RB) * (worldPivot - b->position);

  relativeOrientation = glm::inverse(a->orientation) * b->orientation;
}

void FixedConstraint::solve(float dt)
{
  if (!bodyA || !bodyB) return;

  glm::mat3 RA    = glm::toMat3(bodyA->orientation);
  glm::mat3 RB    = glm::toMat3(bodyB->orientation);
  glm::mat3 invIA = worldInvI(*bodyA);
  glm::mat3 invIB = worldInvI(*bodyB);

  glm::vec3 rA = RA * localAnchorA;
  glm::vec3 rB = RB * localAnchorB;

  solvePointConstraint(*bodyA, *bodyB, rA, rB, invIA, invIB, beta, dt);
  solveOrientationConstraint(*bodyA, *bodyB, relativeOrientation, invIA, invIB, beta, dt);
}

bool FixedConstraint::connects(const RigidBody *a, const RigidBody *b) const
{
  return (bodyA == a && bodyB == b) || (bodyA == b && bodyB == a);
}

bool FixedConstraint::involves(const RigidBody *body) const
{
  return bodyA == body || bodyB == body;
}

// -----------------------------------------------------------------------
// PointConstraint
// -----------------------------------------------------------------------

PointConstraint::PointConstraint(RigidBody *a, RigidBody *b, const glm::vec3 &worldPivot)
    : bodyA(a), bodyB(b)
{
  glm::mat3 RA = glm::toMat3(a->orientation);
  glm::mat3 RB = glm::toMat3(b->orientation);

  localAnchorA = glm::transpose(RA) * (worldPivot - a->position);
  localAnchorB = glm::transpose(RB) * (worldPivot - b->position);
}

void PointConstraint::solve(float dt)
{
  if (!bodyA || !bodyB) return;

  glm::mat3 RA    = glm::toMat3(bodyA->orientation);
  glm::mat3 RB    = glm::toMat3(bodyB->orientation);
  glm::mat3 invIA = worldInvI(*bodyA);
  glm::mat3 invIB = worldInvI(*bodyB);

  glm::vec3 rA = RA * localAnchorA;
  glm::vec3 rB = RB * localAnchorB;

  solvePointConstraint(*bodyA, *bodyB, rA, rB, invIA, invIB, beta, dt);
}

bool PointConstraint::connects(const RigidBody *a, const RigidBody *b) const
{
  return (bodyA == a && bodyB == b) || (bodyA == b && bodyB == a);
}

bool PointConstraint::involves(const RigidBody *body) const
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

  solvePointConstraint(*bodyA, *bodyB, rA, rB, invIA, invIB, beta, dt);

  glm::vec3 axis  = glm::normalize(RA * localAxisA);
  glm::vec3 axisB = glm::normalize(RB * localAxisB);
  solveAxisAlignmentConstraint(*bodyA, *bodyB, axis, axisB, invIA, invIB, beta, dt);

  if (limitsEnabled)
    solveAngularLimit(*bodyA, *bodyB, axis, invIA, invIB, getAngle(), lowerLimit, upperLimit, limitBeta, dt);

  if (motorEnabled)
    solveAngularMotor(*bodyA, *bodyB, axis, invIA, invIB, motorTargetSpeed, motorMaxTorque, dt);
}

// -----------------------------------------------------------------------
// SliderConstraint
// -----------------------------------------------------------------------

SliderConstraint::SliderConstraint(RigidBody *a, RigidBody *b,
                                   const glm::vec3 &worldPivot,
                                   const glm::vec3 &worldAxis)
    : bodyA(a), bodyB(b)
{
  glm::mat3 RA  = glm::toMat3(a->orientation);
  glm::mat3 RB  = glm::toMat3(b->orientation);
  glm::mat3 RAt = glm::transpose(RA);
  glm::mat3 RBt = glm::transpose(RB);

  localAnchorA = RAt * (worldPivot - a->position);
  localAnchorB = RBt * (worldPivot - b->position);
  localAxisA   = RAt * glm::normalize(worldAxis);

  relativeOrientation = glm::inverse(a->orientation) * b->orientation;
}

bool SliderConstraint::connects(const RigidBody *a, const RigidBody *b) const
{
  return (bodyA == a && bodyB == b) || (bodyA == b && bodyB == a);
}

bool SliderConstraint::involves(const RigidBody *body) const
{
  return bodyA == body || bodyB == body;
}

glm::vec3 SliderConstraint::axisWorld() const
{
  return glm::normalize(glm::toMat3(bodyA->orientation) * localAxisA);
}

float SliderConstraint::getPosition() const
{
  glm::mat3 RA = glm::toMat3(bodyA->orientation);
  glm::mat3 RB = glm::toMat3(bodyB->orientation);

  glm::vec3 rA = RA * localAnchorA;
  glm::vec3 rB = RB * localAnchorB;
  glm::vec3 axis = glm::normalize(RA * localAxisA);

  glm::vec3 anchorA = bodyA->position + rA;
  glm::vec3 anchorB = bodyB->position + rB;
  return glm::dot(anchorB - anchorA, axis);
}

void SliderConstraint::setLimits(float lowerM, float upperM)
{
  limitsEnabled = true;
  lowerLimit = lowerM;
  upperLimit = upperM;
}

void SliderConstraint::clearLimits()
{
  limitsEnabled = false;
}

void SliderConstraint::enableMotor(bool enabled)
{
  motorEnabled = enabled;
}

void SliderConstraint::setMotorTargetSpeed(float speedMPerSec)
{
  motorTargetSpeed = speedMPerSec;
}

void SliderConstraint::setMotorMaxForce(float forceN)
{
  motorMaxForce = forceN;
}

void SliderConstraint::solve(float dt)
{
  if (!bodyA || !bodyB) return;

  glm::mat3 RA    = glm::toMat3(bodyA->orientation);
  glm::mat3 RB    = glm::toMat3(bodyB->orientation);
  glm::mat3 invIA = worldInvI(*bodyA);
  glm::mat3 invIB = worldInvI(*bodyB);

  glm::vec3 rA   = RA * localAnchorA;
  glm::vec3 rB   = RB * localAnchorB;
  glm::vec3 axis = glm::normalize(RA * localAxisA);

  solvePerpendicularConstraint(*bodyA, *bodyB, rA, rB, axis, invIA, invIB, beta, dt);
  solveOrientationConstraint(*bodyA, *bodyB, relativeOrientation, invIA, invIB, beta, dt);

  if (limitsEnabled)
    solveLinearLimit(*bodyA, *bodyB, rA, rB, axis, invIA, invIB, getPosition(), lowerLimit, upperLimit, limitBeta, dt);

  if (motorEnabled)
    solveLinearMotor(*bodyA, *bodyB, rA, rB, axis, invIA, invIB, motorTargetSpeed, motorMaxForce, dt);
}
