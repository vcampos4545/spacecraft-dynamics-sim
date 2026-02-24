#include "RigidBody.h"
#include "ForceGenerator.h"
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/quaternion.hpp>
#include <cmath>

// --------------------------------------------------
// Construction
// --------------------------------------------------

RigidBody::RigidBody(RigidBodyShape shapeType, const glm::vec3 &sizeVec)
    : position(0.0f),
      velocity(0.0f),
      orientation(1, 0, 0, 0),
      angularVelocity(0.0f),
      mass(0.0f),
      invMass(0.0f),
      forceAccum(0.0f),
      torqueAccum(0.0f),
      shape(shapeType),
      size(sizeVec),
      density(0.0f)
{
}

std::unique_ptr<RigidBody> RigidBody::fromDensity(
    RigidBodyShape shape,
    const glm::vec3 &size,
    float density)
{
  auto body = std::make_unique<RigidBody>(shape, size);

  body->density = density;
  body->mass = density * body->computeVolume();
  body->computeInertiaFromMass();

  return body;
}

std::unique_ptr<RigidBody> RigidBody::fromMass(
    RigidBodyShape shape,
    const glm::vec3 &size,
    float mass)
{
  auto body = std::make_unique<RigidBody>(shape, size);

  body->mass = mass;
  body->density = mass / body->computeVolume();
  body->computeInertiaFromMass();

  return body;
}

RigidBody::~RigidBody() {}
// --------------------------------------------------
// Mass & Inertia Computation
// --------------------------------------------------

float RigidBody::computeVolume() const
{
  switch (shape)
  {
  case RigidBodyShape::BOX:
    return size.x * size.y * size.z;

  case RigidBodyShape::SPHERE:
    return (4.0f / 3.0f) * M_PI * size.x * size.x * size.x;

  case RigidBodyShape::CYLINDER:
    return M_PI * size.x * size.x * size.y;
  }
  return 0.0f;
}

void RigidBody::computeInertiaFromMass()
{
  glm::mat3 I(0.0f);

  switch (shape)
  {
  case RigidBodyShape::BOX:
  {
    float w = size.x;
    float h = size.y;
    float d = size.z;

    float Ixx = (1.0f / 12.0f) * mass * (h * h + d * d);
    float Iyy = (1.0f / 12.0f) * mass * (w * w + d * d);
    float Izz = (1.0f / 12.0f) * mass * (w * w + h * h);

    I = glm::mat3(
        Ixx, 0, 0,
        0, Iyy, 0,
        0, 0, Izz);
    break;
  }

  case RigidBodyShape::SPHERE:
  {
    float r = size.x;
    float Ival = (2.0f / 5.0f) * mass * r * r;

    I = glm::mat3(
        Ival, 0, 0,
        0, Ival, 0,
        0, 0, Ival);
    break;
  }

  case RigidBodyShape::CYLINDER:
  {
    float r = size.x;
    float l = size.y;

    float Izz = (1.0f / 2.0f) * mass * r * r;
    float Iyy = (1.0f / 4.0f) * mass * r * r + (1.0f / 12.0f) * mass * l * l;

    I = glm::mat3(
        Iyy, 0, 0,
        0, Iyy, 0,
        0, 0, Izz);
    break;
  }
  }

  invMass = (mass > 0.0f) ? 1.0f / mass : 0.0f;
  inertiaTensor = I;
  invInertiaTensor = glm::inverse(I);
}

// --------------------------------------------------
// Force & Torque
// --------------------------------------------------

void RigidBody::applyForce(const glm::vec3 &force)
{
  forceAccum += force;
}

void RigidBody::applyTorque(const glm::vec3 &torque)
{
  torqueAccum += torque;
}

void RigidBody::applyForceAtPoint(const glm::vec3 &force,
                                  const glm::vec3 &worldPoint)
{
  forceAccum += force;

  glm::vec3 r = worldPoint - position;
  torqueAccum += glm::cross(r, force);
}

void RigidBody::addForceGenerator(std::unique_ptr<ForceGenerator> generator)
{
  if (!generator)
    return;

  forceGenerators.push_back(std::move(generator));
}

void RigidBody::applyForceGenerators(float dt)
{
  for (auto &fg : forceGenerators)
    fg->apply(*this, dt);
}
// --------------------------------------------------
// Integration
// --------------------------------------------------

void RigidBody::integrate(float dt)
{
  if (invMass == 0.0f)
    return;

  // Apply all force generators (adds to accumulators alongside any
  // forces already applied externally this frame, e.g. thrusters)
  applyForceGenerators(dt);

  // Integrate motion
  integrateLinear(dt);
  integrateAngular(dt);

  // Clear for next frame (done at the end so external applyForce calls
  // made before world.step() are included in this frame's integration)
  clearAccumulators();
}

void RigidBody::integrateLinear(float dt)
{
  glm::vec3 accel = forceAccum * invMass;

  velocity += accel * dt;
  position += velocity * dt;
}

void RigidBody::integrateAngular(float dt)
{
  glm::mat3 R = glm::toMat3(orientation);
  glm::mat3 invI_world = R * invInertiaTensor * glm::transpose(R);

  glm::vec3 angularAccel = invI_world * torqueAccum;

  angularVelocity += angularAccel * dt;

  glm::quat omegaQuat(
      0.0f,
      angularVelocity.x,
      angularVelocity.y,
      angularVelocity.z);

  orientation += 0.5f * omegaQuat * orientation * dt;
  orientation = glm::normalize(orientation);
}

void RigidBody::clearAccumulators()
{
  forceAccum = glm::vec3(0.0f);
  torqueAccum = glm::vec3(0.0f);
}

// --------------------------------------------------
// Collision
// --------------------------------------------------

glm::vec3 RigidBody::getVelocityAtPoint(const glm::vec3 &worldPoint) const
{
  glm::vec3 r = worldPoint - position;
  return velocity + glm::cross(angularVelocity, r);
}

void RigidBody::getWorldCorners(glm::vec3 corners[8]) const
{
  glm::vec3 halfSize = size * 0.5f;
  glm::mat3 R = glm::toMat3(orientation);

  // Local space corners
  glm::vec3 localCorners[8] = {
      {-halfSize.x, -halfSize.y, -halfSize.z},
      {+halfSize.x, -halfSize.y, -halfSize.z},
      {-halfSize.x, +halfSize.y, -halfSize.z},
      {+halfSize.x, +halfSize.y, -halfSize.z},
      {-halfSize.x, -halfSize.y, +halfSize.z},
      {+halfSize.x, -halfSize.y, +halfSize.z},
      {-halfSize.x, +halfSize.y, +halfSize.z},
      {+halfSize.x, +halfSize.y, +halfSize.z},
  };

  for (int i = 0; i < 8; i++)
  {
    corners[i] = position + R * localCorners[i];
  }
}

void RigidBody::resolveGroundCollision(float groundZ,
                                       float restitution,
                                       float friction)
{
  switch (shape)
  {
  case RigidBodyShape::BOX:
    resolveBoxGroundCollision(groundZ, restitution, friction);
    break;
  case RigidBodyShape::SPHERE:
    resolveSphereGroundCollision(groundZ, restitution, friction);
    break;
  case RigidBodyShape::CYLINDER:
    resolveCylinderGroundCollision(groundZ, restitution, friction);
    break;
  }
}
void RigidBody::resolveBoxGroundCollision(float groundZ,
                                          float restitution,
                                          float friction)
{
  glm::vec3 corners[8];
  getWorldCorners(corners);

  glm::vec3 groundNormal(0, 0, 1); // Ground faces up

  // World-space inverse inertia tensor
  glm::mat3 R = glm::toMat3(orientation);
  glm::mat3 invI_world = R * invInertiaTensor * glm::transpose(R);

  for (int i = 0; i < 8; i++)
  {
    float penetration = groundZ - corners[i].z;

    if (penetration <= 0.0f)
      continue; // No collision at this corner

    glm::vec3 contactPoint = corners[i];
    glm::vec3 r = contactPoint - position;

    // Velocity at contact point
    glm::vec3 velAtContact = getVelocityAtPoint(contactPoint);
    float velNormal = glm::dot(velAtContact, groundNormal);

    // Only resolve if moving into the ground
    if (velNormal >= 0.0f)
    {
      // Still penetrating but moving away - just correct position
      position.z += penetration;
      continue;
    }

    // Compute impulse denominator:
    // j = -(1+e) * vn / (1/m + n · ((I^-1 * (r x n)) x r))
    glm::vec3 rCrossN = glm::cross(r, groundNormal);
    glm::vec3 inertiaTermVec = glm::cross(invI_world * rCrossN, r);
    float inertiaTerm = glm::dot(inertiaTermVec, groundNormal);

    float impulseMag = -(1.0f + restitution) * velNormal /
                       (invMass + inertiaTerm);

    // Apply normal impulse
    glm::vec3 impulse = impulseMag * groundNormal;
    velocity += impulse * invMass;
    angularVelocity += invI_world * glm::cross(r, impulse);

    // Friction impulse (tangential)
    glm::vec3 velTangent = velAtContact - velNormal * groundNormal;
    float velTangentMag = glm::length(velTangent);

    if (velTangentMag > 0.001f)
    {
      glm::vec3 tangent = velTangent / velTangentMag;

      // Friction impulse magnitude (Coulomb friction, clamped)
      float frictionImpulseMag = friction * impulseMag;

      glm::vec3 frictionImpulse = -frictionImpulseMag * tangent;
      velocity += frictionImpulse * invMass;
      angularVelocity += invI_world * glm::cross(r, frictionImpulse);
    }

    // Position correction to resolve penetration
    position.z += penetration;
  }
}

void RigidBody::resolveSphereGroundCollision(float groundZ,
                                             float restitution,
                                             float friction)
{
  float radius = size.x;
  float penetration = groundZ + radius - position.z;

  if (penetration <= 0.0f)
    return; // No collision

  glm::vec3 groundNormal(0, 0, 1);
  glm::vec3 contactPoint = position - glm::vec3(0, 0, radius);
  glm::vec3 r = contactPoint - position;

  // World-space inverse inertia tensor
  glm::mat3 R = glm::toMat3(orientation);
  glm::mat3 invI_world = R * invInertiaTensor * glm::transpose(R);

  // Velocity at contact point
  glm::vec3 velAtContact = getVelocityAtPoint(contactPoint);
  float velNormal = glm::dot(velAtContact, groundNormal);

  if (velNormal >= 0.0f)
  {
    // Moving away, just correct position
    position.z += penetration;
    return;
  }

  // Compute impulse
  glm::vec3 rCrossN = glm::cross(r, groundNormal);
  glm::vec3 inertiaTermVec = glm::cross(invI_world * rCrossN, r);
  float inertiaTerm = glm::dot(inertiaTermVec, groundNormal);

  float impulseMag = -(1.0f + restitution) * velNormal /
                     (invMass + inertiaTerm);

  // Apply normal impulse
  glm::vec3 impulse = impulseMag * groundNormal;
  velocity += impulse * invMass;
  angularVelocity += invI_world * glm::cross(r, impulse);

  // Friction impulse
  glm::vec3 velTangent = velAtContact - velNormal * groundNormal;
  float velTangentMag = glm::length(velTangent);

  if (velTangentMag > 0.001f)
  {
    glm::vec3 tangent = velTangent / velTangentMag;
    float frictionImpulseMag = friction * impulseMag;
    glm::vec3 frictionImpulse = -frictionImpulseMag * tangent;
    velocity += frictionImpulse * invMass;
    angularVelocity += invI_world * glm::cross(r, frictionImpulse);
  }

  // Position correction
  position.z += penetration;
}

void RigidBody::resolveCylinderGroundCollision(float groundZ,
                                               float restitution,
                                               float friction)
{
  float radius = size.x;
  float halfLength = size.y * 0.5f;

  glm::mat3 R = glm::toMat3(orientation);
  glm::mat3 invI_world = R * invInertiaTensor * glm::transpose(R);
  glm::vec3 groundNormal(0, 0, 1);

  // Local Z axis is the cylinder's axis
  glm::vec3 localAxis(0, 0, 1);
  glm::vec3 worldAxis = R * localAxis;

  // Sample points around both end caps
  const int numSamples = 8;

  for (int cap = 0; cap < 2; cap++)
  {
    float capSign = (cap == 0) ? -1.0f : 1.0f;
    glm::vec3 capCenter = position + worldAxis * (capSign * halfLength);

    // Get vectors perpendicular to cylinder axis for sampling the rim
    glm::vec3 perpX, perpY;
    if (std::abs(worldAxis.z) < 0.99f)
    {
      perpX = glm::normalize(glm::cross(worldAxis, glm::vec3(0, 0, 1)));
    }
    else
    {
      perpX = glm::normalize(glm::cross(worldAxis, glm::vec3(1, 0, 0)));
    }
    perpY = glm::cross(worldAxis, perpX);

    // Check cap center
    float centerPenetration = groundZ - capCenter.z;
    if (centerPenetration > 0.0f)
    {
      resolveContactPoint(capCenter, groundZ, groundNormal, invI_world, restitution, friction);
    }

    // Check points around the rim
    for (int i = 0; i < numSamples; i++)
    {
      float angle = (2.0f * M_PI * i) / numSamples;
      glm::vec3 rimPoint = capCenter + radius * (cos(angle) * perpX + sin(angle) * perpY);

      float penetration = groundZ - rimPoint.z;
      if (penetration > 0.0f)
      {
        resolveContactPoint(rimPoint, groundZ, groundNormal, invI_world, restitution, friction);
      }
    }
  }
}

void RigidBody::resolveContactPoint(const glm::vec3 &contactPoint,
                                    float groundZ,
                                    const glm::vec3 &groundNormal,
                                    const glm::mat3 &invI_world,
                                    float restitution,
                                    float friction)
{
  float penetration = groundZ - contactPoint.z;
  glm::vec3 r = contactPoint - position;

  glm::vec3 velAtContact = getVelocityAtPoint(contactPoint);
  float velNormal = glm::dot(velAtContact, groundNormal);

  if (velNormal >= 0.0f)
  {
    position.z += penetration;
    return;
  }

  glm::vec3 rCrossN = glm::cross(r, groundNormal);
  glm::vec3 inertiaTermVec = glm::cross(invI_world * rCrossN, r);
  float inertiaTerm = glm::dot(inertiaTermVec, groundNormal);

  float impulseMag = -(1.0f + restitution) * velNormal /
                     (invMass + inertiaTerm);

  glm::vec3 impulse = impulseMag * groundNormal;
  velocity += impulse * invMass;
  angularVelocity += invI_world * glm::cross(r, impulse);

  glm::vec3 velTangent = velAtContact - velNormal * groundNormal;
  float velTangentMag = glm::length(velTangent);

  if (velTangentMag > 0.001f)
  {
    glm::vec3 tangent = velTangent / velTangentMag;
    float frictionImpulseMag = friction * impulseMag;
    glm::vec3 frictionImpulse = -frictionImpulseMag * tangent;
    velocity += frictionImpulse * invMass;
    angularVelocity += invI_world * glm::cross(r, frictionImpulse);
  }

  position.z += penetration;
}