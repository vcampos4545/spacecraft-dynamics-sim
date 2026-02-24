#include "PhysicsWorld.h"
#include <algorithm>
#include <cmath>

PhysicsWorld::~PhysicsWorld() = default;
// --------------------------------------------------
// Body Management
// --------------------------------------------------

RigidBody *PhysicsWorld::createBody(RigidBodyShape shape,
                                    const glm::vec3 &size,
                                    float mass)
{
  bodies.push_back(RigidBody::fromMass(shape, size, mass));
  return bodies.back().get();
}

void PhysicsWorld::removeBody(RigidBody *body)
{
  bodies.erase(
      std::remove_if(bodies.begin(), bodies.end(),
                     [body](const std::unique_ptr<RigidBody> &b)
                     {
                       return b.get() == body;
                     }),
      bodies.end());
}

void PhysicsWorld::clear()
{
  bodies.clear();
}

// --------------------------------------------------
// Simulation
// --------------------------------------------------

void PhysicsWorld::step(float dt)
{
  // Fixed timestep accumulator pattern
  accumulator += dt;

  // Clamp to prevent spiral of death
  if (accumulator > 0.2f)
    accumulator = 0.2f;

  while (accumulator >= fixedTimestep)
  {
    stepFixed(fixedTimestep);
    accumulator -= fixedTimestep;
  }
}

void PhysicsWorld::stepFixed(float dt)
{
  integrateAll(dt);
  detectAndResolveCollisions();
}

void PhysicsWorld::integrateAll(float dt)
{
  for (auto &body : bodies)
  {
    // Apply world gravity as a force before integration
    if (body->invMass > 0.0f)
      body->applyForce(gravity * body->mass);

    body->integrate(dt);
  }
}

// --------------------------------------------------
// Collision Detection & Resolution
// --------------------------------------------------

void PhysicsWorld::detectAndResolveCollisions()
{
  // Ground collisions
  resolveGroundCollisions();

  // Body-body collisions (O(n^2) - replace with broad phase later)
  for (size_t i = 0; i < bodies.size(); i++)
  {
    for (size_t j = i + 1; j < bodies.size(); j++)
    {
      resolveBodyBodyCollision(*bodies[i], *bodies[j]);
    }
  }
}

void PhysicsWorld::resolveGroundCollisions()
{
  const float groundZ = 0.0f;
  const float restitution = 0.3f;
  const float friction = 0.5f;

  for (auto &body : bodies)
  {
    body->resolveGroundCollision(groundZ, restitution, friction);
  }
}

void PhysicsWorld::resolveBodyBodyCollision(RigidBody &a, RigidBody &b)
{
  // Dispatch based on shape types
  if (a.shape == RigidBodyShape::SPHERE && b.shape == RigidBodyShape::SPHERE)
  {
    resolveSphereSphereCollision(a, b);
  }
  // Add more shape combinations as needed
  // else if (a.shape == RigidBodyShape::SPHERE && b.shape == RigidBodyShape::BOX)
  // {
  //   resolveSphereBoxCollision(a, b);
  // }
}

void PhysicsWorld::resolveSphereSphereCollision(RigidBody &a, RigidBody &b)
{
  float radiusA = a.size.x;
  float radiusB = b.size.x;
  float combinedRadius = radiusA + radiusB;

  glm::vec3 delta = b.position - a.position;
  float distSq = glm::dot(delta, delta);

  if (distSq >= combinedRadius * combinedRadius)
    return; // No collision

  float dist = std::sqrt(distSq);
  if (dist < 0.0001f)
  {
    // Spheres at same position, push apart arbitrarily
    delta = glm::vec3(1, 0, 0);
    dist = 0.0001f;
  }

  glm::vec3 normal = delta / dist;
  float penetration = combinedRadius - dist;

  // Relative velocity at contact
  glm::vec3 relVel = b.velocity - a.velocity;
  float velAlongNormal = glm::dot(relVel, normal);

  // Don't resolve if separating
  if (velAlongNormal > 0)
  {
    // Still need position correction
    float totalInvMass = a.invMass + b.invMass;
    if (totalInvMass > 0)
    {
      glm::vec3 correction = (penetration / totalInvMass) * normal;
      a.position -= a.invMass * correction;
      b.position += b.invMass * correction;
    }
    return;
  }

  // Restitution (bounciness)
  float restitution = 0.3f;

  // Impulse magnitude
  float totalInvMass = a.invMass + b.invMass;
  if (totalInvMass == 0)
    return; // Both static

  float impulseMag = -(1.0f + restitution) * velAlongNormal / totalInvMass;

  // Apply impulse
  glm::vec3 impulse = impulseMag * normal;
  a.velocity -= a.invMass * impulse;
  b.velocity += b.invMass * impulse;

  // Position correction to resolve penetration
  glm::vec3 correction = (penetration / totalInvMass) * normal;
  a.position -= a.invMass * correction;
  b.position += b.invMass * correction;
}

void PhysicsWorld::resolveSphereBoxCollision(RigidBody &sphere, RigidBody &box)
{
  // TODO: Implement sphere-box collision
}

void PhysicsWorld::resolveBoxBoxCollision(RigidBody &a, RigidBody &b)
{
  // TODO: Implement box-box collision (SAT algorithm)
}
