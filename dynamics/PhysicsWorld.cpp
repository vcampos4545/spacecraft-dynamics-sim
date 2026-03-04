#include "PhysicsWorld.h"
#include <algorithm>
#include <cmath>
#include <vector>
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/quaternion.hpp>

// ======================================================
// GJK + EPA  (anonymous namespace — not part of the API)
// ======================================================
namespace {

// Minkowski-difference simplex (newest point at index 0).
struct GJKSimplex
{
  glm::vec3 pts[4];
  int n = 0;

  void push(const glm::vec3 &v)
  {
    for (int i = std::min(n, 3); i > 0; --i)
      pts[i] = pts[i - 1];
    pts[0] = v;
    if (n < 4) ++n;
  }
};

static glm::vec3 minkSup(RigidBody &a, RigidBody &b, const glm::vec3 &d)
{
  return a.support(d) - b.support(-d);
}

// ---- Simplex sub-routines ----

static bool gjkLine(GJKSimplex &s, glm::vec3 &dir)
{
  glm::vec3 A = s.pts[0], B = s.pts[1];
  glm::vec3 AB = B - A, AO = -A;
  if (glm::dot(AB, AO) > 0.0f)
  {
    glm::vec3 triple = glm::cross(glm::cross(AB, AO), AB);
    dir = (glm::length(triple) > 1e-8f) ? triple : AO;
  }
  else
  {
    s.pts[0] = A; s.n = 1;
    dir = AO;
  }
  return false;
}

static bool gjkTriangle(GJKSimplex &s, glm::vec3 &dir)
{
  glm::vec3 A = s.pts[0], B = s.pts[1], C = s.pts[2];
  glm::vec3 AB = B - A, AC = C - A, AO = -A;
  glm::vec3 ABC    = glm::cross(AB, AC);
  glm::vec3 edgeAB = glm::cross(AB, ABC); // outward from AB (away from C)
  glm::vec3 edgeAC = glm::cross(ABC, AC); // outward from AC (away from B)

  if (glm::dot(edgeAB, AO) > 0.0f)
  {
    s.pts[0] = A; s.pts[1] = B; s.n = 2;
    glm::vec3 triple = glm::cross(glm::cross(AB, AO), AB);
    dir = (glm::length(triple) > 1e-8f) ? triple : AO;
    return false;
  }
  if (glm::dot(edgeAC, AO) > 0.0f)
  {
    s.pts[0] = A; s.pts[1] = C; s.n = 2;
    glm::vec3 triple = glm::cross(glm::cross(AC, AO), AC);
    dir = (glm::length(triple) > 1e-8f) ? triple : AO;
    return false;
  }
  // Origin above or below the triangle
  if (glm::dot(ABC, AO) > 0.0f)
    dir = ABC;
  else
  {
    std::swap(s.pts[1], s.pts[2]);
    dir = -ABC;
  }
  return false;
}

static bool gjkTetrahedron(GJKSimplex &s, glm::vec3 &dir)
{
  glm::vec3 A = s.pts[0], B = s.pts[1], C = s.pts[2], D = s.pts[3];
  glm::vec3 AO = -A;
  glm::vec3 AB = B - A, AC = C - A, AD = D - A;

  glm::vec3 fABC = glm::cross(AB, AC);
  glm::vec3 fACD = glm::cross(AC, AD);
  glm::vec3 fADB = glm::cross(AD, AB);

  // Ensure each face normal points away from the opposite vertex
  if (glm::dot(fABC, AD) > 0.0f) fABC = -fABC;
  if (glm::dot(fACD, AB) > 0.0f) fACD = -fACD;
  if (glm::dot(fADB, AC) > 0.0f) fADB = -fADB;

  if (glm::dot(fABC, AO) > 0.0f) { s.pts[0]=A; s.pts[1]=B; s.pts[2]=C; s.n=3; return gjkTriangle(s, dir); }
  if (glm::dot(fACD, AO) > 0.0f) { s.pts[0]=A; s.pts[1]=C; s.pts[2]=D; s.n=3; return gjkTriangle(s, dir); }
  if (glm::dot(fADB, AO) > 0.0f) { s.pts[0]=A; s.pts[1]=D; s.pts[2]=B; s.n=3; return gjkTriangle(s, dir); }
  return true; // origin inside tetrahedron
}

static bool gjkDoSimplex(GJKSimplex &s, glm::vec3 &dir)
{
  switch (s.n)
  {
  case 2: return gjkLine(s, dir);
  case 3: return gjkTriangle(s, dir);
  case 4: return gjkTetrahedron(s, dir);
  default: return false;
  }
}

static bool gjkIntersect(RigidBody &a, RigidBody &b, GJKSimplex &simplex)
{
  glm::vec3 dir = b.position - a.position;
  if (glm::length(dir) < 1e-6f) dir = glm::vec3(1, 0, 0);

  simplex.n = 0;
  simplex.push(minkSup(a, b, dir));
  dir = -simplex.pts[0];

  for (int i = 0; i < 64; ++i)
  {
    glm::vec3 A = minkSup(a, b, dir);
    if (glm::dot(A, dir) < 0.0f) return false; // no intersection
    simplex.push(A);
    if (gjkDoSimplex(simplex, dir)) return true;
    if (glm::length(dir) < 1e-10f) return true; // degenerate
  }
  return false;
}

// ---- EPA ----

struct EPAFace { glm::vec3 normal; float dist; int a, b, c; };
struct ContactManifold { glm::vec3 normal; float depth; glm::vec3 contactPt; };

static EPAFace buildFace(const std::vector<glm::vec3> &v, int a, int b, int c,
                         const glm::vec3 &interior)
{
  glm::vec3 AB = v[b] - v[a], AC = v[c] - v[a];
  glm::vec3 n  = glm::cross(AB, AC);
  float len    = glm::length(n);
  if (len < 1e-8f) return {{0,0,1}, 0.0f, a, b, c};
  n /= len;
  if (glm::dot(n, interior - v[a]) > 0.0f) n = -n; // point outward from centroid
  float d = glm::dot(n, v[a]);
  return {n, std::max(0.0f, d), a, b, c};
}

static void addSilhouetteEdge(std::vector<std::pair<int,int>> &edges, int a, int b)
{
  for (auto it = edges.begin(); it != edges.end(); ++it)
    if (it->first == b && it->second == a) { edges.erase(it); return; }
  edges.emplace_back(a, b);
}

static ContactManifold epaGetContact(RigidBody &a, RigidBody &b, GJKSimplex &gjkSimplex)
{
  std::vector<glm::vec3> verts(gjkSimplex.pts, gjkSimplex.pts + gjkSimplex.n);

  // Ensure we start with a tetrahedron
  while ((int)verts.size() < 4)
  {
    glm::vec3 d = (verts.size() == 1)
                      ? glm::vec3(1, 0, 0)
                      : (verts.size() == 2)
                            ? glm::cross(verts[1] - verts[0], glm::vec3(0, 1, 0))
                            : glm::cross(verts[1] - verts[0], verts[2] - verts[0]);
    if (glm::length(d) < 1e-8f) d = glm::vec3(0, 0, 1);
    verts.push_back(minkSup(a, b, glm::normalize(d)));
  }

  glm::vec3 centroid = (verts[0] + verts[1] + verts[2] + verts[3]) * 0.25f;
  std::vector<EPAFace> faces = {
      buildFace(verts, 0, 1, 2, centroid),
      buildFace(verts, 0, 3, 1, centroid),
      buildFace(verts, 0, 2, 3, centroid),
      buildFace(verts, 1, 3, 2, centroid)};

  for (int iter = 0; iter < 64; ++iter)
  {
    // Find face closest to origin
    int minIdx = 0;
    for (int i = 1; i < (int)faces.size(); ++i)
      if (faces[i].dist < faces[minIdx].dist) minIdx = i;

    const EPAFace &closest = faces[minIdx];
    glm::vec3 newVert      = minkSup(a, b, closest.normal);
    float newDist          = glm::dot(closest.normal, newVert);

    if (newDist - closest.dist < 1e-4f)
      return {closest.normal, closest.dist, a.support(closest.normal)};

    verts.push_back(newVert);
    int newIdx = (int)verts.size() - 1;

    // Remove faces visible from newVert; record silhouette edges
    std::vector<std::pair<int,int>> silhouette;
    std::vector<EPAFace> remaining;
    for (auto &f : faces)
    {
      if (glm::dot(f.normal, newVert - verts[f.a]) > 0.0f)
      {
        addSilhouetteEdge(silhouette, f.a, f.b);
        addSilhouetteEdge(silhouette, f.b, f.c);
        addSilhouetteEdge(silhouette, f.c, f.a);
      }
      else
        remaining.push_back(f);
    }

    faces = remaining;
    glm::vec3 newCentroid = glm::vec3(0);
    for (auto &v : verts) newCentroid += v;
    newCentroid /= (float)verts.size();
    for (auto &e : silhouette)
      faces.push_back(buildFace(verts, e.first, e.second, newIdx, newCentroid));
  }

  // Fallback: return best face found
  int minIdx = 0;
  for (int i = 1; i < (int)faces.size(); ++i)
    if (faces[i].dist < faces[minIdx].dist) minIdx = i;
  return {faces[minIdx].normal, faces[minIdx].dist, a.support(faces[minIdx].normal)};
}

} // anonymous namespace

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
  constraints.clear();
}

FixedJoint *PhysicsWorld::addFixedJoint(RigidBody *a, RigidBody *b,
                                        const glm::vec3 &worldPivot)
{
  auto joint  = std::make_unique<FixedJoint>(a, b, worldPivot);
  FixedJoint *ptr = joint.get();
  constraints.push_back(std::move(joint));
  return ptr;
}

DistanceConstraint *PhysicsWorld::addDistanceConstraint(RigidBody *a, RigidBody *b,
                                                        const glm::vec3 &worldPivotA,
                                                        const glm::vec3 &worldPivotB,
                                                        float restLength,
                                                        bool unilateral)
{
  auto dc  = std::make_unique<DistanceConstraint>(a, b, worldPivotA, worldPivotB,
                                                  restLength, unilateral);
  DistanceConstraint *ptr = dc.get();
  constraints.push_back(std::move(dc));
  return ptr;
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
  solveConstraints(dt);
  detectAndResolveCollisions();
}

void PhysicsWorld::solveConstraints(float dt)
{
  const int iterations = 10;
  for (int i = 0; i < iterations; ++i)
    for (auto &c : constraints)
      c->solve(dt);
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
      // Skip pairs connected by a constraint (they must not fight each other)
      bool skip = false;
      for (auto &c : constraints)
        if (c->connects(bodies[i].get(), bodies[j].get())) { skip = true; break; }
      if (skip) continue;

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
  if (a.invMass == 0.0f && b.invMass == 0.0f) return;

  GJKSimplex simplex;
  if (!gjkIntersect(a, b, simplex)) return;

  ContactManifold contact = epaGetContact(a, b, simplex);

  float totalInvMass = a.invMass + b.invMass;
  if (totalInvMass <= 0.0f) return;

  // Position correction
  glm::vec3 correction = (contact.depth / totalInvMass) * contact.normal;
  a.position -= a.invMass * correction;
  b.position += b.invMass * correction;

  // Per-body rotation matrices and world-space inverse inertias
  glm::mat3 RA       = glm::toMat3(a.orientation);
  glm::mat3 RB       = glm::toMat3(b.orientation);
  glm::mat3 invIA    = RA * a.invInertiaTensor * glm::transpose(RA);
  glm::mat3 invIB    = RB * b.invInertiaTensor * glm::transpose(RB);

  glm::vec3 rA       = contact.contactPt - a.position;
  glm::vec3 rB       = contact.contactPt - b.position;

  glm::vec3 velA     = a.velocity + glm::cross(a.angularVelocity, rA);
  glm::vec3 velB     = b.velocity + glm::cross(b.angularVelocity, rB);
  glm::vec3 relVel   = velA - velB;

  float vn = glm::dot(relVel, contact.normal);
  if (vn > 0.0f) return; // already separating

  glm::vec3 rAxN = glm::cross(rA, contact.normal);
  glm::vec3 rBxN = glm::cross(rB, contact.normal);
  float denom    = totalInvMass
                   + glm::dot(contact.normal, glm::cross(invIA * rAxN, rA))
                   + glm::dot(contact.normal, glm::cross(invIB * rBxN, rB));

  const float restitution = 0.3f;
  float j        = -(1.0f + restitution) * vn / denom;
  glm::vec3 imp  = j * contact.normal;

  a.velocity         += a.invMass * imp;
  b.velocity         -= b.invMass * imp;
  a.angularVelocity  += invIA * glm::cross(rA,  imp);
  b.angularVelocity  -= invIB * glm::cross(rB,  imp);
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
