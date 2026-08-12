#pragma once
#include <rigidbody/RigidBody.h>
#include <rigidbody/Constraint.h>
#include <rigidbody/ForceGenerator.h>
#include <rigidbody/orbit/CelestialSystem.h>
#include <rigidbody/orbit/CelestialPerturbation.h>
#include <vector>
#include <memory>

class PhysicsWorld
{
public:
  float fixedTimestep = 1.0f / 120.0f; // 120 Hz physics

  PhysicsWorld() = default;
  ~PhysicsWorld();

  // Body management
  RigidBody *createBody(RigidBodyShape shape,
                        const glm::vec3 &size,
                        float mass);

  // Removing a body also removes any constraints that reference it (a
  // constraint left pointing at a freed body would dangle).
  void removeBody(RigidBody *body);
  void clear();

  // Constraint management — see Constraint.h for what each removes/allows.
  FixedConstraint  *addFixedConstraint(RigidBody *a, RigidBody *b, const glm::vec3 &worldPivot);
  PointConstraint  *addPointConstraint(RigidBody *a, RigidBody *b, const glm::vec3 &worldPivot);
  HingeConstraint  *addHingeConstraint(RigidBody *a, RigidBody *b,
                                       const glm::vec3 &worldPivot,
                                       const glm::vec3 &worldAxis);
  SliderConstraint *addSliderConstraint(RigidBody *a, RigidBody *b,
                                        const glm::vec3 &worldPivot,
                                        const glm::vec3 &worldAxis);
  DistanceConstraint *addDistanceConstraint(RigidBody *a, RigidBody *b,
                                            const glm::vec3 &worldPivotA,
                                            const glm::vec3 &worldPivotB,
                                            float restLength,
                                            bool unilateral = false);

  void removeConstraint(Constraint *constraint);

  // Global force generators (see rigidbody/environment/) apply to every
  // body in the world each substep, e.g. UniformGravity/CentralBodyGravity
  // or a wind field. The world starts with none -- no gravity, no drag,
  // nothing -- so a scenario opts into whatever physical effects it
  // actually wants, the same way a body opts into whatever
  // actuators/sensors it's given. Ownership transfers to the world; keep
  // the raw pointer beforehand if you need to read the generator back
  // later (e.g. reading UniformGravity::acceleration for a sensor model),
  // same pattern as RigidBody::addForceGenerator.
  void addGlobalForceGenerator(std::unique_ptr<ForceGenerator> generator);

  // Simulation
  void step(float dt);

  // Accessors
  const std::vector<std::unique_ptr<RigidBody>> &getBodies() const { return bodies; }
  size_t getBodyCount() const { return bodies.size(); }

  // ---------------------------------------------------------------------
  // Orbital mode: opt-in, per body. A world with neither of these called
  // behaves exactly as today (pure float32 flat-world physics via the
  // ForceGenerators above). Attaching a CelestialSystem and putting a body
  // in orbital mode instead drives that body's *translational* state from
  // a real double-precision, hierarchical orbit propagation each step() --
  // rotational dynamics (orientation, angular velocity, actuator
  // ForceGenerators) still integrate exactly as they do for any other
  // body. See rigidbody/orbit/CelestialSystem.h.
  // ---------------------------------------------------------------------

  // `system` must outlive this world. `jd0` is the Julian Date at the
  // moment this is called; step() advances it internally by real elapsed
  // dt each call.
  void attachCelestialSystem(CelestialSystem *system, double jd0);

  // Puts `body` in orbital mode: its own OrbitState/OrbitPropagator
  // (owned internally) governs `body->position` from here on, propagated
  // relative to `primary` (TwoBodyGravity toward primary's mu, plus a
  // CelestialPerturbation for each entry in `perturbers`). Requires
  // attachCelestialSystem() to have been called first.
  void setOrbitalMode(RigidBody *body, const CelestialBody *primary,
                       const std::vector<const CelestialBody *> &perturbers,
                       const OrbitState &initialState);

  // Read-only queries for an orbital-mode body -- the point of this
  // feature: a consumer calls these instead of deriving eclipse/field
  // state itself. Undefined (asserts) if `body` isn't in orbital mode.
  bool isInEclipse(const RigidBody *body) const;
  glm::vec3 ambientFieldAt(const RigidBody *body) const;
  glm::vec3 sunDirectionAt(const RigidBody *body) const;

private:
  std::vector<std::unique_ptr<RigidBody>> bodies;
  std::vector<std::unique_ptr<Constraint>> constraints;
  std::vector<std::unique_ptr<ForceGenerator>> globalForceGenerators;
  float accumulator = 0.0f;

  void stepFixed(float dt);
  void integrateAll(float dt);
  void solveConstraints(float dt);
  void detectAndResolveCollisions();

  // Body-to-body collision
  void resolveBodyBodyCollision(RigidBody &a, RigidBody &b);
  void resolveSphereSphereCollision(RigidBody &a, RigidBody &b);
  void resolveSphereBoxCollision(RigidBody &sphere, RigidBody &box);
  void resolveBoxBoxCollision(RigidBody &a, RigidBody &b);

  // Ground collision (z = 0 plane)
  void resolveGroundCollisions();

  // Orbital mode
  struct OrbitalModeEntry
  {
    RigidBody *body = nullptr;
    const CelestialBody *primary = nullptr;
    OrbitState state;
    OrbitPropagator propagator; // owns a TwoBodyGravity + one CelestialPerturbation per perturber
    std::vector<CelestialPerturbation *> perturbationTerms; // raw observing pointers into propagator, for jd refresh
  };

  CelestialSystem *celestialSystem_ = nullptr;
  double currentJd_ = 2451545.0;
  std::vector<OrbitalModeEntry> orbitalModeEntries_;

  const OrbitalModeEntry *findOrbitalModeEntry(const RigidBody *body) const;
  void stepOrbitalMode(float dt);
};
