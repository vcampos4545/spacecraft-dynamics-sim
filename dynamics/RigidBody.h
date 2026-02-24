#pragma once
#include <memory>
#include <vector>
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>

class ForceGenerator;

enum class RigidBodyShape
{
  BOX,
  SPHERE,
  CYLINDER
};

class RigidBody
{
public:
  // State
  glm::vec3 position;
  glm::vec3 velocity;

  glm::quat orientation;
  glm::vec3 angularVelocity;

  float mass;
  float invMass;

  glm::mat3 inertiaTensor;
  glm::mat3 invInertiaTensor;

  // Force/torque state
  std::vector<std::unique_ptr<ForceGenerator>> forceGenerators;
  glm::vec3 forceAccum;
  glm::vec3 torqueAccum;

  // Shape data
  RigidBodyShape shape;
  glm::vec3 size; // (w, h, d), (r, l, nil), or (r, nil, nil)
  float density;  // kg/m^3

public:
  RigidBody(RigidBodyShape shape, const glm::vec3 &size);
  ~RigidBody();
  // Factory methods for construction
  static std::unique_ptr<RigidBody> fromDensity(RigidBodyShape shape,
                                                const glm::vec3 &size,
                                                float density);

  static std::unique_ptr<RigidBody> fromMass(RigidBodyShape shape,
                                             const glm::vec3 &size,
                                             float mass);

  void applyForce(const glm::vec3 &force);
  void applyTorque(const glm::vec3 &torque);
  void applyForceAtPoint(const glm::vec3 &force,
                         const glm::vec3 &worldPoint);
  void addForceGenerator(std::unique_ptr<ForceGenerator> generator);
  void applyForceGenerators(float dt);

  void integrate(float dt);
  void resolveGroundCollision(float groundZ = 0.0f,
                              float restitution = 0.3f,
                              float friction = 0.5f);
  void resolveBoxGroundCollision(float groundZ,
                                 float restitution,
                                 float friction);
  void resolveSphereGroundCollision(float groundZ,
                                    float restitution,
                                    float friction);
  void resolveCylinderGroundCollision(float groundZ,
                                      float restitution,
                                      float friction);

  // Get velocity at a world-space point on the body
  glm::vec3 getVelocityAtPoint(const glm::vec3 &worldPoint) const;

private:
  void computeInertiaFromMass();
  float computeVolume() const;

  void integrateLinear(float dt);
  void integrateAngular(float dt);
  void clearAccumulators();

  // Get all 8 corners of the box in world space
  void getWorldCorners(glm::vec3 corners[8]) const;

  // Helper for resolving a single contact point
  void resolveContactPoint(const glm::vec3 &contactPoint,
                           float groundZ,
                           const glm::vec3 &groundNormal,
                           const glm::mat3 &invI_world,
                           float restitution,
                           float friction);
};
