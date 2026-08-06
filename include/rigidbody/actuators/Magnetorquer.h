#pragma once

#include <rigidbody/ForceGenerator.h>
#include <glm/glm.hpp>

class RigidBody;

// A magnetic torque rod/coil: commands a magnetic dipole moment along its
// own axis, which reacts against the ambient magnetic field to produce a
// torque (torque = m x B, same cross-product law a real torque rod obeys).
// Unlike a ReactionWheel, a magnetorquer has no moving parts and never
// saturates its own momentum storage -- but it also can't produce torque
// along the field direction (m x B is zero there), and its authority scales
// with the local field strength, which is weak in LEO (tens of uT). Good
// for slow detumbling/momentum dumping, not for fast precise slews.
//
// ambientFieldWorld is NOT sampled internally -- unlike UniformGravity's
// constant acceleration, the ambient field varies with orbital
// position/time (see rigidbody/environment/uniform/UniformMagneticField.h
// or rigidbody/environment/central_body/CentralBodyMagneticField.h), so
// the scenario is expected to write it here once per frame before
// PhysicsWorld::step() calls apply(), the same way ADCS::gravity is set
// externally rather than hardcoded.
class Magnetorquer : public ForceGenerator
{
public:
  glm::vec3 mountPositionBody; // where the rod is mounted (body frame)
  glm::vec3 axisBody;          // dipole moment direction when commanded positive (body frame, normalized)

  float maxDipoleMoment;      // max |m| (A*m^2)
  float commandedDipoleMoment; // signed dipole moment along axisBody (A*m^2)

  glm::vec3 ambientFieldWorld{0.0f}; // set externally each frame, Tesla, world frame

  // Fault model, same role as ReactionWheel::healthFactor: fraction of
  // commandedDipoleMoment actually delivered (1 = healthy, 0 = dead coil).
  float healthFactor = 1.0f;

  Magnetorquer(
      glm::vec3 mountPosBody,
      glm::vec3 axisBody,
      float maxDipoleMomentAm2);

  // Command a dipole moment output (-1 to 1 normalized, or raw A*m^2)
  void commandNormalized(float command); // -1 to 1
  void commandDipoleMoment(float momentAm2); // direct A*m^2

  // Applies torque = m x B to the body (call each physics step, after
  // ambientFieldWorld has been set for this frame).
  void apply(RigidBody &body, float dt) override;

  // Get world-space position/axis/dipole-moment for visualization.
  glm::vec3 getWorldMountPosition(const RigidBody &body) const;
  glm::vec3 getWorldAxis(const RigidBody &body) const;
  glm::vec3 getWorldDipoleMoment(const RigidBody &body) const; // axis * commandedDipoleMoment * healthFactor

  float getSaturationRatio() const; // -1 to 1, commandedDipoleMoment / maxDipoleMoment
};
