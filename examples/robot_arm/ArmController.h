#pragma once
#include <rigidbody/RigidBody.h>
#include <rigidbody/PhysicsWorld.h>
#include <rigidbody/Constraint.h>
#include <glm/glm.hpp>

// Mission phases for a single pick: mirrors the mission-sequencer pattern
// used elsewhere in this project (FlightSoftware, LandingSoftware) rather
// than being scenario-specific.
enum class ArmPhase
{
  IDLE,     // holding home pose, waiting for the pick command
  APPROACH, // move the end effector above the cube
  DESCEND,  // move the end effector down to the cube
  GRASP,    // weld the end effector to the cube
  LIFT,     // move back up, carrying the cube
  DONE,     // holding the lifted pose
};

// A 3-DOF serial arm (base yaw, shoulder pitch, elbow pitch) that picks up a
// cube using closed-form 2-link inverse kinematics plus each joint's own
// built-in HingeConstraint velocity-servo motor -- no custom torque math
// needed here, unlike the gimbal-driven scenarios elsewhere in this
// project, since HingeConstraint already provides exactly the actuator
// this problem wants.
//
// Kinematics (all in the arm's own frame, shoulder pivot at the origin):
//  - Home pose (all joint angles = 0) has the arm fully extended straight
//    up (+Z), matching each HingeConstraint's own "angle = 0 at
//    construction" convention directly -- no offset bookkeeping needed.
//  - Yaw rotates about world +Z, aiming the arm's vertical plane at the
//    target's azimuth.
//  - Shoulder/elbow are a standard 2-link planar arm within that plane:
//    r = L1*sin(shoulder) + L2*sin(shoulder+elbow)   (reach, horizontal)
//    h = L1*cos(shoulder) + L2*cos(shoulder+elbow)   (height above shoulder)
//    solved in closed form via the law of cosines, then clamped to the
//    arm's actual reachable annulus so an out-of-range target never
//    produces a NaN from acos().
//
// Grasping is a simplified, deliberately non-physical model: this is a
// rigid-body engine with no soft/frictional finger-contact simulation, so
// "closing the gripper" is a FixedConstraint welding the forearm to the
// cube once the end effector arrives within tolerance -- the same
// kinematic-attach technique this project already uses for rocket staging,
// not a real grasp force. There's no separate finger geometry simulated
// either; the "gripper" is a single attachment point at the forearm's tip.
class ArmController
{
public:
  ArmController(PhysicsWorld &world,
               HingeConstraint *yaw, HingeConstraint *shoulder, HingeConstraint *elbow,
               RigidBody *forearm, RigidBody *cube,
               glm::vec3 shoulderPivotWorld, float upperArmLength, float forearmLength,
               float forearmTipLocalZ);

  // Advances the sequencer, solves IK for the current phase's target, and
  // drives all three joint motors toward it. Call once per frame, before
  // PhysicsWorld::step(). `pickCommand` is the only manual input.
  void update(float dt, bool pickCommand);

  ArmPhase phase() const { return m_phase; }
  const char *phaseName() const;

  glm::vec3 targetPosition() const { return m_currentTarget; }
  glm::vec3 endEffectorPosition() const;

  float yawAngle() const;
  float shoulderAngle() const;
  float elbowAngle() const;

  // Cosmetic only (see class comment) -- true while the gripper should
  // render closed (GRASP/LIFT/DONE), for the scenario's own drawing code.
  bool gripperClosed() const { return m_phase == ArmPhase::GRASP || m_phase == ArmPhase::LIFT || m_phase == ArmPhase::DONE; }

private:
  void enterPhase(ArmPhase next);
  void solveIK(const glm::vec3 &targetWorld, float &outYaw, float &outShoulder, float &outElbow) const;
  void driveJoint(HingeConstraint *joint, float targetAngle);
  bool arrivedAtTarget() const;

  PhysicsWorld &m_world;
  HingeConstraint *m_yaw;
  HingeConstraint *m_shoulder;
  HingeConstraint *m_elbow;
  RigidBody *m_forearm;
  RigidBody *m_cube;
  FixedConstraint *m_grasp = nullptr;

  glm::vec3 m_shoulderPivotWorld;
  float m_upperArmLength;
  float m_forearmLength;
  float m_forearmTipLocalZ; // local +Z offset from forearm's own origin to its tip

  ArmPhase m_phase = ArmPhase::IDLE;
  float m_phaseTime = 0.0f;
  glm::vec3 m_currentTarget{0.0f};
  // Both captured once at grasp time, not derived from the cube's position
  // during LIFT -- the cube is welded to the forearm and rising by then,
  // so re-deriving the target from its live position would make LIFT chase
  // an endlessly-receding target instead of aiming at a fixed point.
  glm::vec2 m_liftTargetXY{0.0f};
  float m_liftTargetHeight = 0.0f;
};
