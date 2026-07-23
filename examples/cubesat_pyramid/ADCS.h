#pragma once
#include <glm/glm.hpp>
#include <rigidbody/actuators/ReactionWheel.h>
#include <rigidbody/sensors/IMU.h>
#include <rigidbody/RigidBody.h>
#include "Controllers.h"

// Pointing modes. Nadir/Sun/Target/Slew/Fine-pointing all resolve to a
// target attitude and go through the normal PID/LQR/cascaded quaternion
// controller (with different tuning per mode -- see modeTuning() in
// ADCS.cpp); Detumble bypasses attitude control entirely and just damps
// rate, since a freshly-deployed, tumbling satellite has no attitude
// reference worth chasing yet.
enum class PointingMode
{
  NADIR,         // body +Z toward a fixed "down" direction (no orbit in this sim, so no real nadir vector)
  SUN_POINTING,  // body +Z toward the sun sphere
  DETUMBLE,      // damp angular rate toward zero; ignores attitude entirely
  TARGET,        // body +Z toward `target`, default tuning
  SLEW,          // body +Z toward `target`, tuned for a fast large-angle move
  FINE_POINTING, // body +Z toward `target`, tuned for a slow, precise settle
};

enum class ControllerType
{
  PID,
  LQR,
  CASCADED
};

class ADCS
{
public:
  PointingMode mode = PointingMode::TARGET;
  ControllerType controllerType = ControllerType::PID;

  glm::vec3 target = {0.5f, 0.5f, 0.5f};      // world position for TARGET/SLEW/FINE_POINTING
  glm::vec3 sunPosition = {0.0f, 0.0f, 0.0f}; // world position for SUN_POINTING

  glm::quat targetAttitude;
  glm::vec3 torqueCommand;
  std::vector<float> wheelCommands; // wheelCommands[i] -> torque for wheels[i]

  // Attitude estimate, propagated by integrating the IMU's gyro reading
  // every run() (a strapdown INS with no absolute reference to correct
  // it -- see run()). This is what guidance/control actually use; ground
  // truth (body->orientation) is never read here.
  glm::quat estimatedAttitude{1, 0, 0, 0};

  // Hardware references
  RigidBody *body = nullptr;
  std::vector<ReactionWheel *> wheels;
  IMU *imu = nullptr;

  // Ambient gravity the body is in, needed to interpret the IMU's
  // accelerometer as specific force (see IMU::sample). The cubesat
  // scenarios run at zero gravity, but ADCS doesn't hardcode that.
  glm::vec3 gravity{0.0f};

public:
  ADCS() = default;
  ADCS(RigidBody *body_, const std::vector<ReactionWheel *> &wheels_, IMU *imu_);
  ~ADCS();

  void run(float dt);
  void resetController();

  void computeGuidance(float dt);
  void computeControl(glm::quat attitude, glm::vec3 rate, float dt);
  void allocateActuators();
  void sendCommands();

private:
  PIDController pid;
  LQRController lqr;
  CascadedController cascaded;

  PointingMode lastTunedMode; // re-tune only when mode actually changes
  float detumbleKd = 0.0f;    // rate-damping gain for DETUMBLE, derived from inertia at construction

  // Applies the per-mode gain/rate-limit preset (see ADCS.cpp) to whichever
  // controller is active. Called automatically from run() when `mode`
  // changes.
  void retuneForMode();

  // DETUMBLE's control law: pure rate damping, no attitude term.
  glm::vec3 computeDetumbleTorque(const glm::vec3 &rate) const;
};
