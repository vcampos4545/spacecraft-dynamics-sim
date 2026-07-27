#include <vgl/vgl.h>
#include <rigidbody/PhysicsWorld.h>
#include <rigidbody/Constraint.h>
#include <rigidbody/environment/Gravity.h>
#include <rigidbody/actuators/ReactionWheel.h>
#include <rigidbody/sensors/IMU.h>
#include "common/World.h"
#include "common/Telemetry.h"
#include <cstdio>
#include <cmath>
#include <memory>
#include <glm/gtc/quaternion.hpp>

// The real reaction-wheel-balancer firmware's portable "flight software" --
// compiled here completely unmodified, from its actual location on disk
// (see CMakeLists.txt's SITL_FIRMWARE_DIR). Everything hardware-specific it
// would normally depend on (Arduino core, the Adafruit MPU6050 driver) is
// satisfied instead by examples/sitl/arduino_shim/, which this target adds
// to its include path ahead of anywhere a real one might be found.
#include <BalanceController.h>
#include <IMUFilter.h>
#include <Config.h>
#include <Adafruit_MPU6050.h> // the shim -- for SITLBridge::injectReading/injectFailure
#include <Arduino.h>          // the shim -- for RAD_TO_DEG/DEG_TO_RAD, used below for display

// ---------------------------------------------------------------------------
// SITL (Software-in-the-Loop): the real balance firmware's control code
// (BalanceController.cpp, IMUFilter.cpp -- see the includes above) runs
// completely unmodified in this process, driving and reading a simulated
// triangle-on-a-pivot instead of real hardware. This is different from
// HITL: there's no ESP32, no serial link, no I2C -- the firmware's C++ is
// linked directly into this executable and called as ordinary functions,
// which is possible here specifically because reaction_wheel_balancer
// already separates its portable control logic (BalanceController,
// IMUFilter) from hardware glue (main.cpp's I2C/SimpleFOC/watchdog code,
// which is NOT compiled here -- it's genuinely ESP32-specific and isn't
// meant to be portable).
//
// Each frame:
//   1. Sample this project's own IMU sensor model (rigidbody/sensors/IMU.h)
//      on the simulated frame body -- realistic noise/bias, not ground
//      truth, same as any other scenario in this repo.
//   2. Feed that reading into the firmware via SITLBridge::injectReading()
//      (see arduino_shim/Adafruit_MPU6050.h for why injection is a global
//      free function rather than a method call).
//   3. Call the REAL IMUFilter::update() and BalanceController::update() --
//      not a reimplementation -- to get a motor voltage command.
//   4. Convert volts -> torque via the same kt/R relationship
//      reaction_wheel_balancer's own README.md derives for its LQR model,
//      and command the simulated ReactionWheel with it.
//   5. Step the physics, exactly like every other scenario here.
//
// The physical plant (mass, pivot-to-CG distance, inertias, motor
// constants) uses the actual measured values from
// reaction_wheel_balancer/optimizer.ipynb, not placeholders -- the point of
// SITL is testing the real control code against a plant that actually
// resembles the hardware it's tuned for.
//
// Controls:
//   [Space] nudge -- kicks the frame with a disturbance to watch it recover
//   [F]     inject one simulated IMU read failure (exercises the firmware's
//           imuFailStreak safety logic against a fault, not a real I2C wedge)
//   [R]     reset the frame to upright and at rest
// ---------------------------------------------------------------------------

namespace SimConfig
{
  constexpr float CAMERA_NEAR = 0.01f;
  constexpr float CAMERA_FAR = 50.0f;
  constexpr float CAMERA_FOV = 45.0f;

  constexpr int TELEMETRY_HISTORY_SAMPLES = 400;

  constexpr float PIVOT_HEIGHT_M = 0.3f; // just for a sensible camera/ground height, not physical
  constexpr float NUDGE_RAD_PER_S = 3.0f;

  // Real measured values (reaction_wheel_balancer/optimizer.ipynb) -- the
  // same numbers that repo's own LQR derivation uses, not independently
  // chosen for this sim.
  constexpr float ASSEMBLY_MASS_KG = 0.0869f;       // M
  constexpr float PIVOT_TO_CG_M = 0.085f;           // L
  constexpr float PIVOT_INERTIA_KGM2 = 0.000897507898f; // Ip (about the pivot/local-X axis)
  constexpr float WHEEL_INERTIA_KGM2 = 8.52e-5f;    // Iw
  constexpr float MOTOR_KV = 220.0f;
  constexpr float MOTOR_R_OHMS = 2.3f;              // phase resistance
  constexpr float GRAVITY_MS2 = 9.81f;
}

// kt from KV, same conversion reaction_wheel_balancer/README.md uses:
// kt = 60 / (2*pi*KV).
static float motorTorqueConstant()
{
  return 60.0f / (2.0f * glm::pi<float>() * SimConfig::MOTOR_KV);
}

// ---------------------------------------------------------------------------
// Draw helpers
// ---------------------------------------------------------------------------
static void drawFrame(GUI &gui, RigidBody *frame, bool balancing)
{
  glm::vec3 color = balancing ? glm::vec3(0.3f, 0.85f, 0.4f) : glm::vec3(0.7f, 0.4f, 0.2f);
  // A flat plate roughly standing in for the physical triangle; this demo
  // is about the control loop, not the frame's exact geometry.
  gui.drawBox(frame->position, glm::vec3(0.01f, 0.09f, 2.0f * SimConfig::PIVOT_TO_CG_M),
             frame->orientation, color);
}

static void drawWheel(GUI &gui, ReactionWheel &wheel, RigidBody *frame, float wheelAngleRad)
{
  glm::quat spin = glm::angleAxis(wheelAngleRad, glm::vec3(1, 0, 0));
  glm::quat rotation = frame->orientation * spin;
  gui.drawCylinder(wheel.getWorldMountPosition(*frame), 0.03f, 0.012f,
                   glm::vec3(1, 0, 0), rotation, {0.6f, 0.6f, 0.65f});
  // A radial marker line so the wheel's spin is visible, not just implied
  // by its (otherwise rotationally symmetric) cylinder.
  glm::vec3 markerLocal = rotation * glm::vec3(0, 0.03f, 0);
  gui.drawLine(wheel.getWorldMountPosition(*frame), wheel.getWorldMountPosition(*frame) + markerLocal,
              {1.0f, 0.9f, 0.2f}, 2.0f);
}

static const char *stateName(BalanceController::State s)
{
  switch (s)
  {
  case BalanceController::State::Idle: return "Idle";
  case BalanceController::State::WaitUpright: return "WaitUpright";
  case BalanceController::State::Balancing: return "Balancing";
  case BalanceController::State::Fallen: return "Fallen";
  }
  return "Unknown";
}

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------
int main()
{
  GUI gui(1000, 700, "Reaction Wheel Balancer (SITL)");
  World scene(WorldType::DEFAULT);
  scene.apply(gui);
  gui.camera
      .setUp({0, 0, 1})
      .setClipPlanes(SimConfig::CAMERA_NEAR, SimConfig::CAMERA_FAR)
      .setFOV(SimConfig::CAMERA_FOV);

  OrbitalCamera orbit(0.6f, 25.0f, 15.0f, {0, 0, SimConfig::PIVOT_HEIGHT_M});
  orbit.setMinDistance(0.15f)
      .setMaxDistance(5.0f)
      .setZoomSensitivity(0.5f)
      .setPanSensitivity(0.05f);

  glm::vec2 lastMousePos = gui.getMousePosition();

  // =================== DEFINE THE PLANT ===================
  PhysicsWorld world;
  world.addGlobalForceGenerator(std::make_unique<Gravity>(glm::vec3(0.0f, 0.0f, -SimConfig::GRAVITY_MS2)));

  glm::vec3 pivotPos(0, 0, SimConfig::PIVOT_HEIGHT_M);

  RigidBody *anchor = world.createBody(RigidBodyShape::BOX, {0.01f, 0.01f, 0.01f}, 1.0f);
  anchor->position = pivotPos;
  anchor->invMass = 0.0f; // static -- the world-fixed end of the pivot

  RigidBody *frame = world.createBody(
      RigidBodyShape::BOX,
      glm::vec3(0.01f, 0.09f, 2.0f * SimConfig::PIVOT_TO_CG_M),
      SimConfig::ASSEMBLY_MASS_KG);
  frame->position = pivotPos + glm::vec3(0, 0, SimConfig::PIVOT_TO_CG_M);
  // The bifilar-pendulum-measured Ip only really constrains the pivot axis
  // (local X); the other two diagonal entries aren't load-bearing for this
  // 1-DOF problem, so they're set to the same value rather than guessed.
  frame->setInertiaTensor(glm::mat3(SimConfig::PIVOT_INERTIA_KGM2));

  // local +X = pivot axis (matches IMU_PIVOT_AXIS=X in Config.h), local +Z
  // = up when balanced (matches IMU_UP_AXIS=Z) -- chosen so the sim's own
  // body-local axes are already the firmware's axes with no remapping.
  HingeConstraint *pivot = world.addHingeConstraint(anchor, frame, pivotPos, glm::vec3(1, 0, 0));
  pivot->setLimits(glm::radians(-80.0f), glm::radians(80.0f)); // mechanical range before hitting a stand

  ReactionWheel *wheel;
  {
    auto w = std::make_unique<ReactionWheel>(
        glm::vec3(0.0f), glm::vec3(1, 0, 0),
        motorTorqueConstant() * MOTOR_VOLTAGE_LIMIT / SimConfig::MOTOR_R_OHMS, // stall torque at the firmware's own voltage limit
        2000.0f,
        SimConfig::WHEEL_INERTIA_KGM2);
    wheel = w.get();
    frame->addForceGenerator(std::move(w));
  }

  IMU imuSensor; // mounted at the frame's own center of mass, for simplicity

  // =================== THE REAL FIRMWARE ===================
  BalanceController balance;
  balance.begin();
  IMUFilter imuFilter;
  imuFilter.begin();

  // calibrateGyro() loops GYRO_CAL_SAMPLES times internally reading
  // whatever's currently injected -- give it one steady "at rest, upright"
  // reading before calling it, matching "keep the triangle still" from the
  // real calibration procedure.
  SITLBridge::injectReading({0, 0, SimConfig::GRAVITY_MS2}, {0, 0, 0});
  imuFilter.calibrateGyro();

  int imuFailStreak = 0;

  // =================== TELEMETRY ===================
  TelemetryChannel thetaEstHistory(SimConfig::TELEMETRY_HISTORY_SAMPLES);
  TelemetryChannel thetaTruthHistory(SimConfig::TELEMETRY_HISTORY_SAMPLES);
  TelemetryChannel thetaDotHistory(SimConfig::TELEMETRY_HISTORY_SAMPLES);
  TelemetryChannel wheelRpmHistory(SimConfig::TELEMETRY_HISTORY_SAMPLES);
  TelemetryChannel voltageHistory(SimConfig::TELEMETRY_HISTORY_SAMPLES);

  float missionTime = 0.0f;
  float wheelAngleRad = 0.0f;
  float lastTime = glfwGetTime();

  while (!gui.shouldClose())
  {
    float time = glfwGetTime();
    float dt = time - lastTime;
    lastTime = time;
    if (dt <= 0.0f)
      continue;

    // =================== INPUT ===================
    if (gui.isKeyJustPressed(GLFW_KEY_SPACE))
      frame->angularVelocity += glm::vec3(SimConfig::NUDGE_RAD_PER_S, 0, 0);
    if (gui.isKeyJustPressed(GLFW_KEY_F))
      SITLBridge::injectFailure();
    if (gui.isKeyJustPressed(GLFW_KEY_R))
    {
      frame->position = pivotPos + glm::vec3(0, 0, SimConfig::PIVOT_TO_CG_M);
      frame->orientation = glm::quat(1, 0, 0, 0);
      frame->velocity = glm::vec3(0.0f);
      frame->angularVelocity = glm::vec3(0.0f);
    }

    glm::vec2 mousePos = gui.getMousePosition();
    glm::vec2 mouseDelta = mousePos - lastMousePos;
    lastMousePos = mousePos;
    orbit.handleInput(gui, mouseDelta, gui.getScrollDelta());
    orbit.setTarget(frame->position);
    orbit.applyToCamera(gui.camera);

    // =================== SENSE (sim -> real firmware) ===================
    IMU::Reading reading = imuSensor.sample(*frame, glm::vec3(0, 0, -SimConfig::GRAVITY_MS2), dt);
    SITLBridge::injectReading({reading.accel.x, reading.accel.y, reading.accel.z},
                              {reading.gyro.x, reading.gyro.y, reading.gyro.z});

    bool imuOk = imuFilter.update(); // the real firmware's fused theta/thetaDot
    imuFailStreak = imuOk ? 0 : imuFailStreak + 1;

    // =================== CONTROL (the real firmware) ===================
    unsigned long nowMs = (unsigned long)(missionTime * 1000.0f);
    float wheelVel = wheel->currentSpeed;
    float u = 0.0f;
    if (imuFailStreak < I2C_FAIL_THRESHOLD)
      u = balance.update(imuFilter.getAngleRad(), imuFilter.getRateRad(), wheelVel, nowMs);
    // else: mirrors main.cpp -- don't trust a stale angle, leave u at 0

    // =================== ACT (real firmware output -> sim actuator) ======
    float torqueNm = motorTorqueConstant() * u / SimConfig::MOTOR_R_OHMS;
    wheel->commandTorque(torqueNm);

    // =================== PHYSICS ===================
    world.step(dt);
    missionTime += dt;
    wheelAngleRad += wheelVel * dt;

    // =================== TELEMETRY ===================
    thetaEstHistory.push(imuFilter.getAngleRad() * (float)RAD_TO_DEG);
    thetaTruthHistory.push(pivot->getAngle() * (float)RAD_TO_DEG);
    thetaDotHistory.push(imuFilter.getRateRad());
    wheelRpmHistory.push(wheelVel * 60.0f / (2.0f * glm::pi<float>()));
    voltageHistory.push(u);

    // =================== DRAW ===================
    gui.beginFrame();
    scene.draw(gui, 1.0f, 0.1f);

    drawFrame(gui, frame, balance.getState() == BalanceController::State::Balancing);
    drawWheel(gui, *wheel, frame, wheelAngleRad);
    gui.drawSphere(pivotPos, 0.008f, {0.8f, 0.8f, 0.85f});

    ImGui::Begin("SITL: Reaction Wheel Balancer");
    ImGui::Text("Firmware state: %s", stateName(balance.getState()));
    ImGui::Text("Mission time: %.1f s", missionTime);
    ImGui::Text("IMU fail streak: %d", imuFailStreak);
    ImGui::TextColored(ImVec4(0.6f, 0.8f, 1.0f, 1.0f),
                       "[Space] nudge   [F] inject IMU fault   [R] reset");

    ImGui::SeparatorText("Live gain tuning (same fields the real serial protocol sets)");
    ImGui::SliderFloat("Kp", &balance.Kp, 0.0f, 60.0f);
    ImGui::SliderFloat("Kd", &balance.Kd, 0.0f, 10.0f);
    ImGui::SliderFloat("Kw", &balance.Kw, 0.0f, 0.1f);
    float trimDeg = balance.trim * (float)RAD_TO_DEG;
    if (ImGui::SliderFloat("Trim (deg)", &trimDeg, -10.0f, 10.0f))
      balance.trim = trimDeg * (float)DEG_TO_RAD;

    ImGui::SeparatorText("Sensors -- firmware estimate vs. simulated ground truth");
    plotChannel("Theta (firmware, fused)", thetaEstHistory, "deg");
    plotChannel("Theta (sim ground truth)", thetaTruthHistory, "deg");
    plotChannel("Theta dot (firmware)", thetaDotHistory, "rad/s");

    ImGui::SeparatorText("Actuator");
    plotChannel("Wheel speed", wheelRpmHistory, "RPM");
    plotChannel("Motor voltage command", voltageHistory, "V");
    ImGui::End();

    gui.endFrame();
  }
  return 0;
}
