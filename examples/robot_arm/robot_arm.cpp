#include <vgl/vgl.h>
#include <rigidbody/PhysicsWorld.h>
#include <rigidbody/Constraint.h>
#include <rigidbody/environment/Gravity.h>
#include "ArmController.h"
#include "common/World.h"
#include "common/Telemetry.h"
#include "common/ImGuiLayer.h"
#include <cmath>
#include <memory>
#include <glm/gtc/constants.hpp>

// ---------------------------------------------------------------------------
// A 3-DOF robot arm (base yaw, shoulder pitch, elbow pitch) that picks up a
// cube: closed-form inverse kinematics drives each joint's own
// HingeConstraint velocity-servo motor toward a target end-effector
// position, and "grasping" is a FixedConstraint welding the forearm to the
// cube once the end effector arrives -- a kinematic attach, not a
// simulated frictional grip (this is a rigid-body engine with no
// soft-contact finger simulation). See ArmController.h for the full IK
// derivation and mission sequencer.
//
// Controls:
//   [P] pick -- commands the sequencer to approach, descend, grasp, and
//       lift the cube; everything after that is autonomous
// ---------------------------------------------------------------------------
namespace Config
{
  constexpr float CAMERA_NEAR = 0.01f;
  constexpr float CAMERA_FAR = 20.0f;
  constexpr float CAMERA_FOV = 45.0f;

  constexpr float GRID_SIZE = 1.0f;
  constexpr float GRID_STEP = 0.1f;

  constexpr int TELEMETRY_HISTORY_SAMPLES = 400;

  // Link geometry (meters) -- an arm sized for a desktop-scale pick, not a
  // real robot's datasheet.
  const glm::vec3 BASE_SIZE{0.16f, 0.16f, 0.10f};
  const glm::vec3 TURRET_SIZE{0.10f, 0.10f, 0.08f};
  const glm::vec3 UPPER_ARM_SIZE{0.04f, 0.04f, 0.30f};
  const glm::vec3 FOREARM_SIZE{0.035f, 0.035f, 0.25f};

  constexpr float TURRET_MASS_KG = 0.3f;
  constexpr float UPPER_ARM_MASS_KG = 0.5f;
  constexpr float FOREARM_MASS_KG = 0.3f;

  constexpr float CUBE_SIZE_M = 0.05f;
  constexpr float CUBE_MASS_KG = 0.05f;
  const glm::vec3 CUBE_START_POS{0.35f, 0.15f, CUBE_SIZE_M * 0.5f}; // resting on the ground

  constexpr float GRAVITY_MS2 = 9.81f;
}

// ---------------------------------------------------------------------------
// Draw helpers
// ---------------------------------------------------------------------------
static void drawLink(GUI &gui, RigidBody *link, glm::vec3 color)
{
  gui.drawBox(link->position, link->size, link->orientation, color);
}

// Two small cosmetic "finger" markers at the end effector -- purely
// visual, not separate physics bodies (see ArmController.h). Spread apart
// when open, drawn together when the gripper should look closed.
static void drawGripper(GUI &gui, glm::vec3 tipPos, glm::quat forearmOrientation, bool closed)
{
  float spread = closed ? 0.006f : 0.03f;
  glm::vec3 side = forearmOrientation * glm::vec3(1, 0, 0) * spread;
  glm::vec3 color = closed ? glm::vec3(0.3f, 0.85f, 0.4f) : glm::vec3(0.8f, 0.8f, 0.85f);
  gui.drawBox(tipPos + side, glm::vec3(0.008f, 0.03f, 0.02f), forearmOrientation, color);
  gui.drawBox(tipPos - side, glm::vec3(0.008f, 0.03f, 0.02f), forearmOrientation, color);
}

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------
int main()
{
  GUI gui(1000, 700, "Robot Arm Pick");
  ImGuiLayer imguiLayer(gui);
  World scene(WorldType::DEFAULT);
  scene.apply(gui);
  gui.camera
      .setUp({0, 0, 1})
      .setClipPlanes(Config::CAMERA_NEAR, Config::CAMERA_FAR)
      .setFOV(Config::CAMERA_FOV);

  OrbitalCamera orbit(1.1f, 35.0f, 20.0f, {0.15f, 0.05f, 0.3f});
  orbit.setMinDistance(0.2f)
      .setMaxDistance(5.0f)
      .setZoomSensitivity(0.5f)
      .setPanSensitivity(0.05f);

  glm::vec2 lastMousePos = gui.getMousePosition();

  // =================== DEFINE THE ARM ===================
  PhysicsWorld world;
  world.addGlobalForceGenerator(std::make_unique<Gravity>(glm::vec3(0.0f, 0.0f, -Config::GRAVITY_MS2)));

  RigidBody *base = world.createBody(RigidBodyShape::BOX, Config::BASE_SIZE, 1.0f);
  base->position = glm::vec3(0, 0, Config::BASE_SIZE.z * 0.5f);
  base->invMass = 0.0f; // static -- bolted to the bench
  // invMass alone only stops linear motion -- without also zeroing
  // invInertiaTensor, reaction torque from the yaw joint's motor would
  // still spin the "static" base, corrupting every downstream joint's
  // world-frame assumptions (this was a real, previously-shipped bug here).
  base->invInertiaTensor = glm::mat3(0.0f);

  glm::vec3 yawPivot(0, 0, Config::BASE_SIZE.z);
  RigidBody *turret = world.createBody(RigidBodyShape::BOX, Config::TURRET_SIZE, Config::TURRET_MASS_KG);
  turret->position = yawPivot + glm::vec3(0, 0, Config::TURRET_SIZE.z * 0.5f);
  HingeConstraint *yaw = world.addHingeConstraint(base, turret, yawPivot, glm::vec3(0, 0, 1));

  glm::vec3 shoulderPivot = yawPivot + glm::vec3(0, 0, Config::TURRET_SIZE.z);
  RigidBody *upperArm = world.createBody(RigidBodyShape::BOX, Config::UPPER_ARM_SIZE, Config::UPPER_ARM_MASS_KG);
  upperArm->position = shoulderPivot + glm::vec3(0, 0, Config::UPPER_ARM_SIZE.z * 0.5f);
  // Home pose = fully extended straight up (+Z), matching HingeConstraint's
  // own "angle = 0 at construction" convention -- see ArmController.h.
  HingeConstraint *shoulder = world.addHingeConstraint(turret, upperArm, shoulderPivot, glm::vec3(0, 1, 0));

  glm::vec3 elbowPivot = shoulderPivot + glm::vec3(0, 0, Config::UPPER_ARM_SIZE.z);
  RigidBody *forearm = world.createBody(RigidBodyShape::BOX, Config::FOREARM_SIZE, Config::FOREARM_MASS_KG);
  forearm->position = elbowPivot + glm::vec3(0, 0, Config::FOREARM_SIZE.z * 0.5f);
  HingeConstraint *elbow = world.addHingeConstraint(upperArm, forearm, elbowPivot, glm::vec3(0, 1, 0));

  float forearmTipLocalZ = Config::FOREARM_SIZE.z * 0.5f;

  // =================== DEFINE THE CUBE ===================
  RigidBody *cube = world.createBody(RigidBodyShape::BOX, glm::vec3(Config::CUBE_SIZE_M), Config::CUBE_MASS_KG);
  cube->position = Config::CUBE_START_POS;

  ArmController arm(world, yaw, shoulder, elbow, forearm, cube,
                    shoulderPivot, Config::UPPER_ARM_SIZE.z, Config::FOREARM_SIZE.z, forearmTipLocalZ);

  // =================== TELEMETRY ===================
  TelemetryChannel yawDegHistory(Config::TELEMETRY_HISTORY_SAMPLES);
  TelemetryChannel shoulderDegHistory(Config::TELEMETRY_HISTORY_SAMPLES);
  TelemetryChannel elbowDegHistory(Config::TELEMETRY_HISTORY_SAMPLES);
  TelemetryChannel errorHistory(Config::TELEMETRY_HISTORY_SAMPLES);
  TelemetryChannel cubeHeightHistory(Config::TELEMETRY_HISTORY_SAMPLES);

  float missionTime = 0.0f;
  float lastTime = glfwGetTime();

  while (!gui.shouldClose())
  {
    float time = glfwGetTime();
    float dt = time - lastTime;
    lastTime = time;
    if (dt <= 0.0f)
      continue;

    // =================== INPUT / CONTROL ===================
    bool pickCommand = gui.isKeyPressed(GLFW_KEY_P);
    arm.update(dt, pickCommand);

    glm::vec2 mousePos = gui.getMousePosition();
    glm::vec2 mouseDelta = mousePos - lastMousePos;
    lastMousePos = mousePos;
    // Don't drive the orbit camera from mouse input ImGui itself wants
    // (e.g. dragging the telemetry window around) -- otherwise moving a
    // panel also spins the camera underneath it.
    if (!ImGui::GetIO().WantCaptureMouse)
      orbit.handleInput(gui, mouseDelta, gui.getScrollDelta());
    orbit.applyToCamera(gui.camera);

    // =================== PHYSICS ===================
    world.step(dt);
    missionTime += dt;

    // =================== TELEMETRY ===================
    yawDegHistory.push(glm::degrees(arm.yawAngle()));
    shoulderDegHistory.push(glm::degrees(arm.shoulderAngle()));
    elbowDegHistory.push(glm::degrees(arm.elbowAngle()));
    errorHistory.push(glm::length(arm.endEffectorPosition() - arm.targetPosition()) * 1000.0f);
    cubeHeightHistory.push(cube->position.z);

    // =================== DRAW ===================
    gui.beginFrame();
    imguiLayer.beginFrame();
    scene.draw(gui, Config::GRID_SIZE, Config::GRID_STEP);

    drawLink(gui, base, {0.35f, 0.35f, 0.4f});
    drawLink(gui, turret, {0.45f, 0.45f, 0.5f});
    drawLink(gui, upperArm, {0.75f, 0.55f, 0.2f});
    drawLink(gui, forearm, {0.8f, 0.65f, 0.3f});
    drawGripper(gui, arm.endEffectorPosition(), forearm->orientation, arm.gripperClosed());
    gui.drawBox(cube->position, glm::vec3(Config::CUBE_SIZE_M), cube->orientation, {0.85f, 0.2f, 0.2f});

    // Small marker at the current IK target, so it's visible where the arm
    // is actually being commanded to go.
    gui.drawSphere(arm.targetPosition(), 0.008f, {0.2f, 0.8f, 1.0f});

    ImGui::SetNextWindowPos(ImVec2(20, 20), ImGuiCond_FirstUseEver);
    ImGui::Begin("Robot Arm Telemetry");
    ImGui::Text("Phase: %s", arm.phaseName());
    ImGui::Text("Mission time: %.1f s", missionTime);
    if (arm.phase() == ArmPhase::IDLE)
      ImGui::TextColored(ImVec4(1.0f, 0.8f, 0.2f, 1.0f), "[P] to pick up the cube");

    ImGui::SeparatorText("Joint angles (commanded via IK)");
    plotChannel("Yaw", yawDegHistory, "deg");
    plotChannel("Shoulder", shoulderDegHistory, "deg");
    plotChannel("Elbow", elbowDegHistory, "deg");

    ImGui::SeparatorText("End effector");
    plotChannel("Distance to target", errorHistory, "mm");

    ImGui::SeparatorText("Cube");
    plotChannel("Height above ground", cubeHeightHistory, "m");
    ImGui::End();

    imguiLayer.endFrame();
    gui.endFrame();
  }
  return 0;
}
