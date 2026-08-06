#include <vgl/vgl.h>
#include <rigidbody/PhysicsWorld.h>
#include <rigidbody/Constraint.h>
#include "common/World.h"
#include <cstdio>
#include <glm/gtc/constants.hpp>

// ---------------------------------------------------------------------------
// Solar panel deployment.
//
// A bus with two panels hinged to opposite faces (+Y and -Y). Each hinge is
// a HingeConstraint with an angle limit (0 = stowed flat against the bus,
// pi/2 = fully deployed, sticking straight out) and a motor that drives it
// from one to the other. Press 1 / 2 to deploy each panel independently —
// there's no shared "spacecraft" object here, just two RigidBodies and two
// HingeConstraints that scenario code (this file) commands directly.
// ---------------------------------------------------------------------------
namespace Config
{
  constexpr float CAMERA_NEAR = 0.05f;
  constexpr float CAMERA_FAR = 100.0f;
  constexpr float CAMERA_FOV = 45.0f;
  constexpr float CAMERA_INITIAL_DISTANCE = 2.5f;
  constexpr float CAMERA_MIN_DISTANCE = 0.5f;
  constexpr float CAMERA_MAX_DISTANCE = 20.0f;
  constexpr float ZOOM_SENSITIVITY = 1.0f;
  constexpr float PAN_SENSITIVITY = 0.2f;

  constexpr float GRID_SIZE = 5.0f;
  constexpr float GRID_STEP = 1.0f;

  const glm::vec3 BUS_SIZE{0.3f, 0.3f, 0.4f};
  constexpr float BUS_MASS = 4.0f;

  const glm::vec3 PANEL_SIZE{0.28f, 0.01f, 0.5f}; // width x thickness x length
  constexpr float PANEL_MASS = 0.2f;

  constexpr float MOTOR_TARGET_SPEED = 0.8f; // rad/s
  constexpr float MOTOR_MAX_TORQUE = 0.05f;  // Nm

  constexpr float START_Z = 3.0f; // clear of the ground plane at z = 0
}

// ---------------------------------------------------------------------------
// A panel hinged to one face of the bus, stowed flat (angle = 0) and
// deployable out to fully extended (angle = pi/2).
// ---------------------------------------------------------------------------
struct Panel
{
  RigidBody *body = nullptr;
  HingeConstraint *hinge = nullptr;
  glm::vec3 localPivotOffset; // pivot's position relative to the bus, in the bus's local frame
};

// side: +1 for the +Y face (deploys toward +Y), -1 for the -Y face (deploys toward -Y)
static Panel attachPanel(PhysicsWorld &world, RigidBody *bus, float side)
{
  Panel panel;
  panel.localPivotOffset = glm::vec3(0, side * Config::BUS_SIZE.y * 0.5f, 0);
  glm::vec3 pivot = bus->position + panel.localPivotOffset;

  panel.body = world.createBody(RigidBodyShape::BOX, Config::PANEL_SIZE, Config::PANEL_MASS);
  panel.body->position = pivot + glm::vec3(0, 0, Config::PANEL_SIZE.z * 0.5f); // stowed: extends +Z alongside the bus

  // Deploying toward +Y needs axis -X; deploying toward -Y needs axis +X
  // (rotating the panel's stowed +Z arm by +90 degrees about that axis
  // sweeps it out to +/-Y — see the derivation in the accompanying PR).
  glm::vec3 axis(-side, 0, 0);
  panel.hinge = world.addHingeConstraint(bus, panel.body, pivot, axis);
  panel.hinge->setLimits(0.0f, glm::half_pi<float>());
  panel.hinge->setMotorMaxTorque(Config::MOTOR_MAX_TORQUE);

  return panel;
}

static void deploy(Panel &panel)
{
  panel.hinge->enableMotor(true);
  panel.hinge->setMotorTargetSpeed(Config::MOTOR_TARGET_SPEED);
}

// ---------------------------------------------------------------------------
// Draw helpers
// ---------------------------------------------------------------------------
static void drawPanel(GUI &gui, const Panel &panel, glm::vec3 color)
{
  gui.drawBox(panel.body->position, panel.body->size, panel.body->orientation, color);
}

static void updateTitle(GLFWwindow *win, const Panel &panel1, const Panel &panel2)
{
  char buf[256];
  std::snprintf(buf, sizeof(buf),
                "Solar Panel Deployment  |  [1] Panel 1: %.0f deg  |  [2] Panel 2: %.0f deg",
                glm::degrees(panel1.hinge->getAngle()),
                glm::degrees(panel2.hinge->getAngle()));
  glfwSetWindowTitle(win, buf);
}

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------
int main()
{
  GUI gui(800, 600, "Solar Panel Deployment");
  World scene(WorldType::DEFAULT);
  scene.apply(gui);
  gui.camera
      .setUp({0, 0, 1})
      .setClipPlanes(Config::CAMERA_NEAR, Config::CAMERA_FAR)
      .setFOV(Config::CAMERA_FOV);

  OrbitalCamera orbit(
      Config::CAMERA_INITIAL_DISTANCE,
      45.0f, 20.0f,
      {0, 0, Config::START_Z});
  orbit.setMaxDistance(Config::CAMERA_MAX_DISTANCE)
      .setMinDistance(Config::CAMERA_MIN_DISTANCE)
      .setZoomSensitivity(Config::ZOOM_SENSITIVITY)
      .setPanSensitivity(Config::PAN_SENSITIVITY);

  glm::vec2 lastMousePos = gui.getMousePosition();

  // =================== DEFINE THE SPACECRAFT ===================
  // Free-floating: no gravity generator is attached, so there's no gravity
  // at all (the world starts with none by default).
  PhysicsWorld world;

  RigidBody *bus = world.createBody(RigidBodyShape::BOX, Config::BUS_SIZE, Config::BUS_MASS);
  bus->position.z = Config::START_Z;

  Panel panel1 = attachPanel(world, bus, +1.0f); // +Y face
  Panel panel2 = attachPanel(world, bus, -1.0f); // -Y face

  float lastTime = glfwGetTime();
  while (!gui.shouldClose())
  {
    float time = glfwGetTime();
    float dt   = time - lastTime;
    lastTime   = time;

    // =================== INPUT / FLIGHT SOFTWARE ===================
    // Each panel's hinge is commanded directly and independently.
    if (gui.isKeyJustPressed(GLFW_KEY_1)) deploy(panel1);
    if (gui.isKeyJustPressed(GLFW_KEY_2)) deploy(panel2);

    glm::vec2 mousePos   = gui.getMousePosition();
    glm::vec2 mouseDelta = mousePos - lastMousePos;
    lastMousePos         = mousePos;
    orbit.handleInput(gui, mouseDelta, gui.getScrollDelta());
    orbit.applyToCamera(gui.camera);

    // =================== PHYSICS ===================
    world.step(dt);

    // =================== TELEMETRY ===================
    updateTitle(gui.getWindow(), panel1, panel2);

    // =================== DRAW ===================
    gui.beginFrame();
    scene.draw(gui, Config::GRID_SIZE, Config::GRID_STEP);

    gui.drawBox(bus->position, bus->size, bus->orientation, {0.6f, 0.6f, 0.65f});
    drawPanel(gui, panel1, {0.2f, 0.4f, 0.9f});
    drawPanel(gui, panel2, {0.9f, 0.5f, 0.2f});

    // Hinge pivot markers, transformed by the bus's current orientation
    gui.drawSphere(bus->position + bus->orientation * panel1.localPivotOffset, 0.015f, {1, 1, 1});
    gui.drawSphere(bus->position + bus->orientation * panel2.localPivotOffset, 0.015f, {1, 1, 1});

    gui.endFrame();
  }
  return 0;
}
