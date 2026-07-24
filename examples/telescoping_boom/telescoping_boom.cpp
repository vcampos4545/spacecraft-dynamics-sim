#include <vgl/vgl.h>
#include <rigidbody/PhysicsWorld.h>
#include <rigidbody/Constraint.h>
#include "common/World.h"
#include <cstdio>
#include <glm/gtc/constants.hpp>

// ---------------------------------------------------------------------------
// Telescoping antenna boom: a SliderConstraint between a bus and a boom
// segment, sliding along a fixed axis with a hard travel limit. Unlike
// solar_panel_deploy's one-shot "deploy and latch," this one is driven
// continuously by the operator — hold UP to extend, DOWN to retract — to
// show the motor working in both directions instead of just driving to a
// limit and stopping.
// ---------------------------------------------------------------------------
namespace Config
{
  constexpr float CAMERA_NEAR = 0.05f;
  constexpr float CAMERA_FAR = 100.0f;
  constexpr float CAMERA_FOV = 45.0f;
  constexpr float CAMERA_INITIAL_DISTANCE = 3.0f;
  constexpr float CAMERA_MIN_DISTANCE = 0.5f;
  constexpr float CAMERA_MAX_DISTANCE = 20.0f;
  constexpr float ZOOM_SENSITIVITY = 1.0f;
  constexpr float PAN_SENSITIVITY = 0.2f;

  constexpr float GRID_SIZE = 5.0f;
  constexpr float GRID_STEP = 1.0f;

  const glm::vec3 BUS_SIZE{0.3f, 0.3f, 0.3f};
  constexpr float BUS_MASS = 4.0f;

  const glm::vec3 BOOM_SIZE{0.04f, 0.04f, 0.8f};
  constexpr float BOOM_MASS = 0.15f;

  constexpr float MAX_EXTENSION = 1.2f;    // meters of travel
  constexpr float MOTOR_SPEED = 0.4f;      // m/s
  constexpr float MOTOR_MAX_FORCE = 8.0f;  // N

  constexpr float START_Z = 3.0f; // clear of the ground plane at z = 0
}

static void updateTitle(GLFWwindow *win, float extensionM)
{
  char buf[256];
  std::snprintf(buf, sizeof(buf),
                "Telescoping Boom  |  [Up] Extend  [Down] Retract  |  Extension: %.2f m",
                extensionM);
  glfwSetWindowTitle(win, buf);
}

int main()
{
  GUI gui(800, 600, "Telescoping Boom (SliderConstraint)");
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
  // Free-floating: no Gravity generator is attached, so there's no gravity
  // at all (the world starts with none by default).
  PhysicsWorld world;

  RigidBody *bus = world.createBody(RigidBodyShape::BOX, Config::BUS_SIZE, Config::BUS_MASS);
  bus->position.z = Config::START_Z;

  const glm::vec3 localPivotOffset(0, 0, Config::BUS_SIZE.z * 0.5f);
  glm::vec3 pivot = bus->position + localPivotOffset;
  RigidBody *boom = world.createBody(RigidBodyShape::BOX, Config::BOOM_SIZE, Config::BOOM_MASS);
  boom->position = pivot + glm::vec3(0, 0, Config::BOOM_SIZE.z * 0.5f); // fully retracted

  SliderConstraint *slider = world.addSliderConstraint(bus, boom, pivot, glm::vec3(0, 0, 1));
  slider->setLimits(0.0f, Config::MAX_EXTENSION);
  slider->enableMotor(true);
  slider->setMotorMaxForce(Config::MOTOR_MAX_FORCE);

  float lastTime = glfwGetTime();
  while (!gui.shouldClose())
  {
    float time = glfwGetTime();
    float dt   = time - lastTime;
    lastTime   = time;

    // =================== FLIGHT SOFTWARE ===================
    // Continuous operator control: the motor always runs, targeting +/-speed
    // while a key is held and 0 (a soft brake) otherwise.
    float targetSpeed = 0.0f;
    if (gui.isKeyPressed(GLFW_KEY_UP))   targetSpeed += Config::MOTOR_SPEED;
    if (gui.isKeyPressed(GLFW_KEY_DOWN)) targetSpeed -= Config::MOTOR_SPEED;
    slider->setMotorTargetSpeed(targetSpeed);

    glm::vec2 mousePos   = gui.getMousePosition();
    glm::vec2 mouseDelta = mousePos - lastMousePos;
    lastMousePos         = mousePos;
    orbit.handleInput(gui, mouseDelta, gui.getScrollDelta());
    orbit.applyToCamera(gui.camera);

    // =================== PHYSICS ===================
    world.step(dt);

    // =================== TELEMETRY ===================
    updateTitle(gui.getWindow(), slider->getPosition());

    // =================== DRAW ===================
    gui.beginFrame();
    scene.draw(gui, Config::GRID_SIZE, Config::GRID_STEP);

    gui.drawBox(bus->position, bus->size, bus->orientation, {0.6f, 0.6f, 0.65f});
    gui.drawBox(boom->position, boom->size, boom->orientation, {0.3f, 0.8f, 0.4f});
    gui.drawSphere(bus->position + bus->orientation * localPivotOffset, 0.02f, {1, 1, 1});

    gui.endFrame();
  }
  return 0;
}
