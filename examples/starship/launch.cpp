#include <vgl/vgl.h>
#include <rigidbody/PhysicsWorld.h>
#include <rigidbody/Constraint.h>
#include "Booster.h"
#include "Starship.h"
#include <cstdio>
#include <glm/gtc/constants.hpp>

// ---------------------------------------------------------------------------
// Two real-scale cylinders — Super Heavy booster and Starship upper stage,
// stacked and welded together (FixedConstraint) for liftoff, staged by
// removing the weld. Each stage is its own class (Booster / Starship)
// tracking its own propellant; firing its engines depletes that propellant
// based on thrust and Isp, and the body's mass/inertia are kept in sync
// every frame (RigidBody::setMass) — so the stack gets lighter, and thrust
// stops once a stage's tank is empty.
//
// Controls:
//   [Space] booster engines (33, full throttle while held)
//   [E]     ship engines (6, full throttle while held)
//   [1]     stage (remove the weld between booster and ship)
// ---------------------------------------------------------------------------
namespace Config
{
  constexpr float CAMERA_NEAR = 1.0f;
  constexpr float CAMERA_FAR = 20000e3f;
  constexpr float CAMERA_FOV = 45.0f;

  constexpr float GRID_SIZE = 200.0f;
  constexpr float GRID_STEP = 20.0f;
}

// ---------------------------------------------------------------------------
// Draw helpers
// ---------------------------------------------------------------------------
static void drawGrid(GUI &gui)
{
  const glm::vec3 gridColor{0.4f, 0.4f, 0.4f};
  const glm::vec3 axisColorX{0.8f, 0.2f, 0.2f};
  const glm::vec3 axisColorY{0.2f, 0.8f, 0.2f};

  for (float i = -Config::GRID_SIZE; i <= Config::GRID_SIZE; i += Config::GRID_STEP)
  {
    glm::vec3 colorX = (i == 0.0f) ? axisColorX : gridColor;
    glm::vec3 colorY = (i == 0.0f) ? axisColorY : gridColor;

    gui.drawLine({i, -Config::GRID_SIZE, 0}, {i, Config::GRID_SIZE, 0}, colorX);
    gui.drawLine({-Config::GRID_SIZE, i, 0}, {Config::GRID_SIZE, i, 0}, colorY);
  }
}

// Stage body drawn as two cylinder segments split by remaining-propellant
// fraction: bottom `fuelFraction` of the height in `fullColor`, the rest in
// `emptyColor`. RigidBodyShape::CYLINDER's long axis is local Z (see
// RigidBody.cpp), while VGL's drawCylinder defaults to local Y, so this
// passes an explicit {0,0,1} axis to remap — the same pattern falcon-9.cpp
// uses to draw its rocket body.
static void drawFuelCylinder(GUI &gui, RigidBody *stage, float fuelFraction,
                             glm::vec3 fullColor, glm::vec3 emptyColor)
{
  float f = glm::clamp(fuelFraction, 0.0f, 1.0f);
  float H = stage->size.y;
  float R = stage->size.x;
  const glm::vec3 zAxis(0, 0, 1);

  float fullHeight = f * H;
  float emptyHeight = (1.0f - f) * H;
  float fullLocalZ = -H * (1.0f - f) * 0.5f;
  float emptyLocalZ = f * H * 0.5f;

  if (fullHeight > 1e-3f)
    gui.drawCylinder(stage->position + stage->orientation * glm::vec3(0, 0, fullLocalZ),
                     R, fullHeight, zAxis, stage->orientation, fullColor);
  if (emptyHeight > 1e-3f)
    gui.drawCylinder(stage->position + stage->orientation * glm::vec3(0, 0, emptyLocalZ),
                     R, emptyHeight, zAxis, stage->orientation, emptyColor);
}

static void drawEngines(GUI &gui, const std::vector<Thruster *> &engines, RigidBody *stage,
                        bool firing, float radius)
{
  glm::vec3 color = firing ? glm::vec3(1.0f, 0.7f, 0.1f) : glm::vec3(0.3f, 0.3f, 0.3f);
  for (auto *engine : engines)
    gui.drawSphere(engine->getWorldMountPosition(*stage), radius, color);
}

static void updateTitle(GLFWwindow *win, float altitudeM, float boosterFuelPct,
                        float shipFuelPct, bool staged)
{
  char buf[256];
  std::snprintf(buf, sizeof(buf),
                "Starship  |  [Space] Booster  [E] Ship  [1] Stage  |  "
                "Alt: %.0f m  |  Booster fuel: %.0f%%  |  Ship fuel: %.0f%%  |  %s",
                altitudeM, boosterFuelPct, shipFuelPct, staged ? "STAGED" : "STACKED");
  glfwSetWindowTitle(win, buf);
}

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------
int main()
{
  GUI gui(1000, 700, "Starship");
  gui.setLighting(false);
  gui.camera
      .setUp({0, 0, 1})
      .setClipPlanes(Config::CAMERA_NEAR, Config::CAMERA_FAR)
      .setFOV(Config::CAMERA_FOV);

  OrbitalCamera orbit(300.0f, 45.0f, 10.0f, {0, 0, 60.0f});
  orbit.setMinDistance(15.0f)
      .setMaxDistance(50e6f)
      .setZoomSensitivity(5.0f)
      .setPanSensitivity(0.2f);

  glm::vec2 lastMousePos = gui.getMousePosition();

  // =================== DEFINE THE VEHICLE ===================
  PhysicsWorld world;
  world.gravity = {0.0f, 0.0f, -9.81f};

  // Booster stands on the pad, base at z = 0.
  Booster booster(world, glm::vec3(0, 0, Booster::HEIGHT_M * 0.5f));

  // Ship stacked directly on top of the booster.
  Starship ship(world, glm::vec3(0, 0, Booster::HEIGHT_M + Starship::HEIGHT_M * 0.5f));

  glm::vec3 stackJunction(0, 0, Booster::HEIGHT_M);
  FixedConstraint *stackWeld = world.addFixedConstraint(booster.body, ship.body, stackJunction);
  bool staged = false;

  float lastTime = glfwGetTime();
  while (!gui.shouldClose())
  {
    float time = glfwGetTime();
    float dt = time - lastTime;
    lastTime = time;

    // =================== INPUT / FLIGHT SOFTWARE ===================
    float boosterThrottle = gui.isKeyPressed(GLFW_KEY_SPACE) ? 1.0f : 0.0f;
    float shipThrottle = gui.isKeyPressed(GLFW_KEY_E) ? 1.0f : 0.0f;

    if (stackWeld && gui.isKeyJustPressed(GLFW_KEY_1))
    {
      world.removeConstraint(stackWeld);
      stackWeld = nullptr;
      staged = true;
    }

    glm::vec2 mousePos = gui.getMousePosition();
    glm::vec2 mouseDelta = mousePos - lastMousePos;
    lastMousePos = mousePos;
    orbit.handleInput(gui, mouseDelta, gui.getScrollDelta());
    orbit.applyToCamera(gui.camera);

    // Reads each stage's own propellant state, commands its engines, and
    // depletes propellant by the resulting mass flow.
    booster.update(boosterThrottle, dt);
    ship.update(shipThrottle, dt);

    // =================== PHYSICS ===================
    world.step(dt);

    // =================== TELEMETRY ===================
    float altitude = booster.body->position.z - Booster::HEIGHT_M * 0.5f;
    updateTitle(gui.getWindow(), altitude,
               booster.propellantFraction() * 100.0f,
               ship.propellantFraction() * 100.0f,
               staged);

    // =================== DRAW ===================
    gui.beginFrame();
    drawGrid(gui);

    drawFuelCylinder(gui, booster.body, booster.propellantFraction(),
                     {0.2f, 0.8f, 0.3f}, {0.5f, 0.5f, 0.5f});
    drawFuelCylinder(gui, ship.body, ship.propellantFraction(),
                     {0.2f, 0.8f, 0.3f}, {0.75f, 0.75f, 0.75f});

    bool boosterFiring = boosterThrottle > 0.0f && booster.propellantMassKg > 0.0f;
    bool shipFiring = shipThrottle > 0.0f && ship.propellantMassKg > 0.0f;
    drawEngines(gui, booster.centerEngines, booster.body, boosterFiring, Booster::ENGINE_DIAMETER_M * 0.5f);
    drawEngines(gui, booster.outerEngines, booster.body, boosterFiring, Booster::ENGINE_DIAMETER_M * 0.5f);
    drawEngines(gui, ship.centerEngines, ship.body, shipFiring, Starship::ENGINE_DIAMETER_M * 0.5f);
    drawEngines(gui, ship.outerEngines, ship.body, shipFiring, Starship::ENGINE_DIAMETER_M * 0.5f);

    gui.endFrame();
  }
  return 0;
}
