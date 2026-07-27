#include <vgl/vgl.h>
#include <rigidbody/PhysicsWorld.h>
#include <rigidbody/Constraint.h>
#include <rigidbody/environment/Gravity.h>
#include "Booster.h"
#include "Starship.h"
#include "FlightSoftware.h"
#include "common/World.h"
#include "common/ShipModel.h"
#include "common/Telemetry.h"
#include "common/ImGuiLayer.h"
#include <cstdio>
#include <cmath>
#include <algorithm>
#include <memory>
#include <vector>
#include <glm/gtc/constants.hpp>
#include <glm/gtx/quaternion.hpp>

// ---------------------------------------------------------------------------
// Super Heavy booster (real-scale cylinder) and Starship upper stage (real
// -scale 3D model), stacked and welded together (FixedConstraint) for
// liftoff, staged by removing the weld. Each stage is its own class
// (Booster / Starship) tracking its own propellant; firing its engines
// depletes that propellant based on thrust and Isp, and the body's
// mass/inertia are kept in sync every frame (RigidBody::setMass) — so the
// stack gets lighter, and thrust stops once a stage's tank is empty.
//
// FlightSoftware.h runs the ascent + staging autonomously (guidance,
// attitude control, and the MECO/separation/ignition/SECO sequencer) once
// launch is committed -- see that file for how it mirrors a real ascent GNC
// stack. This file just wires it to the vehicle, drives rendering, and
// reads the one manual input that starts the mission.
//
// Controls:
//   [L] commit to launch (T-0) -- everything after that is autonomous
// ---------------------------------------------------------------------------
namespace Config
{
  constexpr float CAMERA_NEAR = 1.0f;
  constexpr float CAMERA_FAR = 20000e3f;
  constexpr float CAMERA_FOV = 45.0f;

  constexpr float GRID_SIZE = 200.0f;
  constexpr float GRID_STEP = 20.0f;

  constexpr int TELEMETRY_HISTORY_SAMPLES = 300;
}

// ---------------------------------------------------------------------------
// Draw helpers
// ---------------------------------------------------------------------------

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

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------
int main()
{
  GUI gui(1000, 700, "Starship");
  ImGuiLayer imguiLayer(gui);
  World scene(WorldType::EARTH);
  scene.apply(gui);
  gui.camera
      .setUp({0, 0, 1})
      .setClipPlanes(Config::CAMERA_NEAR, Config::CAMERA_FAR)
      .setFOV(Config::CAMERA_FOV);
  gui.setLogDepth(Config::CAMERA_FAR); // matches the far clip plane -- standard depth precision falls apart at this range

  OrbitalCamera orbit(300.0f, 45.0f, 10.0f, {0, 0, 60.0f});
  orbit.setMinDistance(15.0f)
      .setMaxDistance(50e6f)
      .setZoomSensitivity(5.0f)
      .setPanSensitivity(0.2f);

  glm::vec2 lastMousePos = gui.getMousePosition();

  // Ship is rendered from a real model; loaded after the GUI/GL context
  // exists (mesh upload needs a bound context). The booster doesn't have
  // one yet, so it still draws as a fuel-colored cylinder.
  OBJMesh shipModel;
  if (!shipModel.load("models/starship.obj"))
  {
    std::fprintf(stderr, "Failed to load models/starship.obj: %s\n", shipModel.getError().c_str());
    return 1;
  }
  ModelFit shipFit = fitModelToCylinder(shipModel, Starship::HEIGHT_M, Starship::DIAMETER_M);

  // =================== DEFINE THE VEHICLE ===================
  PhysicsWorld world;
  world.addGlobalForceGenerator(std::make_unique<Gravity>(glm::vec3(0.0f, 0.0f, -9.81f)));

  // Booster stands on the pad, base at z = 0.
  Booster booster(world, glm::vec3(0, 0, Booster::HEIGHT_M * 0.5f));

  // Ship stacked directly on top of the booster.
  Starship ship(world, glm::vec3(0, 0, Booster::HEIGHT_M + Starship::HEIGHT_M * 0.5f));

  glm::vec3 stackJunction(0, 0, Booster::HEIGHT_M);
  FixedConstraint *stackWeld = world.addFixedConstraint(booster.body, ship.body, stackJunction);

  FlightSoftware fsw(world, booster, ship, stackWeld);

  // Sensor-side (state) and actuator-side (commanded) telemetry channels.
  TelemetryChannel altitudeHistory(Config::TELEMETRY_HISTORY_SAMPLES);
  TelemetryChannel speedHistory(Config::TELEMETRY_HISTORY_SAMPLES);
  TelemetryChannel boosterFuelHistory(Config::TELEMETRY_HISTORY_SAMPLES);
  TelemetryChannel shipFuelHistory(Config::TELEMETRY_HISTORY_SAMPLES);
  TelemetryChannel boosterThrottleHistory(Config::TELEMETRY_HISTORY_SAMPLES);
  TelemetryChannel shipThrottleHistory(Config::TELEMETRY_HISTORY_SAMPLES);
  TelemetryChannel targetPitchHistory(Config::TELEMETRY_HISTORY_SAMPLES);
  TrajectoryPath boosterTrajectory;
  TrajectoryPath shipTrajectory;

  float lastTime = glfwGetTime();
  while (!gui.shouldClose())
  {
    float time = glfwGetTime();
    float dt = time - lastTime;
    lastTime = time;

    // =================== INPUT / FLIGHT SOFTWARE ===================
    // The only manual input is the launch commit; everything from LIFTOFF
    // through SECO -- pitch program, gimbal control, MECO, staging, ship
    // ignition -- runs autonomously inside FlightSoftware::update().
    bool launchCommit = gui.isKeyPressed(GLFW_KEY_L);
    fsw.update(dt, launchCommit);

    glm::vec2 mousePos = gui.getMousePosition();
    glm::vec2 mouseDelta = mousePos - lastMousePos;
    lastMousePos = mousePos;
    orbit.handleInput(gui, mouseDelta, gui.getScrollDelta());
    orbit.setTarget(ship.body->position); // follow the last stage (ship continues after staging; booster falls away)
    orbit.applyToCamera(gui.camera);

    // Reads each stage's own propellant state, commands its engines, and
    // depletes propellant by the resulting mass flow.
    booster.update(fsw.boosterThrottle(), dt);
    ship.update(fsw.shipThrottle(), fsw.shipThrottle(), dt); // ascent fires every engine together

    // =================== PHYSICS ===================
    world.step(dt);

    // =================== TELEMETRY ===================
    float altitude = booster.body->position.z - Booster::HEIGHT_M * 0.5f;
    altitudeHistory.push(altitude);
    speedHistory.push(glm::length(ship.body->velocity));
    boosterFuelHistory.push(booster.propellantFraction() * 100.0f);
    shipFuelHistory.push(ship.propellantFraction() * 100.0f);
    boosterThrottleHistory.push(fsw.boosterThrottle() * 100.0f);
    shipThrottleHistory.push(fsw.shipThrottle() * 100.0f);
    targetPitchHistory.push(fsw.targetPitchDeg());

    boosterTrajectory.push(
        std::sqrt(booster.body->position.x * booster.body->position.x +
                 booster.body->position.y * booster.body->position.y),
        booster.body->position.z);
    shipTrajectory.push(
        std::sqrt(ship.body->position.x * ship.body->position.x +
                 ship.body->position.y * ship.body->position.y),
        ship.body->position.z);

    // =================== DRAW ===================
    gui.beginFrame();
    imguiLayer.beginFrame();
    scene.draw(gui, Config::GRID_SIZE, Config::GRID_STEP);

    drawFuelCylinder(gui, booster.body, booster.propellantFraction(),
                     {0.2f, 0.8f, 0.3f}, {0.5f, 0.5f, 0.5f});
    drawShipModel(gui, shipModel, shipFit, ship.body);

    bool boosterFiring = fsw.boosterThrottle() > 0.0f && booster.propellantMassKg > 0.0f;
    bool shipFiring = fsw.shipThrottle() > 0.0f && ship.propellantMassKg > 0.0f;
    drawEngines(gui, booster.centerEngines, booster.body, boosterFiring, Booster::ENGINE_DIAMETER_M * 0.5f);
    drawEngines(gui, booster.outerEngines, booster.body, boosterFiring, Booster::ENGINE_DIAMETER_M * 0.5f);
    drawEngines(gui, ship.centerEngines, ship.body, shipFiring, Starship::ENGINE_DIAMETER_M * 0.5f);
    drawEngines(gui, ship.outerEngines, ship.body, shipFiring, Starship::ENGINE_DIAMETER_M * 0.5f);

    ImGui::Begin("Starship Telemetry");
    ImGui::Text("Mission phase: %s", fsw.phaseName());
    ImGui::Text("Mission time (T+): %.1f s", fsw.missionTime());
    ImGui::Text("Stage state: %s", fsw.staged() ? "STAGED" : "STACKED");
    if (fsw.phase() == MissionPhase::PRELAUNCH)
      ImGui::TextColored(ImVec4(1.0f, 0.8f, 0.2f, 1.0f), "[L] to commit to launch");

    ImGui::SeparatorText("Sensors (state estimate)");
    plotChannel("Altitude", altitudeHistory, "m");
    plotChannel("Speed", speedHistory, "m/s");

    ImGui::SeparatorText("Guidance / Control");
    plotChannel("Target pitch", targetPitchHistory, "deg");

    ImGui::SeparatorText("Actuators (commanded throttle)");
    plotChannel("Booster throttle", boosterThrottleHistory, "%");
    plotChannel("Ship throttle", shipThrottleHistory, "%");

    ImGui::SeparatorText("Propellant");
    plotChannel("Booster fuel", boosterFuelHistory, "%");
    plotChannel("Ship fuel", shipFuelHistory, "%");

    ImGui::SeparatorText("Trajectory");
    drawTrajectoryPlot("Altitude vs downrange", {
      {&boosterTrajectory, "Booster", IM_COL32(255, 140, 60, 255)},
      {&shipTrajectory, "Ship", IM_COL32(80, 200, 255, 255)},
    });
    ImGui::End();

    imguiLayer.endFrame();
    gui.endFrame();
  }
  return 0;
}
