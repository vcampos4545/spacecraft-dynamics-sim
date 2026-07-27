#include <vgl/vgl.h>
#include <rigidbody/PhysicsWorld.h>
#include <rigidbody/environment/Gravity.h>
#include "Starship.h"
#include "LandingSoftware.h"
#include "common/World.h"
#include "common/ShipModel.h"
#include "common/Telemetry.h"
#include "common/ImGuiLayer.h"
#include <cstdio>
#include <cmath>
#include <memory>
#include <glm/gtc/constants.hpp>
#include <glm/gtx/quaternion.hpp>

// ---------------------------------------------------------------------------
// Starship landing: drop a single Starship upper stage from altitude, tilted
// a few degrees off vertical, and let LandingSoftware.h try to bring it down
// to a soft, upright landing using only its own engines -- no aerodynamic
// control surfaces, just the low-altitude powered-descent-guidance problem
// (see LandingSoftware.h for what that guidance law actually is and how it
// differs from SpaceX's real convex-optimization-based approach).
//
// Controls:
//   [D] drop -- releases the hold and starts the descent; everything after
//       that (throttle, gimbal, touchdown detection) is autonomous
// ---------------------------------------------------------------------------
namespace Config
{
  constexpr float CAMERA_NEAR = 0.5f;
  constexpr float CAMERA_FAR = 5000.0f;
  constexpr float CAMERA_FOV = 45.0f;

  constexpr float GRID_SIZE = 100.0f;
  constexpr float GRID_STEP = 10.0f;

  constexpr int TELEMETRY_HISTORY_SAMPLES = 300;

  // Start pose: base this many meters above the pad, tilted off vertical.
  // The tilt alone (no initial lateral offset) is enough to force a real
  // lateral-correction problem: once engines light while still tilted, a
  // transient sideways thrust component kicks the ship off-target before
  // the attitude loop rights it, and the lateral guidance loop has to walk
  // it back to the target point during the rest of the descent.
  constexpr float START_ALTITUDE_M = 600.0f;
  constexpr float START_TILT_DEG = 30.0f;

  // A landing configuration is nearly dry -- a full 1600t tank would make
  // the vehicle unrealistically heavy and sluggish for a short burn. 10%
  // of capacity gives a comfortable but not absurd thrust-to-weight ratio
  // with the 3 center engines alone (~2.7:1 at this mass).
  constexpr float LANDING_PROPELLANT_KG = 0.10f * 1600.0f * 1000.0f;
}

// ---------------------------------------------------------------------------
// Draw helpers
// ---------------------------------------------------------------------------
static void drawEngines(GUI &gui, const std::vector<Thruster *> &engines, RigidBody *stage,
                        bool firing, float radius)
{
  glm::vec3 color = firing ? glm::vec3(1.0f, 0.7f, 0.1f) : glm::vec3(0.3f, 0.3f, 0.3f);
  for (auto *engine : engines)
    gui.drawSphere(engine->getWorldMountPosition(*stage), radius, color);
}

// A small flat marker at the target landing point, so the target is
// visible even before the ship has moved.
static void drawLandingPad(GUI &gui, glm::vec2 targetXY)
{
  gui.drawCylinder(glm::vec3(targetXY.x, targetXY.y, 0.05f), 12.0f, 0.1f, {0.7f, 0.15f, 0.15f});
}

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------
int main()
{
  GUI gui(1000, 700, "Starship Landing");
  ImGuiLayer imguiLayer(gui);
  World scene(WorldType::EARTH);
  scene.apply(gui);
  gui.camera
      .setUp({0, 0, 1})
      .setClipPlanes(Config::CAMERA_NEAR, Config::CAMERA_FAR)
      .setFOV(Config::CAMERA_FOV);
  gui.setLogDepth(Config::CAMERA_FAR);

  OrbitalCamera orbit(200.0f, 30.0f, 15.0f, {0, 0, Config::START_ALTITUDE_M * 0.5f});
  orbit.setMinDistance(10.0f)
      .setMaxDistance(3000.0f)
      .setZoomSensitivity(3.0f)
      .setPanSensitivity(0.1f);

  glm::vec2 lastMousePos = gui.getMousePosition();

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

  glm::vec2 targetXY(0.0f, 0.0f);
  glm::vec3 startPosition(targetXY.x, targetXY.y, Config::START_ALTITUDE_M + Starship::HEIGHT_M * 0.5f);

  Starship ship(world, startPosition);
  ship.body->orientation = glm::angleAxis(glm::radians(Config::START_TILT_DEG), glm::vec3(0, 1, 0));

  // Land nearly dry -- see Config::LANDING_PROPELLANT_KG.
  ship.propellantMassKg = Config::LANDING_PROPELLANT_KG;
  ship.body->setMass(ship.dryMassKg + ship.propellantMassKg);

  LandingSoftware landing(ship, targetXY);

  // Sensor-side (state) and actuator-side (commanded) telemetry channels.
  TelemetryChannel altitudeHistory(Config::TELEMETRY_HISTORY_SAMPLES);
  TelemetryChannel verticalSpeedHistory(Config::TELEMETRY_HISTORY_SAMPLES);
  TelemetryChannel throttleHistory(Config::TELEMETRY_HISTORY_SAMPLES);
  TelemetryChannel tiltXHistory(Config::TELEMETRY_HISTORY_SAMPLES);
  TelemetryChannel tiltYHistory(Config::TELEMETRY_HISTORY_SAMPLES);
  TelemetryChannel lateralErrorHistory(Config::TELEMETRY_HISTORY_SAMPLES);
  TrajectoryPath shipTrajectory;

  float lastTime = glfwGetTime();
  while (!gui.shouldClose())
  {
    float time = glfwGetTime();
    float dt = time - lastTime;
    lastTime = time;

    // =================== INPUT / FLIGHT SOFTWARE ===================
    // The only manual input is the drop command; throttle, gimbal, and
    // touchdown detection all run autonomously inside
    // LandingSoftware::update().
    bool dropCommand = gui.isKeyPressed(GLFW_KEY_D);
    landing.update(dt, dropCommand);

    glm::vec2 mousePos = gui.getMousePosition();
    glm::vec2 mouseDelta = mousePos - lastMousePos;
    lastMousePos = mousePos;
    orbit.handleInput(gui, mouseDelta, gui.getScrollDelta());
    orbit.setTarget(ship.body->position);
    orbit.applyToCamera(gui.camera);

    // Only the 3 gimbal-capable center engines are used for landing (see
    // LandingSoftware.h) -- the vacuum-optimized outer ring never fires.
    ship.update(landing.throttle(), 0.0f, dt);

    // =================== PHYSICS ===================
    world.step(dt);

    // =================== TELEMETRY ===================
    glm::vec2 shipXY(ship.body->position.x, ship.body->position.y);
    float lateralError = glm::length(targetXY - shipXY);

    altitudeHistory.push(landing.altitude());
    verticalSpeedHistory.push(ship.body->velocity.z);
    throttleHistory.push(landing.throttle() * 100.0f);
    tiltXHistory.push(landing.targetTiltDeg().x);
    tiltYHistory.push(landing.targetTiltDeg().y);
    lateralErrorHistory.push(lateralError);
    shipTrajectory.push(lateralError, landing.altitude());

    // =================== DRAW ===================
    gui.beginFrame();
    imguiLayer.beginFrame();
    scene.draw(gui, Config::GRID_SIZE, Config::GRID_STEP);

    drawLandingPad(gui, targetXY);
    drawShipModel(gui, shipModel, shipFit, ship.body);

    bool firing = landing.throttle() > 0.0f && ship.propellantMassKg > 0.0f;
    drawEngines(gui, ship.centerEngines, ship.body, firing, Starship::ENGINE_DIAMETER_M * 0.5f);
    drawEngines(gui, ship.outerEngines, ship.body, false, Starship::ENGINE_DIAMETER_M * 0.5f);

    ImGui::Begin("Landing Telemetry");
    ImGui::Text("Mission phase: %s", landing.phaseName());
    ImGui::Text("Mission time (T+): %.1f s", landing.missionTime());
    if (landing.phase() == LandingPhase::HOLDING)
      ImGui::TextColored(ImVec4(1.0f, 0.8f, 0.2f, 1.0f), "[D] to drop");
    else if (landing.phase() == LandingPhase::LANDED)
      ImGui::TextColored(ImVec4(0.3f, 1.0f, 0.4f, 1.0f), "Touchdown -- landed safely");
    else if (landing.phase() == LandingPhase::CRASHED)
      ImGui::TextColored(ImVec4(1.0f, 0.3f, 0.3f, 1.0f), "Touchdown -- too hard or too tilted");

    ImGui::SeparatorText("Sensors (state estimate)");
    plotChannel("Altitude", altitudeHistory, "m");
    plotChannel("Vertical speed", verticalSpeedHistory, "m/s");
    plotChannel("Distance to target", lateralErrorHistory, "m");

    ImGui::SeparatorText("Guidance / Control");
    plotChannel("Tilt toward +X", tiltXHistory, "deg");
    plotChannel("Tilt toward +Y", tiltYHistory, "deg");

    ImGui::SeparatorText("Actuators (commanded throttle)");
    plotChannel("Center engines throttle", throttleHistory, "%");

    ImGui::SeparatorText("Trajectory");
    drawTrajectoryPlot("Altitude vs distance to target", {
                                                             {&shipTrajectory, "Ship", IM_COL32(80, 200, 255, 255)},
                                                         });
    ImGui::End();

    imguiLayer.endFrame();
    gui.endFrame();
  }
  return 0;
}
