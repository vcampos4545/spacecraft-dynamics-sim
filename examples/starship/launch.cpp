#include <vgl/vgl.h>
#include <rigidbody/PhysicsWorld.h>
#include <rigidbody/Constraint.h>
#include <rigidbody/environment/Gravity.h>
#include "Booster.h"
#include "Starship.h"
#include "FlightSoftware.h"
#include "common/World.h"
#include <cstdio>
#include <cfloat>
#include <cmath>
#include <algorithm>
#include <memory>
#include <vector>
#include <glm/gtc/constants.hpp>
#include <glm/gtx/quaternion.hpp>
#include <imgui.h>

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

// Fits a loaded OBJ model's bounding box onto a cylinder of the given
// height and diameter, without needing to know the model's authored units
// or pivot in advance:
//  - the mesh's longest bounding-box axis is treated as its nose/base axis
//    and mapped onto height (matches Starship.cpp's local +Z = nose; for
//    models/starship.obj the nose sits at the min end of that axis, engines
//    /legs at the max end);
//  - of the remaining two axes, the SHORTER one is treated as the true
//    body diameter (the taller one is inflated by non-radially-symmetric
//    geometry, e.g. fold-out landing legs) and both are scaled by the same
//    factor so the cross-section stays circular instead of being squashed
//    into an ellipse;
//  - the bounding-box centroid is recentered onto the body's origin, since
//    RigidBody::position is the cylinder's center but this model's own
//    origin sits at its base.
struct ModelFit
{
  glm::vec3 scale{0.0f};
  glm::quat align{1.0f, 0.0f, 0.0f, 0.0f}; // mesh-local axes -> body-local axes
  glm::vec3 pivotOffsetLocal{0.0f};        // mesh-local bounding-box centroid
};

static ModelFit fitModelToCylinder(const OBJMesh &mesh, float heightM, float diameterM)
{
  glm::vec3 lo = mesh.getBoundsMin();
  glm::vec3 hi = mesh.getBoundsMax();
  glm::vec3 extent = hi - lo;

  int up = 0;
  if (extent.y > extent[up]) up = 1;
  if (extent.z > extent[up]) up = 2;
  int a = (up + 1) % 3;
  int b = (up + 2) % 3;

  ModelFit fit;
  fit.scale[up] = heightM / extent[up];
  float crossScale = diameterM / std::min(extent[a], extent[b]);
  fit.scale[a] = crossScale;
  fit.scale[b] = crossScale;

  // The longest axis is reliably the nose/base axis, but which *end* is
  // the nose isn't derivable from the bounding box alone -- confirmed by
  // running it that models/starship.obj has its nose at the min end (flip
  // this sign if a differently-authored model points the other way).
  glm::vec3 meshUp(0.0f);
  meshUp[up] = -1.0f;
  fit.align = glm::rotation(meshUp, glm::vec3(0, 0, 1));

  fit.pivotOffsetLocal = 0.5f * (lo + hi);
  return fit;
}

static void drawShipModel(GUI &gui, OBJMesh &mesh, const ModelFit &fit, RigidBody *stage)
{
  glm::quat rotation = stage->orientation * fit.align;
  glm::vec3 centroidOffset = rotation * (fit.scale * fit.pivotOffsetLocal);
  gui.drawOBJMesh(mesh, stage->position - centroidOffset, fit.scale, rotation);
}

static void drawEngines(GUI &gui, const std::vector<Thruster *> &engines, RigidBody *stage,
                        bool firing, float radius)
{
  glm::vec3 color = firing ? glm::vec3(1.0f, 0.7f, 0.1f) : glm::vec3(0.3f, 0.3f, 0.3f);
  for (auto *engine : engines)
    gui.drawSphere(engine->getWorldMountPosition(*stage), radius, color);
}

// ---------------------------------------------------------------------------
// Telemetry -- fixed-length rolling history per channel, drawn as an
// ImGui::PlotLines graph in a telemetry window instead of window-title text.
// ---------------------------------------------------------------------------
struct TelemetryChannel
{
  std::vector<float> samples;
  size_t capacity;

  explicit TelemetryChannel(size_t cap) : capacity(cap) { samples.reserve(cap); }

  void push(float value)
  {
    if (samples.size() >= capacity)
      samples.erase(samples.begin());
    samples.push_back(value);
  }

  float last() const { return samples.empty() ? 0.0f : samples.back(); }
};

static void plotChannel(const char *label, const TelemetryChannel &ch, const char *unit)
{
  char overlay[64];
  std::snprintf(overlay, sizeof(overlay), "%.1f %s", ch.last(), unit);
  ImGui::PlotLines(label, ch.samples.data(), (int)ch.samples.size(), 0,
                   overlay, FLT_MAX, FLT_MAX, ImVec2(0, 60));
}

// Ascent trajectory: downrange distance (x) vs altitude (y), unlike the
// TelemetryChannel plots above which are scalar-vs-time. ImGui's core
// PlotLines only plots a single series against an implicit time/index axis,
// so an actual XY trajectory has to be drawn by hand onto the window's draw
// list. Unbounded (not a rolling window like TelemetryChannel) since the
// point is to see the whole flown path so far, not just the last few
// seconds -- a ~10 minute mission at 60fps is at most a few tens of
// thousands of points, cheap to store and draw.
struct TrajectoryPath
{
  std::vector<ImVec2> points; // x = downrange (m), y = altitude (m)

  void push(float downrangeM, float altitudeM)
  {
    points.push_back(ImVec2(downrangeM, altitudeM));
  }
};

static void drawTrajectoryPlot(const char *label,
                               const TrajectoryPath &boosterPath, ImU32 boosterColor,
                               const TrajectoryPath &shipPath, ImU32 shipColor)
{
  ImGui::Text("%s", label);
  ImGui::TextColored(ImColor(boosterColor), "-- Booster");
  ImGui::SameLine();
  ImGui::TextColored(ImColor(shipColor), "-- Ship");

  ImVec2 canvasPos = ImGui::GetCursorScreenPos();
  ImVec2 canvasSize(ImGui::GetContentRegionAvail().x, 180.0f);
  ImGui::InvisibleButton(label, canvasSize);

  ImVec2 rectMin = canvasPos;
  ImVec2 rectMax = ImVec2(canvasPos.x + canvasSize.x, canvasPos.y + canvasSize.y);
  ImDrawList *drawList = ImGui::GetWindowDrawList();
  drawList->AddRectFilled(rectMin, rectMax, IM_COL32(20, 20, 25, 255));
  drawList->AddRect(rectMin, rectMax, IM_COL32(90, 90, 100, 255));

  // Auto-scaled axes, fit to whichever path currently has the larger range.
  // Always include the origin (the pad) so the plot reads sensibly before
  // much data has accumulated.
  float minX = 0.0f, maxX = 0.0f, minY = 0.0f, maxY = 0.0f;
  auto expand = [&](const TrajectoryPath &p)
  {
    for (const ImVec2 &pt : p.points)
    {
      maxX = std::max(maxX, pt.x);
      maxY = std::max(maxY, pt.y);
    }
  };
  expand(boosterPath);
  expand(shipPath);
  float rangeX = std::max(maxX - minX, 1.0f);
  float rangeY = std::max(maxY - minY, 1.0f);

  auto toScreen = [&](const ImVec2 &pt)
  {
    float u = (pt.x - minX) / rangeX;
    float v = (pt.y - minY) / rangeY;
    return ImVec2(rectMin.x + u * canvasSize.x, rectMax.y - v * canvasSize.y);
  };

  auto drawPath = [&](const TrajectoryPath &p, ImU32 color)
  {
    if (p.points.size() < 2)
      return;
    std::vector<ImVec2> screenPoints;
    screenPoints.reserve(p.points.size());
    for (const ImVec2 &pt : p.points)
      screenPoints.push_back(toScreen(pt));
    drawList->AddPolyline(screenPoints.data(), (int)screenPoints.size(), color, 0, 2.0f);
  };
  drawPath(boosterPath, boosterColor);
  drawPath(shipPath, shipColor);

  char axisLabel[96];
  std::snprintf(axisLabel, sizeof(axisLabel), "downrange 0-%.0f m, altitude 0-%.0f m", maxX, maxY);
  drawList->AddText(ImVec2(rectMin.x + 4, rectMin.y + 4), IM_COL32(210, 210, 220, 255), axisLabel);
}

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------
int main()
{
  GUI gui(1000, 700, "Starship");
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
    ship.update(fsw.shipThrottle(), dt);

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
    drawTrajectoryPlot("Altitude vs downrange", boosterTrajectory, IM_COL32(255, 140, 60, 255),
                       shipTrajectory, IM_COL32(80, 200, 255, 255));
    ImGui::End();

    gui.endFrame();
  }
  return 0;
}
