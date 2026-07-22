#include <vgl/vgl.h>
#include <rigidbody/PhysicsWorld.h>
#include <rigidbody/actuators/ReactionWheel.h>
#include "ADCS.h"
#include <random>
#include <memory>
#include <glm/gtc/constants.hpp>

// ---------------------------------------------------------------------------
// Spacecraft definition: a 1U cubesat body with 3-axis reaction wheels.
// This is scenario code, not engine code — the engine only knows about
// RigidBody + ForceGenerator; "Cubesat" is just how this scenario groups the
// actuators it built for its own flight software to reference.
// ---------------------------------------------------------------------------
struct Cubesat
{
  RigidBody *body = nullptr;
  std::vector<ReactionWheel *> wheels;
};

static Cubesat buildCubesat(PhysicsWorld &world)
{
  Cubesat sat;
  sat.body = world.createBody(
      RigidBodyShape::BOX,
      glm::vec3(0.1f, 0.1f, 0.1f), // 10 x 10 x 10 cm
      1.33f);                      // max mass of 1U cubesat (kg)

  sat.body->position.z = sat.body->size.y * 3; // float above the ground

  glm::vec3 axes[3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
  for (int i = 0; i < 3; ++i)
  {
    auto wheel = std::make_unique<ReactionWheel>(
        glm::vec3(0.0f), // mount at body center
        axes[i],
        0.001f,                                        // max torque (Nm)
        6000.0f * (2.0f * glm::pi<float>() / 60.0f),    // 6000 RPM max
        1e-6f);                                         // wheel inertia (kg*m^2)

    sat.wheels.push_back(wheel.get());
    sat.body->addForceGenerator(std::move(wheel));
  }

  return sat;
}

// ---------------------------------------------------------------------------
// Random target on unit sphere
// ---------------------------------------------------------------------------
static glm::vec3 randomTarget()
{
  static std::mt19937 rng(std::random_device{}());
  static std::uniform_real_distribution<float> dist(-1.0f, 1.0f);
  glm::vec3 v(dist(rng), dist(rng), dist(rng));
  return glm::normalize(v);
}

// ---------------------------------------------------------------------------
// Simple rolling telemetry buffer
// ---------------------------------------------------------------------------
struct PlotBuffer
{
  static constexpr int N = 300;
  float data[N] = {};
  int head = 0;

  void push(float v) { data[head++ % N] = v; }
  float get(int i) const { return data[(head + i) % N]; }
};

// Draw a single time-series strip in world space.
// origin: bottom-left corner, width along +X, height along +Z (Z-up world).
static void drawPlot(GUI &gui, const PlotBuffer &buf,
                     float minVal, float maxVal,
                     glm::vec3 origin, float width, float height,
                     glm::vec3 color)
{
  const glm::vec3 borderColor{0.3f, 0.3f, 0.3f};
  gui.drawLine(origin,                              origin + glm::vec3(width, 0, 0),      borderColor);
  gui.drawLine(origin + glm::vec3(0, 0, height),   origin + glm::vec3(width, 0, height),  borderColor);
  gui.drawLine(origin,                              origin + glm::vec3(0, 0, height),      borderColor);
  gui.drawLine(origin + glm::vec3(width, 0, 0),    origin + glm::vec3(width, 0, height),  borderColor);

  float range = (maxVal - minVal);
  if (range == 0.0f) return;

  for (int i = 0; i < PlotBuffer::N - 1; i++)
  {
    float x0 = (float)i       / (PlotBuffer::N - 1) * width;
    float x1 = (float)(i + 1) / (PlotBuffer::N - 1) * width;
    float y0 = glm::clamp((buf.get(i)     - minVal) / range, 0.0f, 1.0f) * height;
    float y1 = glm::clamp((buf.get(i + 1) - minVal) / range, 0.0f, 1.0f) * height;
    gui.drawLine(origin + glm::vec3(x0, 0, y0),
                 origin + glm::vec3(x1, 0, y1),
                 color);
  }
}

// ---------------------------------------------------------------------------
// Config
// ---------------------------------------------------------------------------
namespace Config
{
  // Camera
  constexpr float CAMERA_NEAR = 0.1f;
  constexpr float CAMERA_FAR = 100.0f;
  constexpr float CAMERA_FOV = 45.0f;
  constexpr float CAMERA_INITIAL_DISTANCE = 1.0f;
  constexpr float CAMERA_MIN_DISTANCE = 0.1f;
  constexpr float CAMERA_MAX_DISTANCE = 100.0f;
  constexpr float ZOOM_SENSITIVITY = 1.0f;
  constexpr float PAN_SENSITIVITY = 0.2f;

  // Grid
  constexpr float GRID_SIZE = 5.0f;
  constexpr float GRID_STEP = 1.0f;

  // Plot panel (world space, Y=-1.5 plane, X in [-0.5, 0.5], Z up)
  const glm::vec3 PLOT_ORIGIN{-0.5f, -1.5f, 0.0f};
  constexpr float PLOT_WIDTH  = 1.0f;
  constexpr float PLOT_HEIGHT = 0.18f;
  constexpr float PLOT_GAP    = 0.05f;
}

// ---------------------------------------------------------------------------
// Draw helpers
// ---------------------------------------------------------------------------
void drawGrid(GUI &gui)
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

void drawSatellite(GUI &gui, RigidBody *sat)
{
  gui.drawBox(sat->position, sat->size, sat->orientation, {1.0f, 1.0f, 0});

  float arrowLength = 0.25f;
  glm::vec3 bodyXAxis = sat->orientation * glm::vec3(1, 0, 0);
  glm::vec3 bodyYAxis = sat->orientation * glm::vec3(0, 1, 0);
  glm::vec3 bodyZAxis = sat->orientation * glm::vec3(0, 0, 1);
  gui.drawArrow(sat->position, sat->position + bodyXAxis * arrowLength, glm::vec3(1, 0, 0));
  gui.drawArrow(sat->position, sat->position + bodyYAxis * arrowLength, glm::vec3(0, 1, 0));
  gui.drawArrow(sat->position, sat->position + bodyZAxis * arrowLength, glm::vec3(0, 0, 1));
}

void drawReactionWheels(GUI &gui, const std::vector<ReactionWheel *> &reactionWheels, RigidBody *sat)
{
  glm::vec3 totalAngular{0};
  float axisLength = 0.20f;
  for (auto &wheel : reactionWheels)
  {
    glm::vec3 worldPos  = wheel->getWorldMountPosition(*sat);
    glm::vec3 worldAxis = wheel->getWorldSpinAxis(*sat);

    float satRatio    = wheel->getSaturationRatio();
    float absSatRatio = std::abs(satRatio);
    glm::vec3 axisColor;
    if (absSatRatio < 0.5f)       axisColor = {0, 1, 0};
    else if (absSatRatio < 0.9f)  axisColor = {1, 1, 0};
    else                          axisColor = {1, 0, 0};

    gui.drawArrow(worldPos, worldPos + worldAxis * axisLength * satRatio, axisColor);
    totalAngular += worldAxis * satRatio;
  }
  gui.drawArrow(sat->position, sat->position + totalAngular * axisLength, {1.0f, 0.65f, 0});
}

void drawTelemetryPlots(GUI &gui,
                        const PlotBuffer &attErr,
                        const PlotBuffer &angRate,
                        const PlotBuffer wheelSpeeds[3])
{
  float step = Config::PLOT_HEIGHT + Config::PLOT_GAP;

  // Attitude error (rad)  — top
  drawPlot(gui, attErr,         0.0f, glm::pi<float>(),
           Config::PLOT_ORIGIN + glm::vec3(0, 0, step * 2),
           Config::PLOT_WIDTH, Config::PLOT_HEIGHT, {0.2f, 0.8f, 1.0f});

  // Angular rate magnitude (rad/s)
  drawPlot(gui, angRate,        0.0f, 2.0f,
           Config::PLOT_ORIGIN + glm::vec3(0, 0, step * 1),
           Config::PLOT_WIDTH, Config::PLOT_HEIGHT, {1.0f, 0.6f, 0.1f});

  // Wheel speed saturation ratio per axis (R/G/B = X/Y/Z wheel)
  const glm::vec3 wheelColors[3] = {{1, 0.3f, 0.3f}, {0.3f, 1, 0.3f}, {0.3f, 0.3f, 1}};
  for (int i = 0; i < 3; i++)
    drawPlot(gui, wheelSpeeds[i], -1.0f, 1.0f,
             Config::PLOT_ORIGIN,
             Config::PLOT_WIDTH, Config::PLOT_HEIGHT, wheelColors[i]);
}

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------
int main()
{
  GUI gui(800, 600, "CubeSat Attitude Control");
  gui.camera
      .setUp({0, 0, 1})
      .setClipPlanes(Config::CAMERA_NEAR, Config::CAMERA_FAR)
      .setFOV(Config::CAMERA_FOV);

  OrbitalCamera orbit(
      Config::CAMERA_INITIAL_DISTANCE,
      45.0f, 0.0f,
      {0, 0, 0});
  orbit.setMaxDistance(Config::CAMERA_MAX_DISTANCE)
      .setMinDistance(Config::CAMERA_MIN_DISTANCE)
      .setZoomSensitivity(Config::ZOOM_SENSITIVITY)
      .setPanSensitivity(Config::PAN_SENSITIVITY);

  glm::vec2 lastMousePos = gui.getMousePosition();

  PhysicsWorld world;
  Cubesat sat = buildCubesat(world);

  // Flight software: ADCS holds references to the body (its attitude/rate
  // sensors) and the wheels (its actuators) — see ADCS.h/.cpp.
  ADCS adcs(sat.body, sat.wheels);
  adcs.target = randomTarget();
  float adcsTimer = 0.0f;

  // Telemetry buffers
  PlotBuffer plotAttErr;
  PlotBuffer plotAngRate;
  PlotBuffer plotWheelSpeed[3];

  float lastTime = glfwGetTime();
  while (!gui.shouldClose())
  {
    float time = glfwGetTime();
    float dt   = time - lastTime;
    lastTime   = time;

    // =================== INPUT ===================
    glm::vec2 mousePos   = gui.getMousePosition();
    glm::vec2 mouseDelta = mousePos - lastMousePos;
    lastMousePos         = mousePos;

    if (gui.isKeyJustPressed(GLFW_KEY_SPACE))
    {
      adcs.target = randomTarget();
      adcs.resetController(); // clear integral windup from previous target
    }

    orbit.handleInput(gui, mouseDelta, gui.getScrollDelta());
    orbit.applyToCamera(gui.camera);

    // =================== FLIGHT SOFTWARE (20 Hz) ===================
    // Reads body attitude/rate as sensors, commands the reaction wheels.
    adcsTimer += dt;
    if (adcsTimer > 0.05f)
    {
      adcs.run(adcsTimer);
      adcsTimer = 0.0f;
    }

    // =================== PHYSICS ===================
    world.step(dt);

    // =================== TELEMETRY ===================
    {
      RigidBody &body = *sat.body;

      // Attitude error: angle between body +Z and target direction
      glm::vec3 bodyZ    = body.orientation * glm::vec3(0, 0, 1);
      glm::vec3 toTarget = glm::normalize(adcs.target - body.position);
      float errRad       = std::acos(glm::clamp(glm::dot(bodyZ, toTarget), -1.0f, 1.0f));
      plotAttErr.push(errRad);

      plotAngRate.push(glm::length(body.angularVelocity));

      for (int i = 0; i < 3; i++)
        plotWheelSpeed[i].push(sat.wheels[i]->getSaturationRatio());
    }

    // =================== DRAW ===================
    gui.beginFrame();
    drawGrid(gui);
    drawSatellite(gui, sat.body);
    drawReactionWheels(gui, sat.wheels, sat.body);
    gui.drawSphere(adcs.target, 0.05f, {0, 1.0f, 0});
    drawTelemetryPlots(gui, plotAttErr, plotAngRate, plotWheelSpeed);
    gui.endFrame();
  }
  return 0;
}
