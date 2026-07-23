#include <vgl/vgl.h>
#include <rigidbody/PhysicsWorld.h>
#include <rigidbody/actuators/ReactionWheel.h>
#include <rigidbody/sensors/IMU.h>
#include "ADCS.h"
#include "common/World.h"
#include <random>
#include <memory>
#include <cmath>
#include <cstdio>
#include <glm/gtc/constants.hpp>

// ---------------------------------------------------------------------------
// Full cubesat ADCS: pointing modes (Nadir/Sun/Detumble/Target/Slew/Fine),
// an IMU as the only source of attitude/rate information the flight
// software ever reads (see ADCS.h/.cpp), and randomly-occurring reaction
// wheel faults (reduced torque or complete failure). Otherwise the same
// pyramid-wheel cubesat as examples/cubesat_pyramid was before this pass.
//
// Controls:
//   [1-6]   pointing mode: Nadir / Sun / Detumble / Target / Slew / Fine
//   [Space] new random target (for Target/Slew/Fine modes)
//   [T]     kick the body into a random tumble (to test Detumble)
//   [F]     force a wheel fault immediately (faults also occur on their own)
// ---------------------------------------------------------------------------
struct Cubesat
{
  RigidBody *body = nullptr;
  std::vector<ReactionWheel *> wheels;
  IMU imu; // re-mounted off-center in buildCubesatPyramid()
};

static Cubesat buildCubesatPyramid(PhysicsWorld &world)
{
  Cubesat sat;
  sat.body = world.createBody(
      RigidBodyShape::BOX,
      glm::vec3(0.1f, 0.1f, 0.1f), // 10 x 10 x 10 cm
      1.33f);                      // max mass of 1U cubesat (kg)

  sat.body->position.z = sat.body->size.y * 3; // float above the ground

  // IMU board mounted in a corner of the bus, not at the center of mass --
  // like a real PCB, so its accelerometer isn't trivially always-zero (it
  // picks up centripetal/tangential terms from body rotation).
  sat.imu = IMU(glm::vec3(0.03f, 0.03f, 0.02f));

  // Pyramid layout: 4 wheels, each spin axis tilted `skew` from body +Z,
  // spaced 90 degrees apart in azimuth. Mounted in a small cluster near the
  // +Z face rather than at the body center, matching how a real RWA pyramid
  // bracket is bolted to one panel.
  const float skew = glm::radians(45.0f);
  const float mountRadius = 0.03f;
  const float mountHeight = 0.04f;

  for (int i = 0; i < 4; ++i)
  {
    float azimuth = glm::radians(45.0f) + i * glm::half_pi<float>(); // 45, 135, 225, 315 deg

    glm::vec3 axis(std::sin(skew) * std::cos(azimuth),
                   std::sin(skew) * std::sin(azimuth),
                   std::cos(skew));

    glm::vec3 mountPos(mountRadius * std::cos(azimuth),
                       mountRadius * std::sin(azimuth),
                       mountHeight);

    auto wheel = std::make_unique<ReactionWheel>(
        mountPos,
        axis,
        0.001f,                                      // max torque (Nm) — same as the 3-wheel cubesat
        6000.0f * (2.0f * glm::pi<float>() / 60.0f), // 6000 RPM max
        1e-6f);                                      // wheel inertia (kg*m^2)

    sat.wheels.push_back(wheel.get());
    sat.body->addForceGenerator(std::move(wheel));
  }

  return sat;
}

// ---------------------------------------------------------------------------
// Random points on the unit sphere, scaled out -- one for arbitrary
// pointing targets, one (farther out, for visual distinction) for the sun.
// ---------------------------------------------------------------------------
static glm::vec3 randomTarget()
{
  static std::mt19937 rng(std::random_device{}());
  static std::uniform_real_distribution<float> dist(-1.0f, 1.0f);
  glm::vec3 v(dist(rng), dist(rng), dist(rng));
  return glm::normalize(v);
}

static glm::vec3 randomSunPosition()
{
  static std::mt19937 rng(std::random_device{}());
  static std::uniform_real_distribution<float> dist(-1.0f, 1.0f);
  glm::vec3 v(dist(rng), dist(rng), dist(rng));
  return glm::normalize(v) * 2.0f;
}

// ---------------------------------------------------------------------------
// Randomly fails or degrades a wheel every so often, independent of
// anything the flight software does -- a real bearing/driver fault doesn't
// wait for a convenient moment. Faults persist once triggered (no
// self-repair); restart the scenario to clear them.
// ---------------------------------------------------------------------------
struct WheelFaultInjector
{
  std::mt19937 rng{std::random_device{}()};
  std::exponential_distribution<float> nextFaultDist;
  float timeUntilNextFault;

  explicit WheelFaultInjector(float meanSecondsBetweenFaults)
      : nextFaultDist(1.0f / meanSecondsBetweenFaults)
  {
    timeUntilNextFault = nextFaultDist(rng);
  }

  void update(float dt, std::vector<ReactionWheel *> &wheels)
  {
    timeUntilNextFault -= dt;
    if (timeUntilNextFault > 0.0f)
      return;
    trigger(wheels);
  }

  void trigger(std::vector<ReactionWheel *> &wheels)
  {
    timeUntilNextFault = nextFaultDist(rng);

    std::uniform_int_distribution<int> pickWheel(0, (int)wheels.size() - 1);
    ReactionWheel *w = wheels[pickWheel(rng)];

    std::uniform_real_distribution<float> roll(0.0f, 1.0f);
    if (roll(rng) < 0.4f)
    {
      w->healthFactor = 0.0f; // complete failure
    }
    else
    {
      std::uniform_real_distribution<float> degrade(0.1f, 0.6f);
      w->healthFactor = degrade(rng); // reduced torque, not dead
    }
  }
};

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
  constexpr float PLOT_WIDTH = 1.0f;
  constexpr float PLOT_HEIGHT = 0.18f;
  constexpr float PLOT_GAP = 0.05f;

  constexpr float MEAN_SECONDS_BETWEEN_FAULTS = 25.0f;
  constexpr float TUMBLE_KICK_RAD_S = 1.5f;
}

// ---------------------------------------------------------------------------
// Draw helpers
// ---------------------------------------------------------------------------

// Body drawn as a 12-edge wireframe box instead of a solid box.
void drawSatelliteWireframe(GUI &gui, RigidBody *sat)
{
  glm::vec3 h = sat->size * 0.5f;
  glm::quat q = sat->orientation;
  glm::vec3 p = sat->position;
  const glm::vec3 color{1.0f, 1.0f, 0.0f};

  glm::vec3 c[8] = {
      p + q * glm::vec3(-h.x, -h.y, -h.z),
      p + q * glm::vec3(+h.x, -h.y, -h.z),
      p + q * glm::vec3(+h.x, +h.y, -h.z),
      p + q * glm::vec3(-h.x, +h.y, -h.z),
      p + q * glm::vec3(-h.x, -h.y, +h.z),
      p + q * glm::vec3(+h.x, -h.y, +h.z),
      p + q * glm::vec3(+h.x, +h.y, +h.z),
      p + q * glm::vec3(-h.x, +h.y, +h.z),
  };

  // Bottom face (4)
  gui.drawLine(c[0], c[1], color);
  gui.drawLine(c[1], c[2], color);
  gui.drawLine(c[2], c[3], color);
  gui.drawLine(c[3], c[0], color);
  // Top face (4)
  gui.drawLine(c[4], c[5], color);
  gui.drawLine(c[5], c[6], color);
  gui.drawLine(c[6], c[7], color);
  gui.drawLine(c[7], c[4], color);
  // Vertical edges (4)
  gui.drawLine(c[0], c[4], color);
  gui.drawLine(c[1], c[5], color);
  gui.drawLine(c[2], c[6], color);
  gui.drawLine(c[3], c[7], color);

  float arrowLength = 0.25f;
  gui.drawArrow(p, p + q * glm::vec3(1, 0, 0) * arrowLength, glm::vec3(1, 0, 0));
  gui.drawArrow(p, p + q * glm::vec3(0, 1, 0) * arrowLength, glm::vec3(0, 1, 0));
  gui.drawArrow(p, p + q * glm::vec3(0, 0, 1) * arrowLength, glm::vec3(0, 0, 1));
}

// Wheels drawn as flat cylinders at their actual mount position/orientation,
// with a speed arrow from each wheel's center. Color communicates health
// first (magenta = degraded, near-black = dead) and saturation only for
// wheels that are actually healthy.
void drawReactionWheels(GUI &gui, const std::vector<ReactionWheel *> &reactionWheels, RigidBody *sat)
{
  const float wheelRadius = 0.02f;
  const float wheelThickness = 0.006f;
  const float arrowLength = 0.05f;

  glm::vec3 totalAngular{0};
  for (auto &wheel : reactionWheels)
  {
    glm::vec3 worldPos = wheel->getWorldMountPosition(*sat);
    glm::vec3 worldAxis = wheel->getWorldSpinAxis(*sat);

    glm::vec3 color;
    if (wheel->healthFactor <= 0.01f)
      color = {0.15f, 0.15f, 0.15f}; // dead
    else if (wheel->healthFactor < 0.99f)
      color = {0.85f, 0.1f, 0.85f}; // degraded
    else
    {
      float satRatio = wheel->getSaturationRatio();
      float absSatRatio = std::abs(satRatio);
      if (absSatRatio < 0.5f)
        color = {0, 1, 0};
      else if (absSatRatio < 0.9f)
        color = {1, 1, 0};
      else
        color = {1, 0, 0};
    }

    // Flat "puck": thin along the spin axis, wide across it.
    gui.drawCylinder(worldPos, wheelRadius, wheelThickness, worldAxis, glm::quat(1, 0, 0, 0), color);

    // Speed arrow from the wheel's own center, along its spin axis --
    // reflects actual current speed regardless of health.
    float satRatio = wheel->getSaturationRatio();
    gui.drawArrow(worldPos, worldPos + worldAxis * arrowLength * satRatio, color);
    totalAngular += worldAxis * satRatio;
  }
  gui.drawArrow(sat->position, sat->position + totalAngular * arrowLength * 4.0f, {1.0f, 0.65f, 0});
}

static const char *modeName(PointingMode m)
{
  switch (m)
  {
  case PointingMode::NADIR:
    return "NADIR";
  case PointingMode::SUN_POINTING:
    return "SUN_POINTING";
  case PointingMode::DETUMBLE:
    return "DETUMBLE";
  case PointingMode::TARGET:
    return "TARGET";
  case PointingMode::SLEW:
    return "SLEW";
  case PointingMode::FINE_POINTING:
    return "FINE_POINTING";
  }
  return "?";
}

static void updateTitle(GLFWwindow *win, PointingMode mode, float attErrDeg,
                        float rateRadS, int faultedWheels)
{
  char buf[256];
  std::snprintf(buf, sizeof(buf),
                "CubeSat ADCS (Pyramid RWA)  |  [1-6] Mode  [Space] Target  [T] Tumble  [F] Fault  |  "
                "Mode: %s  |  Att err: %.1f deg  |  Rate: %.3f rad/s  |  Faulted wheels: %d",
                modeName(mode), attErrDeg, rateRadS, faultedWheels);
  glfwSetWindowTitle(win, buf);
}

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------
int main()
{
  GUI gui(800, 600, "CubeSat ADCS (Pyramid RWA)");
  World scene(WorldType::SPACE);
  scene.apply(gui);
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
  Cubesat sat = buildCubesatPyramid(world);

  // Flight software: ADCS holds references to the body, the wheels (its
  // actuators), and the IMU (its only sensor) — see ADCS.h/.cpp. It never
  // reads body->orientation/angularVelocity directly.
  ADCS adcs(sat.body, sat.wheels, &sat.imu);
  adcs.gravity = world.gravity; // zero in this scenario, but ADCS doesn't assume that
  adcs.target = randomTarget();
  adcs.sunPosition = randomSunPosition();
  float adcsTimer = 0.0f;

  WheelFaultInjector faultInjector(Config::MEAN_SECONDS_BETWEEN_FAULTS);

  float lastTime = glfwGetTime();
  while (!gui.shouldClose())
  {
    float time = glfwGetTime();
    float dt = time - lastTime;
    lastTime = time;

    // =================== INPUT ===================
    glm::vec2 mousePos = gui.getMousePosition();
    glm::vec2 mouseDelta = mousePos - lastMousePos;
    lastMousePos = mousePos;

    if (gui.isKeyJustPressed(GLFW_KEY_1))
      adcs.mode = PointingMode::NADIR;
    if (gui.isKeyJustPressed(GLFW_KEY_2))
      adcs.mode = PointingMode::SUN_POINTING;
    if (gui.isKeyJustPressed(GLFW_KEY_3))
      adcs.mode = PointingMode::DETUMBLE;
    if (gui.isKeyJustPressed(GLFW_KEY_4))
      adcs.mode = PointingMode::TARGET;
    if (gui.isKeyJustPressed(GLFW_KEY_5))
      adcs.mode = PointingMode::SLEW;
    if (gui.isKeyJustPressed(GLFW_KEY_6))
      adcs.mode = PointingMode::FINE_POINTING;

    if (gui.isKeyJustPressed(GLFW_KEY_SPACE))
    {
      adcs.target = randomTarget();
      adcs.resetController(); // clear integral windup from previous target
    }

    if (gui.isKeyJustPressed(GLFW_KEY_T))
    {
      static std::mt19937 tumbleRng(std::random_device{}());
      static std::uniform_real_distribution<float> tumbleDist(-Config::TUMBLE_KICK_RAD_S, Config::TUMBLE_KICK_RAD_S);
      sat.body->angularVelocity = glm::vec3(tumbleDist(tumbleRng), tumbleDist(tumbleRng), tumbleDist(tumbleRng));
    }

    if (gui.isKeyJustPressed(GLFW_KEY_F))
      faultInjector.trigger(sat.wheels);

    orbit.handleInput(gui, mouseDelta, gui.getScrollDelta());
    orbit.applyToCamera(gui.camera);

    // =================== FLIGHT SOFTWARE (20 Hz) ===================
    // Reads the IMU (only) as sensors, commands the reaction wheels.
    adcsTimer += dt;
    if (adcsTimer > 0.05f)
    {
      adcs.run(adcsTimer);
      adcsTimer = 0.0f;
    }

    faultInjector.update(dt, sat.wheels);

    // =================== PHYSICS ===================
    world.step(dt);

    // =================== DRAW ===================
    gui.beginFrame();
    scene.draw(gui, Config::GRID_SIZE, Config::GRID_STEP);
    drawSatelliteWireframe(gui, sat.body);
    drawReactionWheels(gui, sat.wheels, sat.body);
    gui.drawSphere(adcs.target, 0.05f, {0, 1.0f, 0});
    gui.drawSphere(adcs.sunPosition, 0.12f, {1.0f, 0.9f, 0.1f});
    gui.endFrame();
  }
  return 0;
}
