#include <vaughngl/vaughngl.h>
#include "PhysicsWorld.h"
#include "Thruster.h"

/* FALCON 9 NOTES

- 9 Merlin engines, configuration: 8 gimbaled outer, 1 static center
- Total thrust (sea level): 7607 kN
- Total thrust (vacuum):    8227 kN

Merlin Engines
- Propellant: LOX / RP-1
- Thrust: 845 kN (sea level) / 981 kN (vacuum)

*/

// -----------------------------------------------------------------------
// Constants
// -----------------------------------------------------------------------
const int   FALCON_NINE_MASS     = 549054;    // kg
const int   FALCON_NINE_HEIGHT   = 70;        // m
const float FALCON_NINE_DIAMETER = 3.7f;      // m
const int   MERLIN_ENGINE_THRUST = 845000;    // N (sea level)
const double EARTH_RADIUS        = 6371000.0; // m

// -----------------------------------------------------------------------
// Guidance controller (proportional, gimbal-based)
// -----------------------------------------------------------------------
class GuidanceController
{
public:
  glm::vec3 waypoint;
  float Kp        = 0.1f;
  float maxGimbal = 0.15f; // ~8.5 degrees
  bool  enabled   = false;

  GuidanceController(glm::vec3 wp) : waypoint(wp) {}

  void update(RigidBody &rocket, std::vector<Thruster> &engines, float dt)
  {
    if (!enabled)
      return;

    glm::vec3 rocketUp  = rocket.orientation * glm::vec3(0, 0, 1);
    glm::vec3 toWaypoint = waypoint - rocket.position;

    if (glm::length(toWaypoint) < 1.0f)
      return;

    glm::vec3 desiredDir = glm::normalize(toWaypoint);
    glm::vec3 error      = glm::cross(rocketUp, desiredDir);

    float pitchCmd = glm::clamp( Kp * error.y, -maxGimbal, maxGimbal);
    float yawCmd   = glm::clamp(-Kp * error.x, -maxGimbal, maxGimbal);

    for (auto &eng : engines)
    {
      float rate = 1.0f * dt; // rad/s gimbal rate
      eng.pitch += glm::clamp(pitchCmd - eng.pitch, -rate, rate);
      eng.yaw   += glm::clamp(yawCmd   - eng.yaw,   -rate, rate);
      eng.pitch  = glm::clamp(eng.pitch, -maxGimbal, maxGimbal);
      eng.yaw    = glm::clamp(eng.yaw,   -maxGimbal, maxGimbal);
    }
  }
};

// -----------------------------------------------------------------------
// Drawing helpers
// -----------------------------------------------------------------------
void drawGrid(GUI &gui, float size, float step)
{
  const glm::vec3 gridColor {0.4f, 0.4f, 0.4f};
  const glm::vec3 axisColorX{0.8f, 0.2f, 0.2f};
  const glm::vec3 axisColorY{0.2f, 0.8f, 0.2f};

  for (float i = -size; i <= size; i += step)
  {
    gui.drawLine({i, -size, 0}, {i,  size, 0}, i == 0.0f ? axisColorX : gridColor);
    gui.drawLine({-size, i, 0}, { size, i, 0}, i == 0.0f ? axisColorY : gridColor);
  }
}

// -----------------------------------------------------------------------
// Main
// -----------------------------------------------------------------------
int main()
{
  // ---- Window & camera ------------------------------------------------
  GUI gui(800, 600, "Falcon 9 Simulation");
  gui.setLighting(false);

  gui.camera
      .setUp({0, 0, 1})
      .setClipPlanes(1.0f, 20000e3f)
      .setFOV(45.0f);

  OrbitalCamera orbit(1000.0f, 45.0f, 15.0f, {0, 0, 0});
  orbit.setMinDistance(10.0f)
       .setMaxDistance(100e6f)
       .setZoomSensitivity(10.0f)
       .setPanSensitivity(0.2f);

  glm::vec2 lastMousePos = gui.getMousePosition();

  // ---- Physics world --------------------------------------------------
  PhysicsWorld world;
  world.gravity = glm::vec3(0, 0, -9.81f);

  // ---- Rocket body ----------------------------------------------------
  RigidBody *rocket = world.createBody(
      RigidBodyShape::CYLINDER,
      glm::vec3(FALCON_NINE_DIAMETER / 2.0f, FALCON_NINE_HEIGHT, 0.0f),
      FALCON_NINE_MASS);

  // Place rocket upright on the ground (center at half-height)
  rocket->position.z = FALCON_NINE_HEIGHT * 0.5f;

  // ---- Merlin engines -------------------------------------------------
  // 1 static center engine + 8 gimbaled outer engines
  std::vector<Thruster> merlins;

  // Center engine (not gimbaled, but we store pitch/yaw = 0)
  merlins.emplace_back(
      glm::vec3(0.0f, 0.0f, -FALCON_NINE_HEIGHT * 0.5f),
      glm::vec3(0, 0, 1),
      MERLIN_ENGINE_THRUST);

  // 8 outer engines evenly spaced in a ring
  const int   numOuter   = 8;
  const float ringRadius = 1.5f; // m
  for (int i = 0; i < numOuter; ++i)
  {
    float angle = (2.0f * M_PI / numOuter) * i;
    merlins.emplace_back(
        glm::vec3(std::cos(angle) * ringRadius,
                  std::sin(angle) * ringRadius,
                  -FALCON_NINE_HEIGHT * 0.5f),
        glm::vec3(0, 0, 1),
        MERLIN_ENGINE_THRUST);
  }

  // ---- Guidance -------------------------------------------------------
  glm::vec3 waypoint(1000.0f, 0.0f, 700.0f); // 1 km downrange, 700 m altitude
  GuidanceController guidance(waypoint);

  // ---- Input state ----------------------------------------------------
  bool gKeyWasPressed = false;
  float lastTime = glfwGetTime();

  // ---- Main loop ------------------------------------------------------
  while (!gui.shouldClose())
  {
    const float time = glfwGetTime();
    const float dt   = time - lastTime;
    lastTime = time;

    // -- Input ----------------------------------------------------------
    glm::vec2 mousePos   = gui.getMousePosition();
    glm::vec2 mouseDelta = mousePos - lastMousePos;
    lastMousePos = mousePos;

    // Toggle guidance with G key
    bool gKeyPressed = gui.isKeyPressed(GLFW_KEY_G);
    if (gKeyPressed && !gKeyWasPressed)
      guidance.enabled = !guidance.enabled;
    gKeyWasPressed = gKeyPressed;

    // Manual gimbal control (arrow keys, when guidance is off)
    if (!guidance.enabled)
    {
      float gimbalRate = glm::radians(40.0f) * dt;

      if (gui.isKeyPressed(GLFW_KEY_UP))
        for (auto &t : merlins) t.pitch += gimbalRate;
      if (gui.isKeyPressed(GLFW_KEY_DOWN))
        for (auto &t : merlins) t.pitch -= gimbalRate;
      if (gui.isKeyPressed(GLFW_KEY_LEFT))
        for (auto &t : merlins) t.yaw += gimbalRate;
      if (gui.isKeyPressed(GLFW_KEY_RIGHT))
        for (auto &t : merlins) t.yaw -= gimbalRate;
    }
    else
    {
      guidance.update(*rocket, merlins, dt);
    }

    // Fire engines (SPACE for manual, always fire when guidance active)
    if (gui.isKeyPressed(GLFW_KEY_SPACE) || guidance.enabled)
    {
      for (auto &t : merlins)
        t.apply(*rocket, 1.0f); // full throttle
    }

    // -- Simulate -------------------------------------------------------
    orbit.handleInput(gui, mouseDelta, gui.getScrollDelta());
    orbit.applyToCamera(gui.camera);
    world.step(dt);

    // -- Draw -----------------------------------------------------------
    gui.beginFrame();

    drawGrid(gui, 100.0f, 10.0f);

    // Earth
    gui.drawSphere({0.0f, 0.0f, (float)-EARTH_RADIUS}, (float)EARTH_RADIUS, {0.1f, 0.3f, 0.8f});

    // Waypoint
    glm::vec3 wpColor = guidance.enabled ? glm::vec3(0, 1, 0) : glm::vec3(0.5f);
    gui.drawSphere(waypoint, 10.0f, wpColor);
    if (guidance.enabled)
      gui.drawLine(rocket->position, waypoint, {0.0f, 1.0f, 0.5f});

    // Rocket
    glm::vec3 rocketColor = guidance.enabled ? glm::vec3(0.2f, 0.6f, 1.0f) : glm::vec3(1.0f, 0.3f, 0.3f);
    gui.drawCylinder(rocket->position,
                     rocket->size.x,
                     rocket->size.y,
                     {0, 0, 1},
                     rocket->orientation,
                     rocketColor);

    // Engine thrust vectors
    for (auto &t : merlins)
    {
      glm::vec3 mount    = t.getWorldMountPosition(*rocket);
      glm::vec3 thrustDir = t.getWorldThrustDirection(*rocket);
      gui.drawLine(mount, mount - thrustDir * 10.0f, {1.0f, 1.0f, 0.0f});
    }

    gui.endFrame();
  }

  return 0;
}
