#include <vgl/vgl.h>
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/quaternion.hpp>
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
const int FALCON_NINE_MASS = 549054;     // kg
const int FALCON_NINE_HEIGHT = 70;       // m
const float FALCON_NINE_DIAMETER = 3.7f; // m
const int MERLIN_ENGINE_THRUST = 845000; // N (sea level)
const double EARTH_RADIUS = 6371000.0;   // m

// Nose cone (ogive tip, drawn as a wireframe cone on top of the cylinder)
const float CONE_HEIGHT = 13.1f; // m

// Landing legs (4 legs, deployed outward from the base)
const float LEG_OFFSET = 9.0f;  // m radial distance from rocket axis at tip
const float LEG_LENGTH = 10.0f; // m leg length

// -----------------------------------------------------------------------
// Guidance controller (proportional, gimbal-based)
// -----------------------------------------------------------------------
class GuidanceController
{
public:
  glm::vec3 waypoint;
  float Kp = 0.1f;
  float maxGimbal = 0.15f; // ~8.5 degrees
  bool enabled = false;

  GuidanceController(glm::vec3 wp) : waypoint(wp) {}

  void update(RigidBody &rocket, std::vector<Thruster> &engines, float dt)
  {
    if (!enabled)
      return;

    glm::vec3 rocketUp = rocket.orientation * glm::vec3(0, 0, 1);
    glm::vec3 toWaypoint = waypoint - rocket.position;

    if (glm::length(toWaypoint) < 1.0f)
      return;

    glm::vec3 desiredDir = glm::normalize(toWaypoint);
    glm::vec3 error = glm::cross(rocketUp, desiredDir);

    float pitchCmd = glm::clamp(Kp * error.y, -maxGimbal, maxGimbal);
    float yawCmd = glm::clamp(-Kp * error.x, -maxGimbal, maxGimbal);

    for (auto &eng : engines)
    {
      float rate = 1.0f * dt; // rad/s gimbal rate
      eng.pitch += glm::clamp(pitchCmd - eng.pitch, -rate, rate);
      eng.yaw += glm::clamp(yawCmd - eng.yaw, -rate, rate);
      eng.pitch = glm::clamp(eng.pitch, -maxGimbal, maxGimbal);
      eng.yaw = glm::clamp(eng.yaw, -maxGimbal, maxGimbal);
    }
  }
};

// -----------------------------------------------------------------------
// Drawing helpers
// -----------------------------------------------------------------------
void drawGrid(GUI &gui, float size, float step)
{
  const glm::vec3 gridColor{0.4f, 0.4f, 0.4f};
  const glm::vec3 axisColorX{0.8f, 0.2f, 0.2f};
  const glm::vec3 axisColorY{0.2f, 0.8f, 0.2f};

  for (float i = -size; i <= size; i += step)
  {
    gui.drawLine({i, -size, 0}, {i, size, 0}, i == 0.0f ? axisColorX : gridColor);
    gui.drawLine({-size, i, 0}, {size, i, 0}, i == 0.0f ? axisColorY : gridColor);
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

  // ---- Rocket body (first stage cylinder) -----------------------------
  RigidBody *rocket = world.createBody(
      RigidBodyShape::CYLINDER,
      glm::vec3(FALCON_NINE_DIAMETER / 2.0f, FALCON_NINE_HEIGHT, 0.0f),
      FALCON_NINE_MASS);

  // Place rocket upright: cylinder center at half-height above the ground
  rocket->position.z = FALCON_NINE_HEIGHT * 0.5f;

  // ---- Nose cone (separate rigid body, CONE shape) --------------------
  // CONE local frame: apex at +CONE_HEIGHT/2 along Z, base at -CONE_HEIGHT/2.
  // Attach base to the top of the rocket cylinder.
  const float baseR = FALCON_NINE_DIAMETER * 0.5f;

  RigidBody *noseCone = world.createBody(
      RigidBodyShape::CONE,
      glm::vec3(baseR, CONE_HEIGHT, 0.0f),
      1000.0f); // kg (payload fairing approx.)

  // Nose cone CoM: place geometric centre directly above rocket top.
  noseCone->position.z = rocket->position.z + FALCON_NINE_HEIGHT * 0.5f // top of rocket
                         + CONE_HEIGHT * 0.5f;                          // to cone centre
  noseCone->orientation = rocket->orientation;

  // World pivot = junction between rocket top and cone base
  glm::vec3 noseJunction{0.0f, 0.0f,
                         rocket->position.z + FALCON_NINE_HEIGHT * 0.5f};
  world.addFixedJoint(rocket, noseCone, noseJunction);

  // ---- Landing legs (4 x BOX rigid bodies) ----------------------------
  // Each leg runs from a point on the rocket base-rim outward & downward.
  const float LEG_W = 0.4f; // cross-section width (m)
  const float rocketBaseZ = rocket->position.z - FALCON_NINE_HEIGHT * 0.5f;

  RigidBody *legs[4];
  for (int i = 0; i < 4; ++i)
  {
    float angle = (float(i) / 4.0f) * 2.0f * float(M_PI);

    // Attachment point on rocket base rim (world space)
    glm::vec3 attach(std::cos(angle) * baseR,
                     std::sin(angle) * baseR,
                     rocketBaseZ);

    // Tip of the deployed leg (world space)
    glm::vec3 tip(std::cos(angle) * LEG_OFFSET,
                  std::sin(angle) * LEG_OFFSET,
                  rocketBaseZ - LEG_LENGTH);

    float legLen = glm::length(tip - attach);

    // Orient box so its local +Z axis points from attach to tip
    glm::vec3 legDir = (tip - attach) / legLen;
    glm::vec3 fromZ(0.0f, 0.0f, 1.0f);
    glm::vec3 rotAxis = glm::cross(fromZ, legDir);
    float sinA = glm::length(rotAxis), cosA = glm::dot(fromZ, legDir);
    glm::quat legOrient = (sinA > 1e-6f)
                              ? glm::angleAxis(std::atan2(sinA, cosA), glm::normalize(rotAxis))
                              : glm::quat(1, 0, 0, 0);

    legs[i] = world.createBody(
        RigidBodyShape::BOX,
        glm::vec3(LEG_W, LEG_W, legLen),
        200.0f); // kg per leg

    legs[i]->position = (attach + tip) * 0.5f;
    legs[i]->orientation = legOrient;

    world.addFixedJoint(rocket, legs[i], attach);
  }

  // ---- Merlin engines -------------------------------------------------
  // 1 static center engine + 8 gimbaled outer engines
  std::vector<Thruster> merlins;

  // Center engine (not gimbaled, but we store pitch/yaw = 0)
  merlins.emplace_back(
      glm::vec3(0.0f, 0.0f, -FALCON_NINE_HEIGHT * 0.5f),
      glm::vec3(0, 0, 1),
      MERLIN_ENGINE_THRUST);

  // 8 outer engines evenly spaced in a ring
  const int numOuter = 8;
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
    const float dt = time - lastTime;
    lastTime = time;

    // -- Input ----------------------------------------------------------
    glm::vec2 mousePos = gui.getMousePosition();
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
        for (auto &t : merlins)
          t.pitch += gimbalRate;
      if (gui.isKeyPressed(GLFW_KEY_DOWN))
        for (auto &t : merlins)
          t.pitch -= gimbalRate;
      if (gui.isKeyPressed(GLFW_KEY_LEFT))
        for (auto &t : merlins)
          t.yaw += gimbalRate;
      if (gui.isKeyPressed(GLFW_KEY_RIGHT))
        for (auto &t : merlins)
          t.yaw -= gimbalRate;
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

    // Nose cone (wireframe from its own physics body transform)
    {
      glm::mat3 R = glm::toMat3(noseCone->orientation);
      glm::vec3 axisW = R * glm::vec3(0, 0, 1);
      float halfCH = CONE_HEIGHT * 0.5f;
      glm::vec3 coneBase = noseCone->position - axisW * halfCH; // local -Z = base
      glm::vec3 coneApex = noseCone->position + axisW * halfCH; // local +Z = apex

      glm::vec3 perpX;
      if (std::abs(axisW.z) < 0.99f)
        perpX = glm::normalize(glm::cross(axisW, glm::vec3(0, 0, 1)));
      else
        perpX = glm::normalize(glm::cross(axisW, glm::vec3(1, 0, 0)));
      glm::vec3 perpY = glm::cross(axisW, perpX);

      const int coneSegs = 8;
      for (int i = 0; i < coneSegs; ++i)
      {
        float a0 = (float(i) / coneSegs) * 2.0f * float(M_PI);
        float a1 = (float(i + 1) / coneSegs) * 2.0f * float(M_PI);
        glm::vec3 p0 = coneBase + noseCone->size.x * (std::cos(a0) * perpX + std::sin(a0) * perpY);
        glm::vec3 p1 = coneBase + noseCone->size.x * (std::cos(a1) * perpX + std::sin(a1) * perpY);
        gui.drawLine(p0, coneApex, rocketColor);
        gui.drawLine(p0, p1, rocketColor);
      }
    }

    // Landing legs (each is a BOX rigid body — draw from its own transform)
    {
      glm::vec3 legColor{0.75f, 0.75f, 0.75f};
      for (int i = 0; i < 4; ++i)
        gui.drawBox(legs[i]->position, legs[i]->size,
                    legs[i]->orientation, legColor);
    }

    // Engine thrust vectors
    for (auto &t : merlins)
    {
      glm::vec3 mount = t.getWorldMountPosition(*rocket);
      glm::vec3 thrustDir = t.getWorldThrustDirection(*rocket);
      gui.drawLine(mount, mount - thrustDir * 10.0f, {1.0f, 1.0f, 0.0f});
    }

    gui.endFrame();
  }

  return 0;
}
