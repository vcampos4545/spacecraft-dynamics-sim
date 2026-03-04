#include <vgl/vgl.h>
#include "Universe.h"
#include "spacecraft/Satellite.h"
#include <deque>
#include <cstdio>
#include <glm/gtc/constants.hpp>

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------
namespace Config
{
  constexpr double EARTH_MASS   = 5.972e24;    // kg
  constexpr float  EARTH_RADIUS = 6.371e6f;    // m
  constexpr double ALT_M        = 500e3;       // 500 km LEO
  constexpr double INC_RAD      = 0.7854;      // 45° inclination
  constexpr double RAAN_RAD     = 0.0;

  constexpr float  FAR_PLANE    = 1.5e9f;      // 1.5 billion m (past the Moon)
  constexpr float  NEAR_PLANE   = 1.0f;        // 1 m

  constexpr double CAM_DIST_INIT = 2.0e7;      // 20,000 km
  constexpr double CAM_DIST_MIN  = 1.0;        // 1 m
  constexpr double CAM_DIST_MAX  = 1.0e9;      // 1,000,000 km

  constexpr int    TRAIL_POINTS   = 400;
  constexpr double TRAIL_INTERVAL = 30.0;      // sim-seconds per trail point

  const int TIME_WARPS[] = {1, 10, 100, 1000, 10000};
  const int NUM_WARPS    = 5;
}

// ---------------------------------------------------------------------------
// Space camera
//
// Camera-relative rendering: all world positions are expressed as
//   (worldPos_double - cameraPos_double), then cast to float.
// This eliminates the ~0.7 m float precision error at LEO distances that
// causes the satellite geometry to collapse when zoomed in.
// ---------------------------------------------------------------------------
struct SpaceCamera
{
  double     distance = Config::CAM_DIST_INIT;
  float      yaw      = 45.0f;
  float      pitch    = 25.0f;
  glm::dvec3 target{0, 0, 0};

  // Computed each frame by applyToCamera(); used by callers for camRel().
  glm::dvec3 cameraPos{0, 0, 0};

  void handleInput(GUI &gui, glm::vec2 mouseDelta, glm::vec2 scrollDelta)
  {
    if (gui.isMouseButtonPressed(GLFW_MOUSE_BUTTON_LEFT))
    {
      yaw   -= mouseDelta.x * 0.3f;
      pitch  = glm::clamp(pitch + mouseDelta.y * 0.3f, -89.0f, 89.0f);
    }
    if (scrollDelta.y != 0.0f)
    {
      distance *= std::pow(0.85, (double)scrollDelta.y);
      distance  = glm::clamp(distance, Config::CAM_DIST_MIN, Config::CAM_DIST_MAX);
    }
  }

  // Sets the VGL camera for camera-relative rendering:
  //   - Camera is placed at world origin (0,0,0) in render space.
  //   - The target is expressed relative to the camera in double, then float.
  // Callers must use camRel() for all draw-call positions.
  void applyToCamera(Camera &cam)
  {
    float yr = glm::radians(yaw);
    float pr = glm::radians(pitch);
    glm::dvec3 offset{
        distance * std::cos((double)pr) * std::cos((double)yr),
        distance * std::cos((double)pr) * std::sin((double)yr),
        distance * std::sin((double)pr)};

    cameraPos = target + offset;

    // Camera sits at render-space origin; everything else is relative to it.
    cam.position = {0.0f, 0.0f, 0.0f};
    cam.target   = glm::vec3(target - cameraPos); // exact double subtraction
    cam.setUp({0, 0, 1});
    cam.setClipPlanes(Config::NEAR_PLANE, Config::FAR_PLANE);
  }

  // Convert an ECI world position to camera-relative render space.
  // Subtraction in double eliminates float precision loss at large coordinates.
  glm::vec3 camRel(glm::dvec3 worldPos) const
  {
    return glm::vec3(worldPos - cameraPos);
  }
};

// ---------------------------------------------------------------------------
// Draw helpers — all positions are camera-relative
// ---------------------------------------------------------------------------
static void drawEarth(GUI &gui, glm::vec3 earthPos)
{
  gui.drawSphere(earthPos, Config::EARTH_RADIUS, {0.15f, 0.35f, 0.65f});

  // Atmosphere haze
  gui.setLighting(false);
  gui.drawSphere(earthPos, Config::EARTH_RADIUS * 1.015f, {0.3f, 0.55f, 0.9f});
  gui.setLighting(true);
}

static void drawSatellite(GUI &gui, glm::vec3 renderPos, const RigidBody &body)
{
  gui.drawBox(renderPos, body.size, body.orientation, {1.0f, 1.0f, 0.0f});

  float L = 1.0f;
  glm::vec3 bX = body.orientation * glm::vec3(1, 0, 0);
  glm::vec3 bY = body.orientation * glm::vec3(0, 1, 0);
  glm::vec3 bZ = body.orientation * glm::vec3(0, 0, 1);
  gui.drawArrow(renderPos, renderPos + bX * L, {1, 0, 0});
  gui.drawArrow(renderPos, renderPos + bY * L, {0, 1, 0});
  gui.drawArrow(renderPos, renderPos + bZ * L, {0, 0, 1});
}

static void drawOrbitTrail(GUI &gui, const std::deque<glm::dvec3> &trail,
                           const SpaceCamera &cam)
{
  if (trail.size() < 2) return;
  gui.setLighting(false);
  for (size_t i = 0; i + 1 < trail.size(); i++)
  {
    float t   = (float)i / (float)(trail.size() - 1);
    glm::vec3 col = glm::mix(glm::vec3(0.2f, 0.2f, 0.5f),
                              glm::vec3(0.4f, 0.8f, 1.0f), t);
    gui.drawLine(cam.camRel(trail[i]), cam.camRel(trail[i + 1]), col);
  }
  gui.setLighting(true);
}

// ---------------------------------------------------------------------------
// Telemetry HUD via window title
// ---------------------------------------------------------------------------
static void updateTitle(GLFWwindow *win, int warpIdx, double altM,
                        double periodS, float attErrDeg)
{
  char buf[256];
  std::snprintf(buf, sizeof(buf),
                "Orbital CubeSat  |  Warp: %dx  |  Alt: %.1f km  |  "
                "T: %.1f min  |  Att err: %.1f deg",
                Config::TIME_WARPS[warpIdx],
                altM / 1000.0,
                periodS / 60.0,
                attErrDeg);
  glfwSetWindowTitle(win, buf);
}

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------
int main()
{
  GUI gui(1280, 720, "Orbital CubeSat");
  gui.camera.setFOV(45.0f);
  gui.setLightDirection({1.0f, 0.5f, 0.3f});
  gui.setLogDepth(Config::FAR_PLANE);

  // ---- Universe setup ----
  Universe universe;

  // Earth as the fixed reference body (ECI frame)
  const CelestialBody &earth = universe.addCelestial(
      "Earth", Config::EARTH_MASS, Config::EARTH_RADIUS,
      {0, 0, 0}, {0, 0, 0}, /*fixed=*/true);

  // Satellite (Universe owns the RigidBody; Satellite attaches ADCS + wheels)
  Satellite sat(&universe);

  // Place satellite in a circular LEO orbit
  auto [initPos, initVel] = Universe::circularOrbit(
      earth, Config::ALT_M, Config::INC_RAD, Config::RAAN_RAD);
  universe.setOrbitalState(&sat.getBody(), initPos, initVel);

  // Attitude: nadir-pointing (body +Z toward Earth center)
  sat.adcs.target = {0, 0, 0};
  sat.adcs.resetController();

  // ---- Camera ----
  SpaceCamera cam;
  cam.target = universe.getOrbitalPosition(&sat.getBody());

  // ---- Orbit trail ----
  std::deque<glm::dvec3> trail;
  double trailAccum = 0.0;

  int  warpIdx   = 0;
  bool followSat = true;

  glm::vec2 lastMousePos = gui.getMousePosition();
  float     lastTime     = (float)glfwGetTime();

  while (!gui.shouldClose())
  {
    float now = (float)glfwGetTime();
    float dt  = glm::clamp(now - lastTime, 0.0f, 0.05f);
    lastTime  = now;

    // =================== INPUT ===================
    glm::vec2 mousePos   = gui.getMousePosition();
    glm::vec2 mouseDelta = mousePos - lastMousePos;
    lastMousePos         = mousePos;

    if (gui.isKeyJustPressed(GLFW_KEY_EQUAL) || gui.isKeyJustPressed(GLFW_KEY_KP_ADD))
      warpIdx = std::min(warpIdx + 1, Config::NUM_WARPS - 1);
    if (gui.isKeyJustPressed(GLFW_KEY_MINUS) || gui.isKeyJustPressed(GLFW_KEY_KP_SUBTRACT))
      warpIdx = std::max(warpIdx - 1, 0);

    if (gui.isKeyJustPressed(GLFW_KEY_G)) followSat = false;
    if (gui.isKeyJustPressed(GLFW_KEY_T)) followSat = true;

    cam.handleInput(gui, mouseDelta, gui.getScrollDelta());

    // =================== ADCS (real-time guidance) ===================
    sat.adcs.target = {0, 0, 0}; // nadir = toward Earth center
    sat.update(dt);

    // =================== PHYSICS (orbital + attitude) ===================
    universe.step(dt, Config::TIME_WARPS[warpIdx]);

    // =================== CAMERA ===================
    glm::dvec3 satPos = universe.getOrbitalPosition(&sat.getBody());

    if (followSat)
      cam.target = satPos;
    else
      cam.target = {0, 0, 0};

    cam.applyToCamera(gui.camera);

    // =================== TRAIL SAMPLING ===================
    double warpedDt = (double)dt * Config::TIME_WARPS[warpIdx];
    trailAccum += warpedDt;
    if (trailAccum >= Config::TRAIL_INTERVAL)
    {
      trail.push_back(satPos);
      if ((int)trail.size() > Config::TRAIL_POINTS)
        trail.pop_front();
      trailAccum = 0.0;
    }

    // =================== HUD ===================
    {
      const RigidBody &body = sat.getBody();
      glm::vec3 bZ    = body.orientation * glm::vec3(0, 0, 1);
      glm::vec3 nadir = -glm::normalize(body.position);
      float attErrDeg = glm::degrees(
          std::acos(glm::clamp(glm::dot(bZ, nadir), -1.0f, 1.0f)));
      updateTitle(gui.getWindow(), warpIdx,
                  Universe::altitude(earth, satPos),
                  Universe::orbitalPeriod(earth, satPos,
                      universe.getOrbitalVelocity(&sat.getBody())),
                  attErrDeg);
    }

    // =================== DRAW ===================
    gui.beginFrame();
    drawEarth(gui, cam.camRel({0, 0, 0}));
    drawSatellite(gui, cam.camRel(satPos), sat.getBody());
    drawOrbitTrail(gui, trail, cam);
    gui.endFrame();
  }
  return 0;
}
