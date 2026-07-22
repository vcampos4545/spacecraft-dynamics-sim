#include <vgl/vgl.h>
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/quaternion.hpp>
#include <rigidbody/PhysicsWorld.h>
#include <rigidbody/Constraint.h>
#include <vector>
#include <cmath>
#include <glm/gtc/constants.hpp>

// ---------------------------------------------------------------------------
// A hanging chain: rigid links connected by PointConstraint (ball-socket)
// joints — unlike bars.cpp's DistanceConstraint mobile, each link holds a
// *fixed* separation from its neighbor and rotates completely freely at the
// joint, the way a real chain link does. No flight software here; the chain
// is just released from a swept-back pose and left to swing under gravity.
// ---------------------------------------------------------------------------
namespace Config
{
  constexpr float CAMERA_NEAR = 0.05f;
  constexpr float CAMERA_FAR = 100.0f;
  constexpr float CAMERA_FOV = 45.0f;
  constexpr float CAMERA_INITIAL_DISTANCE = 4.0f;
  constexpr float CAMERA_MIN_DISTANCE = 0.5f;
  constexpr float CAMERA_MAX_DISTANCE = 20.0f;
  constexpr float ZOOM_SENSITIVITY = 1.0f;
  constexpr float PAN_SENSITIVITY = 0.2f;

  constexpr float GRID_SIZE = 5.0f;
  constexpr float GRID_STEP = 1.0f;

  constexpr int NUM_LINKS = 8;
  const glm::vec3 LINK_SIZE{0.06f, 0.06f, 0.25f}; // thin box, long axis = link direction
  constexpr float LINK_MASS = 0.15f;

  constexpr float ANCHOR_Z = 5.0f;      // clear of the ground plane at z = 0
  constexpr float SWEEP_PER_LINK = 0.08f; // initial sideways offset per link (radians-ish lean)
}

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

int main()
{
  GUI gui(800, 600, "Chain (PointConstraint)");
  gui.camera
      .setUp({0, 0, 1})
      .setClipPlanes(Config::CAMERA_NEAR, Config::CAMERA_FAR)
      .setFOV(Config::CAMERA_FOV);

  OrbitalCamera orbit(
      Config::CAMERA_INITIAL_DISTANCE,
      45.0f, 10.0f,
      {0, 0, Config::ANCHOR_Z - 1.0f});
  orbit.setMaxDistance(Config::CAMERA_MAX_DISTANCE)
      .setMinDistance(Config::CAMERA_MIN_DISTANCE)
      .setZoomSensitivity(Config::ZOOM_SENSITIVITY)
      .setPanSensitivity(Config::PAN_SENSITIVITY);

  glm::vec2 lastMousePos = gui.getMousePosition();

  // =================== DEFINE THE CHAIN ===================
  PhysicsWorld world;
  world.gravity = {0.0f, 0.0f, -9.81f};

  RigidBody *anchor = world.createBody(RigidBodyShape::BOX, {0.1f, 0.1f, 0.1f}, 1.0f);
  anchor->invMass = 0.0f;
  anchor->invInertiaTensor = glm::mat3(0.0f);
  anchor->position = {0, 0, Config::ANCHOR_Z};

  std::vector<RigidBody *> links;
  RigidBody *prev = anchor;
  glm::vec3 pivot = anchor->position;

  for (int i = 0; i < Config::NUM_LINKS; ++i)
  {
    RigidBody *link = world.createBody(RigidBodyShape::BOX, Config::LINK_SIZE, Config::LINK_MASS);

    // Swept-back starting pose: each link leans further from vertical, so
    // releasing the chain lets it swing back down through gravity.
    float lean = Config::SWEEP_PER_LINK * (i + 1);
    glm::vec3 dir = glm::normalize(glm::vec3(std::sin(lean), 0.0f, -std::cos(lean)));

    link->position = pivot + dir * (Config::LINK_SIZE.z * 0.5f);
    glm::vec3 fromZ(0, 0, 1);
    glm::vec3 rotAxis = glm::cross(fromZ, dir);
    float sinA = glm::length(rotAxis), cosA = glm::dot(fromZ, dir);
    link->orientation = (sinA > 1e-6f)
                             ? glm::angleAxis(std::atan2(sinA, cosA), glm::normalize(rotAxis))
                             : glm::quat(1, 0, 0, 0);

    world.addPointConstraint(prev, link, pivot);

    pivot = link->position + dir * (Config::LINK_SIZE.z * 0.5f);
    prev = link;
    links.push_back(link);
  }

  float lastTime = glfwGetTime();
  while (!gui.shouldClose())
  {
    float time = glfwGetTime();
    float dt   = time - lastTime;
    lastTime   = time;

    glm::vec2 mousePos   = gui.getMousePosition();
    glm::vec2 mouseDelta = mousePos - lastMousePos;
    lastMousePos         = mousePos;
    orbit.handleInput(gui, mouseDelta, gui.getScrollDelta());
    orbit.applyToCamera(gui.camera);

    world.step(dt);

    gui.beginFrame();
    drawGrid(gui);

    gui.drawSphere(anchor->position, 0.04f, {1, 1, 1});
    const glm::vec3 linkColor{0.7f, 0.55f, 0.2f};
    for (auto *link : links)
      gui.drawBox(link->position, link->size, link->orientation, linkColor);

    gui.endFrame();
  }
  return 0;
}
