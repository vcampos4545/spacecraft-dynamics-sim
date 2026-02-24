#include <vgl/vgl.h>
#include "PhysicsWorld.h"
#include "spacecraft/Satellite.h"

namespace Config
{
  // Camera
  constexpr float CAMERA_NEAR = 0.1f;  // 10 cm
  constexpr float CAMERA_FAR = 100.0f; // 100 m
  constexpr float CAMERA_FOV = 45.0f;
  constexpr float CAMERA_INITIAL_DISTANCE = 1.0f; // 1 m
  constexpr float CAMERA_MIN_DISTANCE = 0.1f;
  constexpr float CAMERA_MAX_DISTANCE = 100.0f;
  constexpr float ZOOM_SENSITIVITY = 1.0f;
  constexpr float PAN_SENSITIVITY = 0.2f;

  // Grid
  constexpr float GRID_SIZE = 5.0f;
  constexpr float GRID_STEP = 1.0f;
}

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
  // Draw sat (blue when controller active, red when manual)
  gui.drawBox(sat->position, sat->size, sat->orientation, {1.0f, 1.0f, 0});

  // Draw body axes (body +Z axis in world space)
  float arrowLength = 0.25f;
  glm::vec3 bodyXAxis = sat->orientation * glm::vec3(1, 0, 0);
  glm::vec3 bodyYAxis = sat->orientation * glm::vec3(0, 1, 0);
  glm::vec3 bodyZAxis = sat->orientation * glm::vec3(0, 0, 1);
  glm::vec3 arrowStart = sat->position;
  glm::vec3 arrowXEnd = sat->position + bodyXAxis * arrowLength;
  glm::vec3 arrowYEnd = sat->position + bodyYAxis * arrowLength;
  glm::vec3 arrowZEnd = sat->position + bodyZAxis * arrowLength;
  gui.drawArrow(arrowStart, arrowXEnd, glm::vec3(1, 0, 0));
  gui.drawArrow(arrowStart, arrowYEnd, glm::vec3(0, 1, 0));
  gui.drawArrow(arrowStart, arrowZEnd, glm::vec3(0, 0, 1));
}

void drawReactionWheels(GUI &gui, const std::vector<ReactionWheel *> &reactionWheels, RigidBody *sat)
{
  glm::vec3 totalAngular;
  float axisLength = 0.20f;
  for (auto &wheel : reactionWheels)
  {
    glm::vec3 worldPos = wheel->getWorldMountPosition(*sat);
    glm::vec3 worldAxis = wheel->getWorldSpinAxis(*sat);

    // Color based on saturation (green = ok, yellow = warning, red = saturated)
    float satRatio = wheel->getSaturationRatio();
    float absSatRatio = abs(satRatio);
    glm::vec3 axisColor;
    if (absSatRatio < 0.5f)
      axisColor = glm::vec3(0, 1, 0); // green
    else if (absSatRatio < 0.9f)
      axisColor = glm::vec3(1, 1, 0); // yellow
    else
      axisColor = glm::vec3(1, 0, 0); // red (saturated)

    // Draw anuglar velocity vector

    gui.drawArrow(worldPos,
                  worldPos + worldAxis * axisLength * satRatio,
                  axisColor);
    totalAngular += worldAxis * satRatio;
  }
  gui.drawArrow(sat->position, sat->position + totalAngular * axisLength, {1.0f, 0.65f, 0});
}

int main()
{
  // Setup gui
  GUI gui(800, 600, "Rigid Body Simulation");
  // Setup camera
  gui.camera
      .setUp({0, 0, 1}) // Z+ up
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

  // Create physics world
  PhysicsWorld world;
  Satellite sat(&world);

  float lastTime = glfwGetTime();
  while (!gui.shouldClose())
  {
    float time = glfwGetTime();
    float dt = time - lastTime;
    lastTime = time;

    // =================== UPDATE GUI ===================
    // Handle camera input
    glm::vec2 mousePos = gui.getMousePosition();
    glm::vec2 mouseDelta = mousePos - lastMousePos;
    lastMousePos = mousePos;

    // if (gui.isKeyJustPressed(GLFW_KEY_SPACE))
    // {
    //   printf("mousePos: %f, %f\n", mousePos.x, mousePos.y);
    //   float windowWidth = gui.getWindowWidth();
    //   float windowHeight = gui.getWindowHeight();
    //   float x = (2.0f * mousePos.x) / windowWidth - 1.0f;
    //   float y = 1.0f - (2.0f * mousePos.y) / windowHeight;

    //   glm::vec3 cameraPosition = gui.camera.position;
    //   glm::mat4 projectionMatrix = gui.camera.getProjectionMatrix(windowWidth / windowHeight);
    //   glm::mat4 viewMatrix = gui.camera.getViewMatrix();

    //   glm::vec4 rayClip = glm::vec4(x, y, -1.0f, 1.0f);
    //   glm::vec4 rayView = glm::inverse(projectionMatrix) * rayClip;

    //   // Turn into direction
    //   rayView = glm::vec4(rayView.x, rayView.y, -1.0f, 0.0f);
    //   glm::vec3 rayWorld = glm::normalize(glm::vec3(glm::inverse(viewMatrix) * rayView));
    //   glm::vec3 rayDirection = rayWorld;

    //   // TODO: Check intersection (Spherical)
    // }

    orbit.handleInput(gui, mouseDelta, gui.getScrollDelta());
    orbit.applyToCamera(gui.camera);

    // =================== UPDATE SIMULATION ===================

    sat.update(dt);
    world.step(dt); // physics

    // =================== DRAW ===================

    gui.beginFrame();
    drawGrid(gui);
    drawSatellite(gui, &sat.getBody());
    drawReactionWheels(gui, sat.getWheels(), &sat.getBody());
    gui.drawSphere(sat.adcs.target, 0.05f, {0, 1.0f, 0}); // adcs target
    gui.endFrame();
  }
  return 0;
}