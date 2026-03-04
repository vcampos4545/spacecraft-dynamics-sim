#include <vgl/vgl.h>
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/quaternion.hpp>
#include "PhysicsWorld.h"
#include "Constraint.h"
#include <cmath>
#include <vector>

/* CALDER MOBILE NOTES
 *
 * 5-level hierarchical Calder mobile.  Each level has:
 *   - A horizontal bar (BOX)
 *   - A sphere hanging from the right end
 *   - (last level only) a sphere hanging from the left end too
 *
 * Bars are connected via distance constraints (strings).
 * The top bar is pinned to a world anchor.
 * All strings are unilateral (max-distance only, like real string).
 *
 * Coordinate system: Z-up (C++ convention).
 */

// -----------------------------------------------------------------------
// Mobile parameters (matching the JS reference scene)
// -----------------------------------------------------------------------
static const int NUM_LEVELS = 5;
static const float BAR_LENGTH = 0.9f;                        // m
static const float THICKNESS = 0.04f;                        // m (bar cross-section)
static const float HEIGHT = 0.3f;                            // m vertical spacing between levels
static const float BASE_RADIUS = 0.08f;                      // m radius of smallest sphere
static const float DISTANCE = 0.5f * BAR_LENGTH - THICKNESS; // 0.41 m

// -----------------------------------------------------------------------
// Helpers
// -----------------------------------------------------------------------
void drawGrid(GUI &gui, float size, float step)
{
  const glm::vec3 gridColor{0.35f, 0.35f, 0.35f};
  const glm::vec3 axisX{0.7f, 0.2f, 0.2f};
  const glm::vec3 axisY{0.2f, 0.7f, 0.2f};

  for (float i = -size; i <= size; i += step)
  {
    gui.drawLine({i, -size, 0}, {i, size, 0}, i == 0.0f ? axisX : gridColor);
    gui.drawLine({-size, i, 0}, {size, i, 0}, i == 0.0f ? axisY : gridColor);
  }
}

// -----------------------------------------------------------------------
// Main
// -----------------------------------------------------------------------
int main()
{
  // ---- Window & camera ------------------------------------------------
  GUI gui(800, 600, "Calder Mobile");
  gui.setLighting(false);

  gui.camera
      .setUp({0, 0, 1})
      .setClipPlanes(0.1f, 500.0f)
      .setFOV(45.0f);

  // Look at the centre of the mobile (roughly mid-height of the chain)
  OrbitalCamera orbit(4.0f, 45.0f, 20.0f, {-1.0f, 0.0f, 1.8f});
  orbit.setMinDistance(0.5f)
      .setMaxDistance(50.0f)
      .setZoomSensitivity(0.5f)
      .setPanSensitivity(0.1f);

  glm::vec2 lastMousePos = gui.getMousePosition();

  // ---- Physics world --------------------------------------------------
  PhysicsWorld world;
  world.gravity = glm::vec3(0.0f, 0.0f, -9.81f);

  // Collect all string constraints for visualization
  std::vector<DistanceConstraint *> strings;

  // ---- Build mobile hierarchy -----------------------------------------
  RigidBody *prevBar = nullptr;
  glm::vec3 barPos(0.0f, 0.0f, 2.5f); // top bar starts at Z = 2.5 m

  for (int level = 0; level < NUM_LEVELS; ++level)
  {
    // -- Horizontal bar (BOX: length x thickness x thickness) -----------
    RigidBody *bar = world.createBody(
        RigidBodyShape::BOX,
        glm::vec3(BAR_LENGTH, THICKNESS, THICKNESS),
        0.5f); // kg
    bar->position = barPos;

    if (prevBar == nullptr)
    {
      // Pin top-centre of first bar to a fixed world point (ball-socket).
      // restLength = 0  →  locks the attachment point to the world.
      glm::vec3 anchor = barPos + glm::vec3(0.0f, 0.0f, THICKNESS * 0.5f);
      strings.push_back(
          world.addDistanceConstraint(bar, nullptr, anchor, anchor, 0.0f, false));
    }
    else
    {
      // String from left end of prevBar down to top-centre of this bar.
      glm::vec3 p0 = prevBar->position + glm::vec3(-DISTANCE, 0.0f, 0.0f);
      glm::vec3 p1 = barPos + glm::vec3(0.0f, 0.0f, THICKNESS * 0.5f);
      float restLen = HEIGHT - THICKNESS; // 0.26 m
      strings.push_back(
          world.addDistanceConstraint(prevBar, bar, p0, p1, restLen, true));
    }

    // -- Sphere radius: volume doubles going up one level ---------------
    // level 0 (top):  baseRadius * 2^(4/3)
    // level 4 (bot):  baseRadius
    float sphereRadius = BASE_RADIUS *
                         std::pow(2.0f, float(NUM_LEVELS - 1 - level) / 3.0f);
    float sphereMass = 1.0f * std::pow(sphereRadius / BASE_RADIUS, 3.0f); // kg

    // -- Right sphere ---------------------------------------------------
    glm::vec3 spherePosR = barPos +
                           glm::vec3(DISTANCE, 0.0f, -(HEIGHT - sphereRadius));

    RigidBody *sphereR = world.createBody(
        RigidBodyShape::SPHERE,
        glm::vec3(sphereRadius, 0.0f, 0.0f),
        sphereMass);
    sphereR->position = spherePosR;

    // String from right end of bar to sphere centre
    glm::vec3 barRightEnd = barPos + glm::vec3(DISTANCE, 0.0f, 0.0f);
    strings.push_back(
        world.addDistanceConstraint(bar, sphereR,
                                    barRightEnd, spherePosR,
                                    HEIGHT - sphereRadius, true));

    // -- Left sphere (last level only) ----------------------------------
    if (level == NUM_LEVELS - 1)
    {
      glm::vec3 spherePosL = barPos +
                             glm::vec3(-DISTANCE, 0.0f, -(HEIGHT - sphereRadius));

      RigidBody *sphereL = world.createBody(
          RigidBodyShape::SPHERE,
          glm::vec3(sphereRadius, 0.0f, 0.0f),
          sphereMass);
      sphereL->position = spherePosL;

      glm::vec3 barLeftEnd = barPos + glm::vec3(-DISTANCE, 0.0f, 0.0f);
      strings.push_back(
          world.addDistanceConstraint(bar, sphereL,
                                      barLeftEnd, spherePosL,
                                      HEIGHT - sphereRadius, true));
    }

    // -- Descend to next level ------------------------------------------
    prevBar = bar;
    barPos.z -= HEIGHT;   // drop one level in Z
    barPos.x -= DISTANCE; // shift left (next bar hangs from this bar's left end)
  }

  // ---- Main loop -------------------------------------------------------
  float lastTime = glfwGetTime();

  while (!gui.shouldClose())
  {
    const float time = glfwGetTime();
    const float dt = time - lastTime;
    lastTime = time;

    // -- Camera input ----------------------------------------------------
    glm::vec2 mousePos = gui.getMousePosition();
    glm::vec2 mouseDelta = mousePos - lastMousePos;
    lastMousePos = mousePos;

    orbit.handleInput(gui, mouseDelta, gui.getScrollDelta());
    orbit.applyToCamera(gui.camera);

    // -- Simulate --------------------------------------------------------
    world.step(dt);

    // -- Draw ------------------------------------------------------------
    gui.beginFrame();

    drawGrid(gui, 3.0f, 0.5f);

    // Strings
    const glm::vec3 stringColor{0.75f, 0.75f, 0.75f};
    for (auto *s : strings)
    {
      auto [pA, pB] = s->getWorldAnchors();
      gui.drawLine(pA, pB, stringColor);
    }

    // Bodies
    const glm::vec3 barColor{0.75f, 0.45f, 0.15f};
    const glm::vec3 sphereColor{0.25f, 0.50f, 0.90f};

    for (const auto &body : world.getBodies())
    {
      if (body->shape == RigidBodyShape::BOX)
        gui.drawBox(body->position, body->size, body->orientation, barColor);
      else if (body->shape == RigidBodyShape::SPHERE)
        gui.drawSphere(body->position, body->size.x, sphereColor);
    }

    gui.endFrame();
  }

  return 0;
}
