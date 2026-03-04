#include "Satellite.h"
#include "ReactionWheel.h"
#include "PhysicsWorld.h"
#include "Universe.h"

// ---------------------------------------------------------------------------
// Helper: attach 3-axis reaction wheels to a body (shared setup)
// ---------------------------------------------------------------------------
static void attachReactionWheels(RigidBody *body,
                                 std::vector<ReactionWheel *> &wheels)
{
  glm::vec3 axes[3] = {
      {1.0f, 0.0f, 0.0f},
      {0.0f, 1.0f, 0.0f},
      {0.0f, 0.0f, 1.0f},
  };

  for (int i = 0; i < 3; ++i)
  {
    auto tmp = std::make_unique<ReactionWheel>(
        glm::vec3(0.0f, 0.0f, 0.0f), // mount at body center
        axes[i],
        0.001f,                          // max torque (Nm)
        6000.0f * (2.0f * M_PI / 60.0f), // 6000 RPM max
        1e-6f);                          // wheel inertia (kg·m²)

    wheels.push_back(tmp.get());
    body->addForceGenerator(std::move(tmp));
  }
}

Satellite::Satellite(PhysicsWorld *world)
{
  // Create 1U cubesat body
  this->body = world->createBody(
      RigidBodyShape::BOX,
      glm::vec3(0.1f, 0.1f, 0.1f), // 10 x 10 x 10 cm
      1.33f);                      // max mass of 1U cubesat (kg)

  // Start satellite floating above the ground
  this->body->position.z = this->body->size.y * 3;

  attachReactionWheels(this->body, wheels);
  adcs = ADCS(this->body, wheels);
}

Satellite::Satellite(Universe *universe)
{
  // Create 1U cubesat body in Universe (Universe owns it)
  this->body = universe->addBody(
      RigidBodyShape::BOX,
      glm::vec3(0.1f, 0.1f, 0.1f), // 10 x 10 x 10 cm
      1.33f);                      // max mass of 1U cubesat (kg)

  attachReactionWheels(this->body, wheels);
  adcs = ADCS(this->body, wheels);
}

void Satellite::update(float dt)
{
  adcsTimer += dt;

  if (adcsTimer > 0.05f) // 20 Hz
  {
    adcs.run(adcsTimer);
    adcsTimer = 0;
  }
}
