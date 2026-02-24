#include "Satellite.h"
#include "ReactionWheel.h"
#include "PhysicsWorld.h"

Satellite::Satellite(PhysicsWorld *world)
{
  // Create 1U cubesat body
  this->body = world->createBody(
      RigidBodyShape::BOX,
      glm::vec3(0.1f, 0.1f, 0.1f), // 10 x 10 x 10 cm
      1.33f);                       // max mass of 1U cubesat (kg)

  // Start satellite floating above the ground
  this->body->position.z = this->body->size.y * 3;

  // Create 3 reaction wheels along X, Y, Z body axes
  // TODO: Switch to 4-wheel pyramid configuration
  glm::vec3 axes[3] = {
      glm::vec3(1.0f, 0.0f, 0.0f),
      glm::vec3(0.0f, 1.0f, 0.0f),
      glm::vec3(0.0f, 0.0f, 1.0f),
  };

  for (int i = 0; i < 3; ++i)
  {
    auto tmp = std::make_unique<ReactionWheel>(
        glm::vec3(0.0f, 0.0f, 0.0f),           // mount at body center
        axes[i],
        0.001f,                                  // max torque (Nm)
        6000.0f * (2.0f * M_PI / 60.0f),        // 6000 RPM max
        1e-6f);                                  // wheel inertia (kg*m^2)

    // Store a raw (non-owning) pointer before transferring ownership
    wheels.push_back(tmp.get());
    // RigidBody takes ownership via the ForceGenerator interface
    this->body->addForceGenerator(std::move(tmp));
  }

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
