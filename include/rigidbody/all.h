#pragma once

// rigidbody — a lightweight rigid-body physics engine.
//
// No rendering or scenario dependency: define bodies, attach actuators
// (ForceGenerators) to them, connect them with constraints, and step a
// PhysicsWorld. Gravity, sensing, and flight software are left entirely to
// the scenario that uses this library.

#include <rigidbody/RigidBody.h>
#include <rigidbody/PhysicsWorld.h>
#include <rigidbody/Constraint.h>
#include <rigidbody/ForceGenerator.h>
#include <rigidbody/actuators/ReactionWheel.h>
#include <rigidbody/actuators/Thruster.h>
