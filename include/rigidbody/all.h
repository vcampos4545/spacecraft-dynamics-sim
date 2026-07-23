#pragma once

// rigidbody — a lightweight rigid-body physics engine.
//
// No rendering or scenario dependency: define bodies, attach actuators
// (ForceGenerators) and sensors to them, connect them with constraints, and
// step a PhysicsWorld. Actuators/sensors model physical hardware (a wheel,
// an IMU) with realistic limits and noise; gravity, state estimation, and
// flight software are left entirely to the scenario that uses this library.

#include <rigidbody/RigidBody.h>
#include <rigidbody/PhysicsWorld.h>
#include <rigidbody/Constraint.h>
#include <rigidbody/ForceGenerator.h>
#include <rigidbody/actuators/ReactionWheel.h>
#include <rigidbody/actuators/Thruster.h>
#include <rigidbody/sensors/IMU.h>
