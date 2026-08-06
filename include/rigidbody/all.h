#pragma once

// rigidbody — a lightweight rigid-body physics engine.
//
// No rendering or scenario dependency: define bodies, attach actuators
// (ForceGenerators) and sensors to them, connect them with constraints, and
// step a PhysicsWorld. Actuators/sensors model physical hardware (a wheel,
// an IMU) with realistic limits and noise. The world itself starts with no
// physical effects at all -- not even gravity -- so environment/ holds
// passive, uncommanded forces (Gravity, Drag) a scenario can opt into via
// PhysicsWorld::addGlobalForceGenerator() or RigidBody::addForceGenerator(),
// as distinct from actuators/, which are commanded hardware a scenario's
// own flight software drives. State estimation and flight software are
// left entirely to the scenario that uses this library.

#include <rigidbody/RigidBody.h>
#include <rigidbody/PhysicsWorld.h>
#include <rigidbody/Constraint.h>
#include <rigidbody/ForceGenerator.h>
#include <rigidbody/actuators/ReactionWheel.h>
#include <rigidbody/actuators/Thruster.h>
#include <rigidbody/actuators/Magnetorquer.h>
#include <rigidbody/environment/Gravity.h>
#include <rigidbody/environment/Drag.h>
#include <rigidbody/environment/MagneticField.h>
#include <rigidbody/sensors/IMU.h>
#include <rigidbody/sensors/Magnetometer.h>
#include <rigidbody/sensors/StarTracker.h>
#include <rigidbody/sensors/SunSensor.h>
