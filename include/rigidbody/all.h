#pragma once

// rigidbody — a lightweight rigid-body physics engine.
//
// No rendering or scenario dependency: define bodies, attach actuators
// (ForceGenerators) and sensors to them, connect them with constraints, and
// step a PhysicsWorld. Actuators/sensors model physical hardware (a wheel,
// an IMU) with realistic limits and noise. The world itself starts with no
// physical effects at all -- not even gravity -- so environment/ holds
// passive, uncommanded forces a scenario can opt into via
// PhysicsWorld::addGlobalForceGenerator() or RigidBody::addForceGenerator(),
// as distinct from actuators/, which are commanded hardware a scenario's
// own flight software drives. environment/uniform/ holds spatially-constant
// or self-contained kinematic models (gravity that's the same everywhere,
// a fixed-altitude atmosphere, a magnetic field driven by a fake kinematic
// orbit phase); environment/central_body/ holds the position-dependent
// counterparts (inverse-square gravity, altitude as true distance from a
// central body, a field sampled at a real position) for scenarios that
// have one -- see orbit/ below for how to get one. State estimation and
// flight software are left entirely to the scenario that uses this
// library.

#include <rigidbody/RigidBody.h>
#include <rigidbody/PhysicsWorld.h>
#include <rigidbody/Constraint.h>
#include <rigidbody/ForceGenerator.h>
#include <rigidbody/actuators/ReactionWheel.h>
#include <rigidbody/actuators/Thruster.h>
#include <rigidbody/actuators/Magnetorquer.h>
#include <rigidbody/environment/uniform/UniformGravity.h>
#include <rigidbody/environment/uniform/UniformDrag.h>
#include <rigidbody/environment/uniform/UniformMagneticField.h>
#include <rigidbody/environment/central_body/CentralBodyGravity.h>
#include <rigidbody/environment/central_body/CentralBodyDrag.h>
#include <rigidbody/environment/central_body/CentralBodyMagneticField.h>
#include <rigidbody/orbit/OrbitState.h>
#include <rigidbody/orbit/OrbitalElements.h>
#include <rigidbody/orbit/OrbitForceModel.h>
#include <rigidbody/orbit/OrbitPropagator.h>
#include <rigidbody/orbit/OrbitFrames.h>
#include <rigidbody/orbit/OrbitTime.h>
#include <rigidbody/orbit/SunModel.h>
#include <rigidbody/orbit/EclipseModel.h>
#include <rigidbody/sensors/IMU.h>
#include <rigidbody/sensors/Magnetometer.h>
#include <rigidbody/sensors/StarTracker.h>
#include <rigidbody/sensors/SunSensor.h>
#include <rigidbody/power/SolarPanel.h>
#include <rigidbody/power/Battery.h>
