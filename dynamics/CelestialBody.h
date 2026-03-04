#pragma once
#include <string>
#include <glm/glm.hpp>

// A massive body that contributes gravitational acceleration to all other bodies.
// Position and velocity are in the simulation's fixed inertial frame.
// If fixed == true the body is treated as an immovable gravitational source
// (suitable for the primary body when using a body-centered reference frame,
// e.g. Earth in an ECI scenario, or Mars in an MCI scenario).
struct CelestialBody
{
  std::string name;
  double      mass   = 0.0; // kg
  double      radius = 0.0; // m  (for rendering and surface-proximity checks)
  glm::dvec3  position{0.0};
  glm::dvec3  velocity{0.0};
  bool        fixed = true; // if true, never integrated
};
