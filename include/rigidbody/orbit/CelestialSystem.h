#pragma once
#include <rigidbody/orbit/CelestialBody.h>
#include <vector>
#include <memory>

// Owns a tree of CelestialBody -- the one place "where is body X right
// now" is answered, so gravity/eclipse/field queries can read a live
// position instead of each hardcoding a specific body (the way
// ThirdBodyGravity/EclipseModel/SunModel/MoonModel do today).
class CelestialSystem
{
public:
  // Ownership stays with the system; the returned pointer is stable for
  // the system's lifetime (bodies are never reallocated/moved).
  CelestialBody *addBody(const std::string &name, const CelestialBodyParams &params, CelestialBody *parent = nullptr);

  // Advances every numerically-propagated body's own orbit by dt seconds;
  // analytic bodies have nothing to advance (their position is a pure
  // function of jd, evaluated on demand by absolutePosition()).
  void step(double dt);

  // This body's position relative to the system origin (root), resolved
  // by summing its parent-relative position up through every ancestor.
  glm::dvec3 absolutePosition(const CelestialBody *body, double jd) const;

  // The system's light source, for eclipse/SRP-style queries -- nullptr if
  // this system has no star (e.g. a purely local/flat-world scenario).
  const CelestialBody *starBody = nullptr;

private:
  std::vector<std::unique_ptr<CelestialBody>> bodies_;
};
