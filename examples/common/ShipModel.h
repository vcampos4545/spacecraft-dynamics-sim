#pragma once
#include <vgl/vgl.h>
#include <algorithm>

// Fits a loaded OBJ model's bounding box onto a cylinder of the given
// height and diameter, without needing to know the model's authored units
// or pivot in advance:
//  - the mesh's longest bounding-box axis is treated as its nose/base axis
//    and mapped onto height (matches Starship.cpp's local +Z = nose; for
//    models/starship.obj the nose sits at the min end of that axis, engines
//    /legs at the max end);
//  - of the remaining two axes, the SHORTER one is treated as the true
//    body diameter (the taller one is inflated by non-radially-symmetric
//    geometry, e.g. fold-out landing legs) and both are scaled by the same
//    factor so the cross-section stays circular instead of being squashed
//    into an ellipse;
//  - the bounding-box centroid is recentered onto the body's origin, since
//    RigidBody::position is the cylinder's center but this model's own
//    origin sits at its base.
//
// Shared by any scenario that draws a Starship from its real model (see
// examples/starship/launch.cpp and examples/starship_landing/landing.cpp)
// rather than duplicated -- unlike most scenario code, this is genuine
// rendering-geometry logic with no FSW/physics content, so it belongs here
// the same way World.h's scene dressing does.
struct ModelFit
{
  glm::vec3 scale{0.0f};
  glm::quat align{1.0f, 0.0f, 0.0f, 0.0f}; // mesh-local axes -> body-local axes
  glm::vec3 pivotOffsetLocal{0.0f};        // mesh-local bounding-box centroid
};

inline ModelFit fitModelToCylinder(const OBJMesh &mesh, float heightM, float diameterM)
{
  glm::vec3 lo = mesh.getBoundsMin();
  glm::vec3 hi = mesh.getBoundsMax();
  glm::vec3 extent = hi - lo;

  int up = 0;
  if (extent.y > extent[up]) up = 1;
  if (extent.z > extent[up]) up = 2;
  int a = (up + 1) % 3;
  int b = (up + 2) % 3;

  ModelFit fit;
  fit.scale[up] = heightM / extent[up];
  float crossScale = diameterM / std::min(extent[a], extent[b]);
  fit.scale[a] = crossScale;
  fit.scale[b] = crossScale;

  // The longest axis is reliably the nose/base axis, but which *end* is
  // the nose isn't derivable from the bounding box alone -- confirmed by
  // running it that models/starship.obj has its nose at the min end (flip
  // this sign if a differently-authored model points the other way).
  glm::vec3 meshUp(0.0f);
  meshUp[up] = -1.0f;
  fit.align = glm::rotation(meshUp, glm::vec3(0, 0, 1));

  fit.pivotOffsetLocal = 0.5f * (lo + hi);
  return fit;
}

inline void drawShipModel(GUI &gui, OBJMesh &mesh, const ModelFit &fit, RigidBody *stage)
{
  glm::quat rotation = stage->orientation * fit.align;
  glm::vec3 centroidOffset = rotation * (fit.scale * fit.pivotOffsetLocal);
  gui.drawOBJMesh(mesh, stage->position - centroidOffset, fit.scale, rotation);
}
