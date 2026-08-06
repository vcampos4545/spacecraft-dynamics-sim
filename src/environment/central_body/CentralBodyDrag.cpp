#include <rigidbody/environment/central_body/CentralBodyDrag.h>
#include <rigidbody/RigidBody.h>
#include <algorithm>
#include <cmath>

CentralBodyDrag::CentralBodyDrag(float frontalAreaM2) : frontalAreaM2(frontalAreaM2)
{
}

void CentralBodyDrag::apply(RigidBody &body, float /*dt*/)
{
  glm::vec3 v = body.velocity;
  float speed = glm::length(v);
  if (speed < 1e-4f)
    return; // avoid a divide-by-zero normalize; negligible drag at rest anyway

  float distFromCenter = glm::length(body.position - centralBodyPositionWorld);
  float altitude = std::max(0.0f, distFromCenter - planetRadiusM);
  float density = seaLevelDensityKgM3 * std::exp(-altitude / scaleHeightM);

  float dragMag = 0.5f * density * dragCoefficient * frontalAreaM2 * speed * speed;
  glm::vec3 dragForce = -dragMag * (v / speed);

  body.applyForce(dragForce);
}
