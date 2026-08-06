#include <rigidbody/environment/central_body/CentralBodyGravity.h>
#include <rigidbody/RigidBody.h>
#include <cmath>

void CentralBodyGravity::apply(RigidBody &body, float /*dt*/)
{
  glm::vec3 r = body.position - centralBodyPositionWorld;
  float rMagSq = glm::dot(r, r);
  if (rMagSq < 1.0f)
    return; // at/inside the central body's center -- undefined direction, nothing sane to apply

  float rMag = std::sqrt(rMagSq);
  glm::vec3 rHat = r / rMag;

  float accelMag = mu / rMagSq;
  body.applyForce(-accelMag * rHat * body.mass);
}
