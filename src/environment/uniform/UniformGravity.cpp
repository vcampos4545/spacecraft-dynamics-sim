#include <rigidbody/environment/uniform/UniformGravity.h>
#include <rigidbody/RigidBody.h>

void UniformGravity::apply(RigidBody &body, float /*dt*/)
{
  body.applyForce(acceleration * body.mass);
}
