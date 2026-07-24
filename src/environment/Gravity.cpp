#include <rigidbody/environment/Gravity.h>
#include <rigidbody/RigidBody.h>

void Gravity::apply(RigidBody &body, float /*dt*/)
{
  body.applyForce(acceleration * body.mass);
}
