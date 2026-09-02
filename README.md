# rigidbody

A lightweight, general-purpose rigid-body physics engine in C++: bodies, constraints, actuators, sensors, and environment forces, with an optional double-precision orbital-mechanics module for anything that needs real orbit-scale accuracy. The physics core has no concept of "spacecraft" — a reaction wheel, a robot-arm joint motor, and a thruster are all just `ForceGenerator`s attached to a body, so the same engine drives spacecraft, launch-vehicle, and general-robotics scenarios. It's meant for writing and testing real controls/flight software against a simulated vehicle instead of hardware.

## What it gives you

- **`RigidBody`** — box/sphere/cylinder/cone shapes, mass & inertia computed from density or mass, GJK+EPA convex collision, ground contact
- **`PhysicsWorld`** — fixed-timestep stepping (120 Hz internally, accumulator-driven), body-body and ground collision resolution, Sequential Impulse constraint solving with Baumgarte stabilization
- **`Constraint`** — five primitives built from shared solver math: `FixedConstraint` (weld, locks all 6 DOF), `PointConstraint` (ball socket, locks translation), `HingeConstraint` (revolute, 1 free rotational DOF with an optional limit + motor), `SliderConstraint` (prismatic, 1 free translational DOF with the same limit/motor pattern), and `DistanceConstraint` (a rope/strut held at a target distance, optionally unilateral like a string). More elaborate joints compose from these — a universal joint is two `HingeConstraint`s sharing a pivot, for instance.
- **Actuators**, as `ForceGenerator`s attached to a body — `ReactionWheel` (torque + saturation + a `healthFactor` fault model for degraded/failed wheels) and `Thruster` (gimbaled, throttled). Neither is spacecraft-specific in the engine itself; attaching a reaction wheel to a non-spacecraft body works exactly the same way.
- **Sensors**, in `include/rigidbody/sensors/` — `IMU` (3-axis gyro + accelerometer modeled after commodity MEMS parts like the MPU6050: per-run turn-on bias, slow bias drift, and measurement noise, not exact ground truth), reporting in the sensor's own body-fixed axes.
- **Environment models**, in `include/rigidbody/environment/` — passive, uncommanded forces/fields a scenario opts into, split into `uniform/` (spatially constant: gravity, drag, magnetic field) and `central_body/` (position-dependent counterparts for orbital/reentry scenarios: real inverse-square gravity, altitude as true distance from a central body, a field sampled at a real position).
- **`orbit/`** — a self-contained, double-precision (`glm::dvec3`/`glm::dquat`) orbital-mechanics module, independent of `RigidBody`/`PhysicsWorld`'s float32 state (a LEO-radius position needs more precision than float32 gives over a mission-duration integration): an `OrbitPropagator` (RK4, pluggable force models — two-body gravity, J2, third-body Sun/Moon perturbation, atmospheric drag, solar radiation pressure), `OrbitalElements` conversions, frame/time utilities (GMST, ECEF↔ECI, geodetic↔ECEF, Julian dates), analytic Sun/Moon ephemerides, eclipse geometry, and a `CelestialSystem` that generalizes all of it into a real multi-body hierarchy (Sun → Earth → Moon, each either propagated or analytic) instead of hardcoding Earth-specific constants into every force model.

## Design principle

The engine never assumes what's driving a body. Your own flight-software or controller code reads body/sensor state and commands actuators each frame; `PhysicsWorld` just integrates whatever forces and torques those actuators produced. That boundary is what let a hardware-abstracted flight-software stack ([Satellite ADCS Simulation](https://github.com/vcampos4545/satellite-adcs-sim)) build on top of this engine without either project needing to know about the other's internals — and it's exactly the same boundary a robot-arm controller or any other actuated rigid body uses.

## Examples

`examples/` has worked scenarios spanning spacecraft, launch vehicles, and general robotics/mechanisms — a CubeSat flown on a 4-wheel reaction-wheel pyramid, a multi-body rocket with gimbaled thrusters and in-flight staging, a robot arm, hinge- and slider-driven mechanisms (a deploying solar panel, a telescoping boom), a constraint-only Calder mobile, and headless correctness checks for the orbital-mechanics module. See each example's own folder for what it demonstrates in detail.

## Dependencies

The library itself needs only:

- CMake ≥ 3.16
- [glm](https://github.com/g-truc/glm) (header-only math)

Building the examples additionally needs glfw and glew, for rendering via [VGL](https://github.com/vcampos4545/VGL):

```bash
# macOS
brew install glm glfw glew

# Ubuntu
sudo apt install libglm-dev libglfw3-dev libglew-dev
```

## Use in your own project

The `rigidbody` CMake target has no rendering dependency — link it from a headless simulation, a test harness, or your own renderer.

### CMake FetchContent (recommended)

```cmake
include(FetchContent)
FetchContent_Declare(
  rigidbody
  GIT_REPOSITORY https://github.com/vcampos4545/rigidbody.git
  GIT_TAG main
)
FetchContent_MakeAvailable(rigidbody)

target_link_libraries(your_app PRIVATE rigidbody)
```

### Git submodule

```bash
git submodule add https://github.com/vcampos4545/rigidbody.git external/rigidbody
```

```cmake
add_subdirectory(external/rigidbody)
target_link_libraries(your_app PRIVATE rigidbody)
```

Either way, only the `rigidbody` library builds — pulling it in as a dependency does not build VGL or the example executables (and so doesn't need glfw/glew), even though this same repo builds all of that when you build it directly. If you do want the examples available from a project that depends on this one, set `RIGIDBODY_BUILD_EXAMPLES=ON`.

## Quick start

```cpp
#include <rigidbody/all.h>

int main()
{
  PhysicsWorld world;
  world.addGlobalForceGenerator(std::make_unique<UniformGravity>(glm::vec3(0.0f, 0.0f, -9.81f)));

  RigidBody *box = world.createBody(RigidBodyShape::BOX, {1, 1, 1}, 10.0f);
  box->position.z = 5.0f;

  while (running)
  {
    world.step(dt);
    // box->position / box->orientation are updated — render or log them
  }
}
```

`#include <rigidbody/all.h>` pulls in every header; feel free to include only the ones you need (e.g. `<rigidbody/PhysicsWorld.h>`) instead.

## Building this repo's examples

```bash
cd build && cmake .. && cmake --build .
```

This builds VGL (from a local `../VGL` checkout if present, otherwise via `FetchContent`) and the full set of example executables listed above (`orbit-validation`, `bars`, `solar-panel-deploy`, `chain`, `telescoping-boom`, `cubesat-pyramid`, `starship`, `starship-landing`, `robot-arm`, and — on supported platforms — `sitl`).

## Building your own simulation

A simulation is a `PhysicsWorld`, one or more `RigidBody`s with actuators attached, and your own flight-software or controller code that reads body state as sensor data and commands those actuators each frame. The engine has no concept of "spacecraft" or "sensor" — that's entirely up to your scenario. Every example in `examples/` follows the same four steps:

### 1. Define your vehicle

Create a `RigidBody` and attach actuators. Actuators are `ForceGenerator`s owned by the body once attached — `addForceGenerator` takes a `unique_ptr`, so keep the raw pointer around to command it later:

```cpp
RigidBody *body = world.createBody(RigidBodyShape::BOX, {0.1f, 0.1f, 0.1f}, 1.33f);

std::vector<ReactionWheel *> wheels;
glm::vec3 axes[3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
for (auto &axis : axes)
{
  auto wheel = std::make_unique<ReactionWheel>(glm::vec3(0.0f), axis, 0.001f /* max torque, Nm */);
  wheels.push_back(wheel.get());
  body->addForceGenerator(std::move(wheel));
}
```

`ReactionWheel`s are stepped automatically every `world.step(dt)` (they're `ForceGenerator`s). `Thruster`s are throttled/fired directly instead — see `examples/starship/` for the pattern (`thruster.apply(body, throttle)` called once per frame).

### 2. Write your flight software or controller

A small class holding references to the body and its actuators, turning "read sensors" into "command actuators." This is entirely your code — model it on `examples/cubesat_pyramid/` for a worked example:

```cpp
class MyFSW
{
public:
  MyFSW(RigidBody *body, std::vector<ReactionWheel *> wheels)
      : body(body), wheels(std::move(wheels)) {}

  void run(float dt)
  {
    // "Read sensors" — ground-truth body state for now; add your own
    // sensor noise / estimation model here if your scenario needs one.
    glm::quat attitude = body->orientation;
    glm::vec3 rate     = body->angularVelocity;

    // ... compute a torque command from attitude/rate ...

    // "Command actuators"
    for (auto *wheel : wheels)
      wheel->commandTorque(/* ... */);
  }

private:
  RigidBody *body;
  std::vector<ReactionWheel *> wheels;
};
```

### 3. Drive it from your main loop

Run your flight software, then step the physics — every scenario in `examples/` follows this shape:

```cpp
MyFSW fsw(body, wheels);

while (running)
{
  fsw.run(dt);     // read sensors, command actuators
  world.step(dt);  // integrate physics (applies actuator forces/torques)
}
```

### 4. Add it to the build

New simulations live in their own folder under `examples/<name>/`. Wire it up as a new `add_executable` in the top-level `CMakeLists.txt` inside the `RIGIDBODY_BUILD_EXAMPLES` block — see any existing target for the pattern. Rendering is optional; the physics side works fine headless.
