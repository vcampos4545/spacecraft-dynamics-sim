# rigidbody

A lightweight rigid-body physics library, originally built for spacecraft simulation but not specific to it — anything that's a rigid body with actuators attached fits.

It gives you:

- **`RigidBody`** — box/sphere/cylinder/cone shapes, mass & inertia computed from density or mass, GJK+EPA convex collision, ground contact
- **`PhysicsWorld`** — fixed-timestep stepping (120 Hz internally, accumulator-driven), body-body and ground collision resolution, constraint solving
- **`Constraint`** — four primitives, each locking a specific set of the 6 relative degrees of freedom between two bodies: `FixedConstraint` (Weld — locks all 6, e.g. a nose cone bolted to a rocket), `PointConstraint` (Point-to-Point / Ball Socket — locks the 3 translational DOF, free rotation), `HingeConstraint` (Revolute — 1 free rotational DOF, with an optional angle limit and motor, e.g. a deployable solar panel), and `SliderConstraint` (Prismatic — 1 free translational DOF, with the same limit/motor pattern, e.g. a piston). They share their underlying math, so more elaborate joints can be built by combining them — a universal joint is two `HingeConstraint`s sharing a pivot, for instance. `DistanceConstraint` (a rope/strut held at a target distance rather than coincident, optionally unilateral like a string) is a fifth, distinct constraint kept alongside these four.
- **Actuators**, as `ForceGenerator`s attached to a body — `ReactionWheel` (torque + saturation) and `Thruster` (gimbaled, throttled)

See [`examples/`](examples/) for complete simulations that show how the pieces fit together — including a full attitude-control flight-software stack for the cubesat.

| Example                                                     | What it demonstrates                                                                                                                                                         |
| ------------------------------------------------------------ | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| [`cubesat`](examples/cubesat/)                                | A 1U cubesat with 3-axis reaction wheels, driven by scenario-owned FSW (`ADCS.h/.cpp` + `Controllers.h/.cpp`: guidance → PID/LQR/cascaded control → wheel torque allocation) |
| [`falcon9`](examples/falcon9/)                                | A multi-body rocket (cylinder + nose cone + 4 landing legs, connected with `FixedConstraint`s) with gimbaled `Thruster`s and an inline guidance controller                  |
| [`bars`](examples/bars/)                                      | A 5-level Calder mobile built entirely from `DistanceConstraint` — no flight software, just constraints                                                                     |
| [`solar_panel_deploy`](examples/solar_panel_deploy/)          | Two panels hinged to a bus with `HingeConstraint`, each independently deployable (press 1 / 2) via a motor driving against an angle limit                                   |
| [`chain`](examples/chain/)                                    | A rigid multi-link chain built from `PointConstraint`s, released from a swept-back pose to swing freely under gravity                                                       |
| [`telescoping_boom`](examples/telescoping_boom/)              | An antenna boom on a `SliderConstraint`, driven continuously by the operator (hold Up/Down) rather than deploying once to a limit                                           |

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
  GIT_REPOSITORY https://github.com/vcampos4545/spacecraft-dynamics-sim.git
  GIT_TAG main
)
FetchContent_MakeAvailable(rigidbody)

target_link_libraries(your_app PRIVATE rigidbody)
```

### Git submodule

```bash
git submodule add https://github.com/vcampos4545/spacecraft-dynamics-sim.git external/rigidbody
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
  world.gravity = {0.0f, 0.0f, -9.81f};

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

This builds VGL (from a local `../VGL` checkout if present, otherwise via `FetchContent`) and produces three executables: `cubesat`, `falcon-9`, `bars`.

## Building your own simulation

A simulation is a `PhysicsWorld`, one or more `RigidBody`s with actuators attached, and your own flight-software code that reads body state as sensor data and commands those actuators each frame. The engine has no concept of "spacecraft" or "sensor" — that's entirely up to your scenario. Every example in `examples/` follows the same four steps:

### 1. Define your spacecraft

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

`ReactionWheel`s are stepped automatically every `world.step(dt)` (they're `ForceGenerator`s). `Thruster`s are throttled/fired directly instead — see `examples/falcon9/falcon-9.cpp` for the pattern (`thruster.apply(body, throttle)` called once per frame).

### 2. Write your flight software

A small class holding references to the body and its actuators, turning "read sensors" into "command actuators." This is entirely your code — model it on `examples/cubesat/ADCS.h/.cpp` for a worked example:

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

New simulations live in their own folder under `examples/<name>/`. Wire it up as a new `add_executable` in the top-level `CMakeLists.txt` inside the `RIGIDBODY_BUILD_EXAMPLES` block — see the `cubesat`, `falcon-9`, and `bars` targets for the pattern. Rendering is optional; the physics side works fine headless.
