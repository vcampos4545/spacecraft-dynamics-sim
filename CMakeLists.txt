cmake_minimum_required(VERSION 3.16)
project(rigidbody)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(glm REQUIRED)

# ---------------------------------------------------------------------------
# rigidbody — lightweight rigid-body physics library.
#
# No rendering, no gravity model, no flight software: just bodies,
# actuators (ForceGenerators), constraints, and a PhysicsWorld to step them.
# Scenarios (see examples/) define their own spacecraft, sensing, and FSW
# on top of this.
# ---------------------------------------------------------------------------
add_library(rigidbody STATIC
    src/RigidBody.cpp
    src/PhysicsWorld.cpp
    src/Constraint.cpp
    src/actuators/ReactionWheel.cpp
    src/actuators/Thruster.cpp
    src/actuators/Magnetorquer.cpp
    src/environment/Gravity.cpp
    src/environment/Drag.cpp
    src/environment/MagneticField.cpp
    src/sensors/IMU.cpp
    src/sensors/Magnetometer.cpp
    src/sensors/StarTracker.cpp
    src/sensors/SunSensor.cpp
    src/power/SolarPanel.cpp
    src/power/Battery.cpp
)

target_include_directories(rigidbody PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(rigidbody PUBLIC glm::glm)

# ---------------------------------------------------------------------------
# Examples — each is a standalone scenario: define a spacecraft out of
# rigidbody primitives, write scenario-specific flight software that reads
# body state as sensors and commands actuators, then drive it from a
# PhysicsWorld. Rendering uses VGL; the library above does not depend on it.
#
# Only built when this is the top-level project (or RIGIDBODY_BUILD_EXAMPLES
# is forced ON), so consuming `rigidbody` via add_subdirectory/FetchContent
# from another project doesn't drag in VGL/glfw/glew.
# ---------------------------------------------------------------------------
if(CMAKE_CURRENT_SOURCE_DIR STREQUAL CMAKE_SOURCE_DIR)
  set(RIGIDBODY_IS_TOP_LEVEL TRUE)
else()
  set(RIGIDBODY_IS_TOP_LEVEL FALSE)
endif()
option(RIGIDBODY_BUILD_EXAMPLES "Build the rigidbody example scenarios (requires VGL, glfw, glew)" ${RIGIDBODY_IS_TOP_LEVEL})

if(RIGIDBODY_BUILD_EXAMPLES)
  include(FetchContent)

  # Always fetched from GitHub -- no local-checkout shortcut, so the build
  # is reproducible for anyone cloning this repo, not just on a machine
  # that happens to have a VGL checkout sitting in the right sibling
  # directory.
  FetchContent_Declare(
    vgl
    GIT_REPOSITORY https://github.com/vcampos4545/VGL.git
    GIT_TAG main
  )
  FetchContent_MakeAvailable(vgl)

  # Dear ImGui -- immediate-mode UI (telemetry panels, plots, etc.) some
  # scenarios draw. Deliberately fetched/built here, not inside VGL: VGL is
  # a generic rendering library and has no ImGui dependency at all. Any
  # scenario that wants panels links this `imgui` target directly and
  # brackets VGL's own frame calls with examples/common/ImGuiLayer.h.
  #
  # find_package(glfw3) here too (not just relying on VGL having already
  # found it): an imported target from find_package is only visible in the
  # directory scope that found it and that scope's own subdirectories --
  # VGL's copy of `glfw` lives in FetchContent's vgl subdirectory, which is
  # a *child* of this scope, not a parent, so it isn't visible here without
  # finding it again at this level too.
  find_package(glfw3 REQUIRED)

  FetchContent_Declare(
    imgui
    GIT_REPOSITORY https://github.com/ocornut/imgui.git
    GIT_TAG v1.91.7
  )
  FetchContent_MakeAvailable(imgui)

  add_library(imgui STATIC
    ${imgui_SOURCE_DIR}/imgui.cpp
    ${imgui_SOURCE_DIR}/imgui_demo.cpp
    ${imgui_SOURCE_DIR}/imgui_draw.cpp
    ${imgui_SOURCE_DIR}/imgui_tables.cpp
    ${imgui_SOURCE_DIR}/imgui_widgets.cpp
    ${imgui_SOURCE_DIR}/backends/imgui_impl_glfw.cpp
    ${imgui_SOURCE_DIR}/backends/imgui_impl_opengl3.cpp
  )
  target_include_directories(imgui PUBLIC
    ${imgui_SOURCE_DIR}
    ${imgui_SOURCE_DIR}/backends
  )
  target_link_libraries(imgui PUBLIC glfw)

  # examples/common/ holds shared scene-dressing code (World.h) used by
  # every scenario below.
  set(EXAMPLES_COMMON_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/examples)

  add_executable(bars
      examples/bars/bars.cpp
  )
  target_include_directories(bars PRIVATE ${EXAMPLES_COMMON_INCLUDE_DIR})
  target_link_libraries(bars PRIVATE rigidbody vgl)

  add_executable(solar-panel-deploy
      examples/solar_panel_deploy/solar_panel_deploy.cpp
  )
  target_include_directories(solar-panel-deploy PRIVATE ${EXAMPLES_COMMON_INCLUDE_DIR})
  target_link_libraries(solar-panel-deploy PRIVATE rigidbody vgl)

  add_executable(chain
      examples/chain/chain.cpp
  )
  target_include_directories(chain PRIVATE ${EXAMPLES_COMMON_INCLUDE_DIR})
  target_link_libraries(chain PRIVATE rigidbody vgl)

  add_executable(telescoping-boom
      examples/telescoping_boom/telescoping_boom.cpp
  )
  target_include_directories(telescoping-boom PRIVATE ${EXAMPLES_COMMON_INCLUDE_DIR})
  target_link_libraries(telescoping-boom PRIVATE rigidbody vgl)

  add_executable(cubesat-pyramid
      examples/cubesat_pyramid/cubesat_pyramid.cpp
      examples/cubesat_pyramid/ADCS.cpp
      examples/cubesat_pyramid/Controllers.cpp
  )
  target_include_directories(cubesat-pyramid PRIVATE examples/cubesat_pyramid ${EXAMPLES_COMMON_INCLUDE_DIR})
  target_link_libraries(cubesat-pyramid PRIVATE rigidbody vgl imgui)

  add_executable(starship
      examples/starship/launch.cpp
      examples/starship/Booster.cpp
      examples/starship/Starship.cpp
      examples/starship/FlightSoftware.cpp
  )
  target_include_directories(starship PRIVATE examples/starship ${EXAMPLES_COMMON_INCLUDE_DIR})
  target_link_libraries(starship PRIVATE rigidbody vgl imgui)

  # examples/starship/models/ holds the ship's .obj/.mtl; copied next to the
  # built executable so the runtime-relative "models/..." load path resolves.
  add_custom_command(TARGET starship POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
    ${CMAKE_CURRENT_SOURCE_DIR}/examples/starship/models $<TARGET_FILE_DIR:starship>/models
  )

  # Reuses Starship.cpp from examples/starship/ (the vehicle definition) and
  # the same models/ directory (single source of truth for the 86MB .obj)
  # rather than duplicating either.
  add_executable(starship-landing
      examples/starship_landing/landing.cpp
      examples/starship_landing/LandingSoftware.cpp
      examples/starship/Starship.cpp
  )
  target_include_directories(starship-landing PRIVATE
      examples/starship_landing examples/starship ${EXAMPLES_COMMON_INCLUDE_DIR})
  target_link_libraries(starship-landing PRIVATE rigidbody vgl imgui)

  add_custom_command(TARGET starship-landing POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
    ${CMAKE_CURRENT_SOURCE_DIR}/examples/starship/models $<TARGET_FILE_DIR:starship-landing>/models
  )

  # SITL: links the real reaction_wheel_balancer firmware's portable control
  # code (BalanceController.cpp, IMUFilter.cpp) directly into a scenario
  # here, unmodified, from wherever that repo actually lives on disk --
  # same "local sibling checkout, path is overridable" shape as VGL_LOCAL_PATH
  # above, since it's the same kind of problem (an external project this one
  # optionally builds against). Only built if that repo is actually present;
  # everything else in this project builds fine without it.
  set(SITL_FIRMWARE_DIR "$ENV{HOME}/Desktop/reaction_wheel_balancer" CACHE PATH
      "Path to a reaction_wheel_balancer checkout, for the sitl example")
  if(EXISTS "${SITL_FIRMWARE_DIR}/src/BalanceController.cpp")
    add_executable(sitl
        examples/sitl/sitl.cpp
        examples/sitl/arduino_shim/Adafruit_MPU6050.cpp
        ${SITL_FIRMWARE_DIR}/src/BalanceController.cpp
        ${SITL_FIRMWARE_DIR}/src/IMUFilter.cpp
    )
    target_include_directories(sitl PRIVATE
        examples/sitl/arduino_shim
        ${SITL_FIRMWARE_DIR}/src
        ${EXAMPLES_COMMON_INCLUDE_DIR}
    )
    target_link_libraries(sitl PRIVATE rigidbody vgl imgui)
  else()
    message(STATUS "sitl example skipped: no firmware found at ${SITL_FIRMWARE_DIR} "
                    "(set -DSITL_FIRMWARE_DIR=/path/to/reaction_wheel_balancer to build it)")
  endif()

  add_executable(robot-arm
      examples/robot_arm/robot_arm.cpp
      examples/robot_arm/ArmController.cpp
  )
  target_include_directories(robot-arm PRIVATE examples/robot_arm ${EXAMPLES_COMMON_INCLUDE_DIR})
  target_link_libraries(robot-arm PRIVATE rigidbody vgl imgui)
endif()
