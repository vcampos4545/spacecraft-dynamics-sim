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
  set(VGL_LOCAL_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../VGL")
  if(EXISTS "${VGL_LOCAL_PATH}/CMakeLists.txt")
    add_subdirectory(${VGL_LOCAL_PATH} vgl_build)
  else()
    include(FetchContent)
    FetchContent_Declare(
      vgl
      GIT_REPOSITORY https://github.com/vcampos4545/VGL.git
      GIT_TAG main
    )
    FetchContent_MakeAvailable(vgl)
  endif()

  add_executable(cubesat
      examples/cubesat/cubesat.cpp
      examples/cubesat/ADCS.cpp
      examples/cubesat/Controllers.cpp
  )
  target_include_directories(cubesat PRIVATE examples/cubesat)
  target_link_libraries(cubesat PRIVATE rigidbody vgl)

  add_executable(falcon-9
      examples/falcon9/falcon-9.cpp
  )
  target_link_libraries(falcon-9 PRIVATE rigidbody vgl)

  add_executable(bars
      examples/bars/bars.cpp
  )
  target_link_libraries(bars PRIVATE rigidbody vgl)

  add_executable(solar-panel-deploy
      examples/solar_panel_deploy/solar_panel_deploy.cpp
  )
  target_link_libraries(solar-panel-deploy PRIVATE rigidbody vgl)
endif()
