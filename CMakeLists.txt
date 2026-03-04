cmake_minimum_required(VERSION 3.16)
project(rigid-body)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Use local VGL for development (switch back to FetchContent for distribution)
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

# ---------------------------------------------------------------------------
# Shared source/include sets
# ---------------------------------------------------------------------------

# Core physics (no spacecraft or FSW)
set(PHYSICS_SOURCES
    dynamics/RigidBody.cpp
    dynamics/PhysicsWorld.cpp
    dynamics/Constraint.cpp
    dynamics/Universe.cpp
    dynamics/actuators/ReactionWheel.cpp
    dynamics/actuators/Thruster.cpp
)

set(PHYSICS_INCLUDE_DIRS
    ${CMAKE_CURRENT_SOURCE_DIR}/dynamics
    ${CMAKE_CURRENT_SOURCE_DIR}/dynamics/actuators
)

# Spacecraft + FSW (satellite, ADCS, controllers) layered on top of physics
set(SPACECRAFT_SOURCES
    fsw/Controllers.cpp
    spacecraft/Satellite.cpp
    spacecraft/subsystems/ADCS.cpp
)

set(SPACECRAFT_INCLUDE_DIRS
    ${PHYSICS_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/spacecraft
    ${CMAKE_CURRENT_SOURCE_DIR}/spacecraft/subsystems
    ${CMAKE_CURRENT_SOURCE_DIR}/fsw
)

# ---------------------------------------------------------------------------
# Targets
# ---------------------------------------------------------------------------

add_executable(main
    cubesat.cpp
    ${PHYSICS_SOURCES}
    ${SPACECRAFT_SOURCES}
)
target_include_directories(main PUBLIC ${SPACECRAFT_INCLUDE_DIRS})
target_link_libraries(main vgl)

add_executable(falcon-9
    falcon-9.cpp
    ${PHYSICS_SOURCES}
)
target_include_directories(falcon-9 PUBLIC ${PHYSICS_INCLUDE_DIRS})
target_link_libraries(falcon-9 vgl)

add_executable(bars
    bars.cpp
    ${PHYSICS_SOURCES}
)
target_include_directories(bars PUBLIC ${PHYSICS_INCLUDE_DIRS})
target_link_libraries(bars vgl)

add_executable(orbital-cubesat
    orbital_cubesat.cpp
    ${PHYSICS_SOURCES}
    ${SPACECRAFT_SOURCES}
)
target_include_directories(orbital-cubesat PUBLIC ${SPACECRAFT_INCLUDE_DIRS})
target_link_libraries(orbital-cubesat vgl)
