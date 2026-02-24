cmake_minimum_required(VERSION 3.16)
project(rigid-body)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Fetch VGL library from GitHub
include(FetchContent)
FetchContent_Declare(
  vgl
  GIT_REPOSITORY https://github.com/vcampos4545/VGL.git
  GIT_TAG main
)
FetchContent_MakeAvailable(vgl)

add_executable(main
    cubesat.cpp
    fsw/Controllers.cpp
    spacecraft/Satellite.cpp
    spacecraft/subsystems/ADCS.cpp
    dynamics/RigidBody.cpp
    dynamics/PhysicsWorld.cpp
    dynamics/actuators/ReactionWheel.cpp
    dynamics/actuators/Thruster.cpp
)

target_include_directories(main PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/spacecraft
    ${CMAKE_CURRENT_SOURCE_DIR}/spacecraft/subsystems
    ${CMAKE_CURRENT_SOURCE_DIR}/fsw
    ${CMAKE_CURRENT_SOURCE_DIR}/dynamics
    ${CMAKE_CURRENT_SOURCE_DIR}/dynamics/actuators
)

target_link_libraries(main vaughngl)

# Physics sources shared between simulations
set(PHYSICS_SOURCES
    dynamics/RigidBody.cpp
    dynamics/PhysicsWorld.cpp
    dynamics/actuators/ReactionWheel.cpp
    dynamics/actuators/Thruster.cpp
)

set(PHYSICS_INCLUDE_DIRS
    ${CMAKE_CURRENT_SOURCE_DIR}/dynamics
    ${CMAKE_CURRENT_SOURCE_DIR}/dynamics/actuators
)

add_executable(falcon-9
    falcon-9.cpp
    ${PHYSICS_SOURCES}
)

target_include_directories(falcon-9 PUBLIC ${PHYSICS_INCLUDE_DIRS})
target_link_libraries(falcon-9 vaughngl)