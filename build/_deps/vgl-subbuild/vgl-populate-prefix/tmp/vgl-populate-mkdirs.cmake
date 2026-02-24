# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file LICENSE.rst or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION ${CMAKE_VERSION}) # this file comes with cmake

# If CMAKE_DISABLE_SOURCE_CHANGES is set to true and the source directory is an
# existing directory in our source tree, calling file(MAKE_DIRECTORY) on it
# would cause a fatal error, even though it would be a no-op.
if(NOT EXISTS "/Users/vaughncampos/Desktop/rigid-body/build/_deps/vgl-src")
  file(MAKE_DIRECTORY "/Users/vaughncampos/Desktop/rigid-body/build/_deps/vgl-src")
endif()
file(MAKE_DIRECTORY
  "/Users/vaughncampos/Desktop/rigid-body/build/_deps/vgl-build"
  "/Users/vaughncampos/Desktop/rigid-body/build/_deps/vgl-subbuild/vgl-populate-prefix"
  "/Users/vaughncampos/Desktop/rigid-body/build/_deps/vgl-subbuild/vgl-populate-prefix/tmp"
  "/Users/vaughncampos/Desktop/rigid-body/build/_deps/vgl-subbuild/vgl-populate-prefix/src/vgl-populate-stamp"
  "/Users/vaughncampos/Desktop/rigid-body/build/_deps/vgl-subbuild/vgl-populate-prefix/src"
  "/Users/vaughncampos/Desktop/rigid-body/build/_deps/vgl-subbuild/vgl-populate-prefix/src/vgl-populate-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/Users/vaughncampos/Desktop/rigid-body/build/_deps/vgl-subbuild/vgl-populate-prefix/src/vgl-populate-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/Users/vaughncampos/Desktop/rigid-body/build/_deps/vgl-subbuild/vgl-populate-prefix/src/vgl-populate-stamp${cfgdir}") # cfgdir has leading slash
endif()
