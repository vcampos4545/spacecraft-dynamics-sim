#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "vgl::vgl" for configuration ""
set_property(TARGET vgl::vgl APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(vgl::vgl PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_NOCONFIG "CXX"
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libvgl.a"
  )

list(APPEND _cmake_import_check_targets vgl::vgl )
list(APPEND _cmake_import_check_files_for_vgl::vgl "${_IMPORT_PREFIX}/lib/libvgl.a" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
