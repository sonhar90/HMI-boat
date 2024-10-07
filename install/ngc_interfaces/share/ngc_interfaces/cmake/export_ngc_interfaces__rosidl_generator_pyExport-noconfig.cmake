#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "ngc_interfaces::ngc_interfaces__rosidl_generator_py" for configuration ""
set_property(TARGET ngc_interfaces::ngc_interfaces__rosidl_generator_py APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(ngc_interfaces::ngc_interfaces__rosidl_generator_py PROPERTIES
  IMPORTED_LINK_DEPENDENT_LIBRARIES_NOCONFIG "ngc_interfaces::ngc_interfaces__rosidl_generator_c;Python3::Python;ngc_interfaces::ngc_interfaces__rosidl_typesupport_c"
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libngc_interfaces__rosidl_generator_py.so"
  IMPORTED_SONAME_NOCONFIG "libngc_interfaces__rosidl_generator_py.so"
  )

list(APPEND _cmake_import_check_targets ngc_interfaces::ngc_interfaces__rosidl_generator_py )
list(APPEND _cmake_import_check_files_for_ngc_interfaces::ngc_interfaces__rosidl_generator_py "${_IMPORT_PREFIX}/lib/libngc_interfaces__rosidl_generator_py.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
