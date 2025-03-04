#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "png16" for configuration "Release"
set_property(TARGET png16 APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(png16 PROPERTIES
  IMPORTED_IMPLIB_RELEASE "${_IMPORT_PREFIX}/lib/libpng16.lib"
  IMPORTED_LINK_INTERFACE_LIBRARIES_RELEASE "F:/laba_work/v2/ORB_SLAM2/orbslam-windows/Thirdparty/Pangolin/build/external/zlib/lib/zlibstatic*.lib"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/bin/libpng16.dll"
  )

list(APPEND _IMPORT_CHECK_TARGETS png16 )
list(APPEND _IMPORT_CHECK_FILES_FOR_png16 "${_IMPORT_PREFIX}/lib/libpng16.lib" "${_IMPORT_PREFIX}/bin/libpng16.dll" )

# Import target "png16_static" for configuration "Release"
set_property(TARGET png16_static APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(png16_static PROPERTIES
  IMPORTED_LINK_INTERFACE_LANGUAGES_RELEASE "C"
  IMPORTED_LINK_INTERFACE_LIBRARIES_RELEASE "F:/laba_work/v2/ORB_SLAM2/orbslam-windows/Thirdparty/Pangolin/build/external/zlib/lib/zlibstatic*.lib"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/libpng16_static.lib"
  )

list(APPEND _IMPORT_CHECK_TARGETS png16_static )
list(APPEND _IMPORT_CHECK_FILES_FOR_png16_static "${_IMPORT_PREFIX}/lib/libpng16_static.lib" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
